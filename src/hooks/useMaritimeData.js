import { useState, useEffect, useRef, useCallback } from "react";
import WebSocketService from "../services/WebSocketService";
import { WS_URL, CAMERA_DEFAULTS } from "../config";

/**
 * React hook for consuming maritime sensor data via WebSocket.
 * Throttles updates to ~20fps to keep RPi rendering smooth.
 *
 * Returns the full telemetry state from ws_server.py:
 *   - proximity (distance, zone)
 *   - cv detections
 *   - gimbal angles
 *   - ladder distance + status
 *   - approach heading, zone, hull profile
 *   - boat tilt (pitch/roll)
 */
const useMaritimeData = () => {
  const [connected, setConnected] = useState(false);
  const [data, setData] = useState({
    // Proximity / LiDAR
    distance: null,
    zone: null,              // "TOO_CLOSE" | "OPTIMAL" | "TOO_FAR"
    proximityStatus: null,   // full JSON from /proximity/status
    // CV
    detections: [],
    sourceResolution: { w: 640, h: 480 },
    // Gimbal
    gimbalYaw: null,
    gimbalPitch: null,
    gimbalTargetLocked: false,    // /gimbal/target_locked — true when tracker has lock
    // Ladder
    ladderDistance: null,     // boarding gate distance (m)
    ladderStatus: null,      // full JSON from /ladder/status
    // Approach
    approachHeading: null,   // heading error degrees
    approachZone: null,      // "TOO_CLOSE" | "OPTIMAL" | "TOO_FAR"
    approachProfile: null,   // hull gap profile JSON
    // Boat tilt
    boatPitch: null,
    boatRoll: null,
    // Camera
    cameraUrls: CAMERA_DEFAULTS,
    // System phase
    phase: null,             // "APPROACHING" | "ZONING" | "HOLDING"
    // Spatial — cargo-side LiDAR points in boat frame + boat extents
    cargoPoints: { hull_x: [], gap: [] },
    boatGeometry: null,      // { bow_x, rear_x, starboard_y, lidar_x, lidar_y }
    // Jetson model
    currentModel: null,
    defaultModel: null,      // persisted user preference (next-boot model)
    // Meta
    alertLevel: "none",      // "none" | "danger"
    lastUpdate: null,
  });

  const wsRef = useRef(null);
  const rafRef = useRef(null);
  const pendingDataRef = useRef(null);
  const ladderLastSeenRef = useRef(0);       // timestamp of last valid ladder distance
  const ladderTimerRef = useRef(null);        // 3s expiry timer

  // Throttled state update via requestAnimationFrame
  const processUpdate = useCallback(() => {
    const msg = pendingDataRef.current;
    if (!msg) return;
    pendingDataRef.current = null;

    setData((prev) => {
      const zone = msg.lidar?.zone || prev.zone;
      // Only TOO_CLOSE triggers an alert — TOO_FAR is informational (approaching)
      const alertLevel = zone === "TOO_CLOSE" ? "danger" : "none";

      // Ladder 3s sliding window: update on new data, expire after 3s of silence
      const hasNewLadder = msg.ladder?.distance !== undefined && msg.ladder.distance !== null;
      let newLadderDistance = prev.ladderDistance;
      let newLadderStatus = prev.ladderStatus;
      if (hasNewLadder) {
        newLadderDistance = msg.ladder.distance;
        newLadderStatus = msg.ladder?.status !== undefined ? msg.ladder.status : prev.ladderStatus;
        ladderLastSeenRef.current = Date.now();
        // Reset the expiry timer
        if (ladderTimerRef.current) clearTimeout(ladderTimerRef.current);
        ladderTimerRef.current = setTimeout(() => {
          setData((p) => ({ ...p, ladderDistance: null, ladderStatus: null }));
          ladderTimerRef.current = null;
        }, 3000);
      }

      return {
        // Proximity
        distance: msg.lidar?.distance ?? prev.distance,
        zone: zone,
        proximityStatus: msg.lidar?.status || prev.proximityStatus,
        // CV
        detections: msg.cv_detections || prev.detections,
        sourceResolution: msg.source_resolution || prev.sourceResolution,
        // Gimbal
        gimbalYaw: msg.gimbal?.yaw ?? prev.gimbalYaw,
        gimbalPitch: msg.gimbal?.pitch ?? prev.gimbalPitch,
        gimbalTargetLocked: msg.gimbal?.target_locked ?? prev.gimbalTargetLocked,
        // Ladder — 3s sliding window: show latest, expire if no new data
        ladderDistance: newLadderDistance,
        ladderStatus: newLadderStatus,
        // Approach
        approachHeading: msg.approach?.heading ?? prev.approachHeading,
        approachZone: msg.approach?.zone || prev.approachZone,
        approachProfile: msg.approach?.profile || prev.approachProfile,
        // Boat
        boatPitch: msg.boat?.pitch ?? prev.boatPitch,
        boatRoll: msg.boat?.roll ?? prev.boatRoll,
        // Camera
        cameraUrls: {
          cam1: msg.camera_urls?.cam1 || prev.cameraUrls.cam1,
          cam2: msg.camera_urls?.cam2 || prev.cameraUrls.cam2,
        },
        // System phase
        phase: msg.phase || prev.phase,
        // Spatial
        cargoPoints: msg.spatial?.cargo_points || prev.cargoPoints,
        boatGeometry: msg.spatial?.boat_geometry || prev.boatGeometry,
        // Jetson model
        currentModel: msg.current_model || prev.currentModel,
        defaultModel: msg.default_model !== undefined ? msg.default_model : prev.defaultModel,
        // Meta
        alertLevel,
        lastUpdate: msg.timestamp || Date.now(),
      };
    });
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    const ws = new WebSocketService(WS_URL);
    wsRef.current = ws;

    ws.onStatusChange((isConnected) => {
      setConnected(isConnected);
      if (!isConnected) {
        setData((prev) => ({ ...prev, alertLevel: "none" }));
      }
    });

    ws.onMessage((msg) => {
      pendingDataRef.current = msg;
      if (!rafRef.current) {
        rafRef.current = requestAnimationFrame(() => {
          processUpdate();
          rafRef.current = null;
        });
      }
    });

    ws.connect();

    return () => {
      ws.disconnect();
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
      if (ladderTimerRef.current) clearTimeout(ladderTimerRef.current);
    };
  }, [processUpdate]);

  const send = useCallback((msg) => {
    wsRef.current?.send(msg);
  }, []);

  return {
    connected,
    send,
    ...data,
  };
};

export default useMaritimeData;
