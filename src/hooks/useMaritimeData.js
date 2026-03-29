import { useState, useEffect, useRef, useCallback } from "react";
import WebSocketService from "../services/WebSocketService";
import { WS_URL, CAMERA_DEFAULTS, DISTANCE_ZONES } from "../config";

/**
 * React hook for consuming maritime sensor data via WebSocket.
 * Throttles updates to ~20fps to keep RPi rendering smooth.
 */
const useMaritimeData = () => {
  const [connected, setConnected] = useState(false);
  const [distance, setDistance] = useState(null);
  const [detections, setDetections] = useState([]);
  const [sourceResolution, setSourceResolution] = useState({ w: 640, h: 480 });
  const [cameraUrls, setCameraUrls] = useState(CAMERA_DEFAULTS);
  const [alertLevel, setAlertLevel] = useState("none");
  const [lastUpdate, setLastUpdate] = useState(null);

  const wsRef = useRef(null);
  const rafRef = useRef(null);
  const pendingDataRef = useRef(null);

  // Compute alert level from distance
  const computeAlertLevel = useCallback((dist) => {
    if (dist === null) return "none";
    if (dist <= DISTANCE_ZONES.danger.max) return "danger";
    if (dist <= DISTANCE_ZONES.caution.max) return "caution";
    return "none";
  }, []);

  // Throttled state update via requestAnimationFrame
  const processUpdate = useCallback(() => {
    const data = pendingDataRef.current;
    if (!data) return;
    pendingDataRef.current = null;

    if (data.lidar?.distance !== undefined) {
      setDistance(data.lidar.distance);
      setAlertLevel(computeAlertLevel(data.lidar.distance));
    }

    if (data.cv_detections) {
      setDetections(data.cv_detections);
    }

    if (data.source_resolution) {
      setSourceResolution(data.source_resolution);
    }

    if (data.camera_urls) {
      setCameraUrls((prev) => ({
        cam1: data.camera_urls.cam1 || prev.cam1,
        cam2: data.camera_urls.cam2 || prev.cam2,
      }));
    }

    setLastUpdate(data.timestamp || Date.now());
  }, [computeAlertLevel]);

  useEffect(() => {
    const ws = new WebSocketService(WS_URL);
    wsRef.current = ws;

    ws.onStatusChange((isConnected) => {
      setConnected(isConnected);
      if (!isConnected) {
        setAlertLevel("none");
      }
    });

    ws.onMessage((data) => {
      pendingDataRef.current = data;
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
    };
  }, [processUpdate]);

  return {
    connected,
    distance,
    detections,
    sourceResolution,
    cameraUrls,
    alertLevel,
    lastUpdate,
  };
};

export default useMaritimeData;
