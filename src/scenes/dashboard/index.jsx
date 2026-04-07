import React, { useRef, useState, useEffect, useCallback } from "react";
import { Box, Typography } from "@mui/material";
import StatusBar from "../global/Topbar";
import CameraFeed from "../../components/CameraFeed";
import CVOverlay from "../../components/CVOverlay";
import DistanceStrip from "../../components/DistanceStrip";
import GuidancePanel from "../../components/GuidancePanel";
import { Cam2StatusFrame, Cam2ReferenceLine } from "../../components/Cam2Overlay";
import AlertBanner from "../../components/AlertBanner";
import PhaseOverlay from "../../components/PhaseOverlay";
import SpatialView from "../../components/SpatialView";
import DiagnosticsView from "../../components/DiagnosticsView";
import useMaritimeData from "../../hooks/useMaritimeData";

// Phase → accent color for camera container borders (smooth transition)
const PHASE_ACCENT = {
  APPROACHING: "#66778825",
  ZONING: "#dd880030",
  HOLDING: "#00ff8835",
};

// ── Camera split ratio — persists across sessions, adjustable in calibration mode
const SPLIT_KEY = "pyxis_cam_split";
const SPLIT_DEFAULT = 0.6;   // cam1 takes 60% by default
const SPLIT_MIN = 0.3;
const SPLIT_MAX = 0.8;

const loadSplitRatio = () => {
  try {
    const v = parseFloat(localStorage.getItem(SPLIT_KEY));
    if (!isNaN(v) && v >= SPLIT_MIN && v <= SPLIT_MAX) return v;
  } catch (e) {}
  return SPLIT_DEFAULT;
};

// ── Spatial tab: cam1 | spatial canvas split — independent of live tab
const SPATIAL_SPLIT_KEY = "pyxis_spatial_split";
const SPATIAL_SPLIT_DEFAULT = 0.45;  // cam1 45%, spatial canvas 55% by default
const SPATIAL_SPLIT_MIN = 0.25;
const SPATIAL_SPLIT_MAX = 0.7;

const loadSpatialSplitRatio = () => {
  try {
    const v = parseFloat(localStorage.getItem(SPATIAL_SPLIT_KEY));
    if (!isNaN(v) && v >= SPATIAL_SPLIT_MIN && v <= SPATIAL_SPLIT_MAX) return v;
  } catch (e) {}
  return SPATIAL_SPLIT_DEFAULT;
};

// ── Heading offset — manual "zero" correction, persists across sessions
const HEADING_OFFSET_KEY = "pyxis_heading_offset";

const loadHeadingOffset = () => {
  try {
    const v = parseFloat(localStorage.getItem(HEADING_OFFSET_KEY));
    if (!isNaN(v)) return v;
  } catch (e) {}
  return 0;
};

const LiveGuidanceView = ({
  cam1Ref,
  cam2Ref,
  cam1Dims,
  data,
  calibrationMode,
  splitRatio,
  setSplitRatio,
  resetSignal,
}) => {
  const phase = data.phase;
  const accent = PHASE_ACCENT[phase] || "transparent";
  const isApproaching = phase === "APPROACHING";

  const [splitDragging, setSplitDragging] = useState(false);
  const gridRef = useRef(null);

  useEffect(() => {
    if (!splitDragging) return;
    const onMove = (e) => {
      const clientX = e.clientX ?? e.touches?.[0]?.clientX;
      if (clientX === undefined) return;
      const el = gridRef.current;
      if (!el) return;
      const rect = el.getBoundingClientRect();
      const rel = (clientX - rect.left) / rect.width;
      setSplitRatio(Math.max(SPLIT_MIN, Math.min(SPLIT_MAX, rel)));
    };
    const onUp = () => setSplitDragging(false);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    window.addEventListener("touchmove", onMove);
    window.addEventListener("touchend", onUp);
    return () => {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      window.removeEventListener("touchmove", onMove);
      window.removeEventListener("touchend", onUp);
    };
  }, [splitDragging, setSplitRatio]);

  return (
    <Box sx={{ display: "flex", flexDirection: "column", flex: 1, minHeight: 0, position: "relative" }}>
      <DistanceStrip
        distance={data.distance}
        zone={data.zone}
        ladderDistance={data.ladderDistance}
      />
      <Box
        ref={gridRef}
        sx={{
          flex: 1,
          display: "grid",
          gridTemplateColumns: `${splitRatio}fr ${1 - splitRatio}fr`,
          gap: "2px",
          minHeight: 0,
          position: "relative",
        }}
      >
        {/* Camera 1 — Ladder View */}
        <Box
          sx={{
            position: "relative",
            overflow: "hidden",
            borderLeft: `3px solid ${accent}`,
            transition: "border-color 0.8s ease",
          }}
        >
          <CameraFeed
            ref={cam1Ref}
            source={{ type: "mjpeg", url: data.cameraUrls.cam1 }}
            label="Camera 1 — Ladder View"
          />
          <Box
            sx={{
              opacity: isApproaching ? 0.25 : 1,
              transition: "opacity 0.8s ease",
            }}
          >
            <CVOverlay
              detections={data.detections}
              width={cam1Dims.width}
              height={cam1Dims.height}
              sourceResolution={data.sourceResolution}
            />
          </Box>
          <PhaseOverlay phase={phase} distance={data.distance} position="cam1" />
        </Box>

        {/* Camera 2 — Position View */}
        <Box
          sx={{
            position: "relative",
            overflow: "hidden",
            borderRight: `3px solid ${accent}`,
            transition: "border-color 0.8s ease",
          }}
        >
          <CameraFeed
            ref={cam2Ref}
            source={{ type: "mjpeg", url: data.cameraUrls.cam2 }}
            label="Camera 2 — Position View"
          />
          <Cam2StatusFrame zone={data.zone} phase={phase} />
          <Cam2ReferenceLine
            calibrationMode={calibrationMode}
            zone={data.zone}
            phase={phase}
            resetSignal={resetSignal}
          />
          <GuidancePanel
            phase={phase}
            detections={data.detections}
            approachHeading={data.approachHeading}
            approachZone={data.approachZone}
            gimbalYaw={data.gimbalYaw}
            gimbalPitch={data.gimbalPitch}
            gimbalTargetLocked={data.gimbalTargetLocked}
            boatPitch={data.boatPitch}
            boatRoll={data.boatRoll}
          />
          <PhaseOverlay phase={phase} distance={data.distance} position="cam2" />
        </Box>

        {/* Camera splitter — draggable in calibration mode only */}
        {calibrationMode && (
          <Box
            onMouseDown={(e) => { e.preventDefault(); setSplitDragging(true); }}
            onTouchStart={(e) => { e.preventDefault(); setSplitDragging(true); }}
            sx={{
              position: "absolute",
              top: 0,
              bottom: 0,
              left: `${splitRatio * 100}%`,
              width: "10px",
              marginLeft: "-5px",
              cursor: "ew-resize",
              zIndex: 15,
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              backgroundColor: "#00b4d825",
              borderLeft: "1px solid #00b4d8aa",
              borderRight: "1px solid #00b4d8aa",
              transition: "background-color 0.2s ease",
              "&:hover": { backgroundColor: "#00b4d850" },
            }}
          >
            <Box
              sx={{
                width: "26px",
                height: "60px",
                borderRadius: "4px",
                backgroundColor: "#00b4d8",
                border: "2px solid #ffffff",
                display: "flex",
                alignItems: "center",
                justifyContent: "center",
                boxShadow: "0 0 14px #00b4d8",
                pointerEvents: "none",
              }}
            >
              <Typography sx={{ fontSize: "15px", color: "#0a0e1a", fontWeight: 900 }}>
                ⇔
              </Typography>
            </Box>
          </Box>
        )}
      </Box>
    </Box>
  );
};

// ── Spatial tab view: cam1 | spatial canvas with draggable divider ──
const SpatialSplitView = ({
  cam1Ref,
  cam1Dims,
  data,
  calibrationMode,
  splitRatio,
  setSplitRatio,
}) => {
  const [splitDragging, setSplitDragging] = useState(false);
  const gridRef = useRef(null);

  useEffect(() => {
    if (!splitDragging) return;
    const onMove = (e) => {
      const clientX = e.clientX ?? e.touches?.[0]?.clientX;
      if (clientX === undefined) return;
      const el = gridRef.current;
      if (!el) return;
      const rect = el.getBoundingClientRect();
      const rel = (clientX - rect.left) / rect.width;
      setSplitRatio(Math.max(SPATIAL_SPLIT_MIN, Math.min(SPATIAL_SPLIT_MAX, rel)));
    };
    const onUp = () => setSplitDragging(false);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    window.addEventListener("touchmove", onMove);
    window.addEventListener("touchend", onUp);
    return () => {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      window.removeEventListener("touchmove", onMove);
      window.removeEventListener("touchend", onUp);
    };
  }, [splitDragging, setSplitRatio]);

  return (
    <Box
      ref={gridRef}
      sx={{
        flex: 1,
        display: "grid",
        gridTemplateColumns: `${splitRatio}fr ${1 - splitRatio}fr`,
        gap: "2px",
        minHeight: 0,
        position: "relative",
      }}
    >
      {/* Camera 1 — Ladder view (same feed as Live tab) */}
      <Box sx={{ position: "relative", overflow: "hidden", backgroundColor: "#000" }}>
        <CameraFeed
          ref={cam1Ref}
          source={{ type: "mjpeg", url: data.cameraUrls.cam1 }}
          label="Camera 1 — Ladder View"
        />
        <CVOverlay
          detections={data.detections}
          width={cam1Dims.width}
          height={cam1Dims.height}
          sourceResolution={data.sourceResolution}
        />
      </Box>

      {/* Spatial canvas */}
      <Box sx={{ position: "relative", minHeight: 0, display: "flex" }}>
        <SpatialView
          distance={data.distance}
          zone={data.zone}
          ladderDistance={data.ladderDistance}
          approachHeading={data.approachHeading}
          approachZone={data.approachZone}
          approachProfile={data.approachProfile}
          gimbalYaw={data.gimbalYaw}
          boatPitch={data.boatPitch}
          boatRoll={data.boatRoll}
          gimbalTargetLocked={data.gimbalTargetLocked}
          phase={data.phase}
          cargoPoints={data.cargoPoints}
          boatGeometry={data.boatGeometry}
        />
      </Box>

      {/* Draggable divider — calibration mode only */}
      {calibrationMode && (
        <Box
          onMouseDown={(e) => { e.preventDefault(); setSplitDragging(true); }}
          onTouchStart={(e) => { e.preventDefault(); setSplitDragging(true); }}
          sx={{
            position: "absolute",
            top: 0,
            bottom: 0,
            left: `${splitRatio * 100}%`,
            width: "10px",
            marginLeft: "-5px",
            cursor: "ew-resize",
            zIndex: 15,
            display: "flex",
            alignItems: "center",
            justifyContent: "center",
            backgroundColor: "#00b4d825",
            borderLeft: "1px solid #00b4d8aa",
            borderRight: "1px solid #00b4d8aa",
            transition: "background-color 0.2s ease",
            "&:hover": { backgroundColor: "#00b4d850" },
          }}
        >
          <Box
            sx={{
              width: "26px",
              height: "60px",
              borderRadius: "4px",
              backgroundColor: "#00b4d8",
              border: "2px solid #ffffff",
              display: "flex",
              alignItems: "center",
              justifyContent: "center",
              boxShadow: "0 0 14px #00b4d8",
              pointerEvents: "none",
            }}
          >
            <Typography sx={{ fontSize: "15px", color: "#0a0e1a", fontWeight: 900 }}>
              ⇔
            </Typography>
          </Box>
        </Box>
      )}
    </Box>
  );
};

// ── Test Mode: simulates distance/phase/heading for indoor testing ──
// Multiple independent "tracks" stacked above the TEST button. Each track
// has its own NEXT button so scenarios and heading can be cycled separately.
const TEST_SCENARIOS = [
  { label: "6.0m APPROACHING", distance: 6.0, zone: "TOO_FAR",   phase: "APPROACHING" },
  { label: "2.0m ZONING",      distance: 2.0, zone: "TOO_FAR",   phase: "ZONING" },
  { label: "1.5m ZONING",      distance: 1.5, zone: "TOO_FAR",   phase: "ZONING" },
  { label: "1.0m ZONING",      distance: 1.0, zone: "OPTIMAL",   phase: "ZONING" },
  { label: "0.8m HOLDING",     distance: 0.8, zone: "OPTIMAL",   phase: "HOLDING" },
  { label: "0.5m HOLDING",     distance: 0.5, zone: "OPTIMAL",   phase: "HOLDING" },
  { label: "0.2m DANGER",      distance: 0.2, zone: "TOO_CLOSE", phase: "ZONING" },
];

const HEADING_TEST_VALUES = [
  { label: "HDG  0°",   value: 0 },
  { label: "HDG +5°",   value: 5 },
  { label: "HDG +10°",  value: 10 },
  { label: "HDG +20°",  value: 20 },
  { label: "HDG +45°",  value: 45 },
  { label: "HDG -5°",   value: -5 },
  { label: "HDG -10°",  value: -10 },
  { label: "HDG -20°",  value: -20 },
  { label: "HDG -45°",  value: -45 },
];

const TestTrackRow = ({ label, onNext }) => (
  <Box sx={{ display: "flex", gap: 1, alignItems: "center", justifyContent: "flex-end" }}>
    <Box
      sx={{
        px: 1.5, py: 0.5,
        backgroundColor: "#0d1117ee",
        border: "1px solid #ffaa0050",
        borderRadius: "4px",
      }}
    >
      <Typography sx={{
        fontFamily: "'JetBrains Mono', monospace",
        fontSize: "13px", color: "#ffaa00", fontWeight: 600,
      }}>
        {label}
      </Typography>
    </Box>
    <Box
      onClick={onNext}
      sx={{
        px: 1.5, py: 0.5,
        backgroundColor: "#1a2035",
        border: "1px solid #ffaa0050",
        borderRadius: "4px",
        cursor: "pointer",
        "&:hover": { backgroundColor: "#2a3045" },
      }}
    >
      <Typography sx={{ fontSize: "13px", color: "#ffaa00", fontWeight: 700 }}>
        NEXT
      </Typography>
    </Box>
  </Box>
);

const TestModeButton = ({
  testMode,
  testIndex,
  testHeadingIndex,
  onToggle,
  onNextScenario,
  onNextHeading,
}) => (
  <Box
    sx={{
      position: "fixed",
      bottom: 12,
      right: 12,
      zIndex: 10000,
      display: "flex",
      flexDirection: "column-reverse",
      gap: 1,
      alignItems: "flex-end",
    }}
  >
    {/* Bottom row: master toggle */}
    <Box
      onClick={onToggle}
      sx={{
        px: 1.5, py: 0.5,
        backgroundColor: testMode ? "#ffaa0020" : "#1a2035",
        border: `1px solid ${testMode ? "#ffaa0060" : "#33333360"}`,
        borderRadius: "4px",
        cursor: "pointer",
        "&:hover": { backgroundColor: testMode ? "#ffaa0030" : "#2a3045" },
      }}
    >
      <Typography sx={{
        fontSize: "13px", fontWeight: 700,
        color: testMode ? "#ffaa00" : "#556",
      }}>
        {testMode ? "TEST ON" : "TEST"}
      </Typography>
    </Box>

    {/* Stacked tracks populate upward when TEST is on */}
    {testMode && (
      <TestTrackRow
        label={TEST_SCENARIOS[testIndex].label}
        onNext={onNextScenario}
      />
    )}
    {testMode && (
      <TestTrackRow
        label={HEADING_TEST_VALUES[testHeadingIndex].label}
        onNext={onNextHeading}
      />
    )}
  </Box>
);

const REMOTE_CHANNEL = "pyxis-remote";

const PilotDashboard = () => {
  const cam1Ref = useRef(null);
  const cam2Ref = useRef(null);
  const [cam1Dims, setCam1Dims] = useState({ width: 0, height: 0 });
  const [activeView, setActiveView] = useState("live");
  const [headingOffset, setHeadingOffset] = useState(loadHeadingOffset);

  // Persist heading offset to localStorage
  useEffect(() => {
    try { localStorage.setItem(HEADING_OFFSET_KEY, String(headingOffset)); } catch (e) {}
  }, [headingOffset]);
  const [testMode, setTestMode] = useState(false);
  const [testIndex, setTestIndex] = useState(0);
  const [testHeadingIndex, setTestHeadingIndex] = useState(0);
  const [calibrationMode, setCalibrationMode] = useState(false);

  // Hoisted calibration state — split ratios for Live + Spatial tabs,
  // and a reset signal consumed by Cam2ReferenceLine to re-seed its points.
  const [liveSplitRatio, setLiveSplitRatio] = useState(loadSplitRatio);
  const [spatialSplitRatio, setSpatialSplitRatio] = useState(loadSpatialSplitRatio);
  const [resetSignal, setResetSignal] = useState(0);

  // Persist split ratios on change (debounced by React render batching — fine
  // for drag updates since each mousemove triggers at most one localStorage write).
  useEffect(() => {
    try { localStorage.setItem(SPLIT_KEY, String(liveSplitRatio)); } catch (e) {}
  }, [liveSplitRatio]);
  useEffect(() => {
    try { localStorage.setItem(SPATIAL_SPLIT_KEY, String(spatialSplitRatio)); } catch (e) {}
  }, [spatialSplitRatio]);

  const handleResetCalibration = useCallback(() => {
    setLiveSplitRatio(SPLIT_DEFAULT);
    setSpatialSplitRatio(SPATIAL_SPLIT_DEFAULT);
    setResetSignal((n) => n + 1);
  }, []);

  const rawData = useMaritimeData();

  // ── BroadcastChannel: receive commands from remote control panel ──
  const remoteChannelRef = useRef(null);
  const rawHeadingRef = useRef(rawData.approachHeading);
  rawHeadingRef.current = rawData.approachHeading;

  useEffect(() => {
    const ch = new BroadcastChannel(REMOTE_CHANNEL);
    remoteChannelRef.current = ch;

    ch.onmessage = (e) => {
      const msg = e.data;
      switch (msg.type) {
        case "view_change":
          setActiveView(msg.view);
          break;
        case "test_toggle":
          setTestMode((prev) => !prev);
          setTestIndex(0);
          setTestHeadingIndex(0);
          break;
        case "test_next_scenario":
          setTestIndex((prev) => (prev + 1) % TEST_SCENARIOS.length);
          break;
        case "test_next_heading":
          setTestHeadingIndex((prev) => (prev + 1) % HEADING_TEST_VALUES.length);
          break;
        case "calibration_toggle":
          setCalibrationMode((v) => !v);
          break;
        case "calibration_reset":
          handleResetCalibration();
          break;
        case "zero_heading":
          if (rawHeadingRef.current !== null) {
            setHeadingOffset(rawHeadingRef.current);
          }
          break;
        case "reset_heading":
          setHeadingOffset(0);
          break;
        default:
          break;
      }
    };

    return () => ch.close();
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [handleResetCalibration]);

  // Sync state to remote whenever relevant state changes
  useEffect(() => {
    remoteChannelRef.current?.postMessage({
      type: "state_sync",
      activeView,
      testMode,
      calibrationMode,
      connected: rawData.connected,
      phase: rawData.phase,
    });
  }, [activeView, testMode, calibrationMode, rawData.connected, rawData.phase]);

  // Apply heading offset — all views see the corrected heading
  const liveData = {
    ...rawData,
    approachHeading: rawData.approachHeading !== null
      ? rawData.approachHeading - headingOffset
      : null,
  };

  // Test mode overrides distance, zone, phase, and heading — each track
  // is cycled independently so bench tests can vary one dimension at a time.
  const data = testMode
    ? {
        ...liveData,
        distance: TEST_SCENARIOS[testIndex].distance,
        zone: TEST_SCENARIOS[testIndex].zone,
        phase: TEST_SCENARIOS[testIndex].phase,
        alertLevel: TEST_SCENARIOS[testIndex].zone === "TOO_CLOSE" ? "danger" : "none",
        approachHeading: HEADING_TEST_VALUES[testHeadingIndex].value,
      }
    : liveData;

  const handleZeroHeading = useCallback(() => {
    if (rawData.approachHeading !== null) {
      setHeadingOffset(rawData.approachHeading);
    }
  }, [rawData.approachHeading]);

  const handleResetHeading = useCallback(() => {
    setHeadingOffset(0);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => {
      if (cam1Ref.current?.getDimensions) {
        const dims = cam1Ref.current.getDimensions();
        setCam1Dims((prev) =>
          prev.width !== dims.width || prev.height !== dims.height ? dims : prev
        );
      }
    }, 500);
    return () => clearInterval(interval);
  }, []);

  return (
    <Box
      sx={{
        height: "100vh",
        width: "100vw",
        display: "flex",
        flexDirection: "column",
        backgroundColor: "#0a0e1a",
        overflow: "hidden",
      }}
    >
      <StatusBar
        connected={data.connected}
        alertLevel={data.alertLevel}
        activeView={activeView}
        onViewChange={setActiveView}
        phase={data.phase}
      />

      <Box sx={{ flex: 1, display: "flex", flexDirection: "column", minHeight: 0, position: "relative" }}>
        {/* Global calibration reset button — visible during calibration on
            Live and Spatial tabs (both have adjustable split + cam2 refs) */}
        {calibrationMode && (activeView === "live" || activeView === "spatial") && (
          <Box
            onClick={handleResetCalibration}
            sx={{
              position: "absolute",
              top: 8,
              left: "50%",
              transform: "translateX(-50%)",
              zIndex: 30,
              px: 2,
              py: 0.7,
              backgroundColor: "#0d1117ee",
              border: "1px solid #00b4d880",
              borderRadius: "4px",
              cursor: "pointer",
              boxShadow: "0 0 10px #00b4d840",
              transition: "all 0.2s ease",
              "&:hover": {
                backgroundColor: "#00b4d820",
                borderColor: "#00b4d8",
                boxShadow: "0 0 14px #00b4d870",
              },
            }}
          >
            <Typography
              sx={{
                fontFamily: "'JetBrains Mono', monospace",
                fontSize: "12px",
                color: "#00b4d8",
                fontWeight: 700,
                letterSpacing: "2px",
              }}
            >
              ↺ RESET CALIBRATION
            </Typography>
          </Box>
        )}
        {activeView === "live" && (
          <LiveGuidanceView
            cam1Ref={cam1Ref}
            cam2Ref={cam2Ref}
            cam1Dims={cam1Dims}
            data={data}
            calibrationMode={calibrationMode}
            splitRatio={liveSplitRatio}
            setSplitRatio={setLiveSplitRatio}
            resetSignal={resetSignal}
          />
        )}
        {activeView === "spatial" && (
          <SpatialSplitView
            cam1Ref={cam1Ref}
            cam1Dims={cam1Dims}
            data={data}
            calibrationMode={calibrationMode}
            splitRatio={spatialSplitRatio}
            setSplitRatio={setSpatialSplitRatio}
          />
        )}
        {activeView === "diagnostics" && (
          <DiagnosticsView
            connected={data.connected}
            distance={data.distance}
            zone={data.zone}
            proximityStatus={data.proximityStatus}
            detections={data.detections}
            gimbalYaw={data.gimbalYaw}
            gimbalPitch={data.gimbalPitch}
            ladderDistance={data.ladderDistance}
            ladderStatus={data.ladderStatus}
            approachHeading={data.approachHeading}
            rawHeading={rawData.approachHeading}
            headingOffset={headingOffset}
            approachZone={data.approachZone}
            approachProfile={data.approachProfile}
            boatPitch={data.boatPitch}
            boatRoll={data.boatRoll}
            lastUpdate={data.lastUpdate}
            currentModel={data.currentModel}
            defaultModel={data.defaultModel}
            onSwitchModel={(model) => rawData.send({ type: "switch_model", model })}
            onSetDefaultModel={(model) => rawData.send({ type: "set_default_model", model })}
            onZeroHeading={handleZeroHeading}
            onResetHeading={handleResetHeading}
            calibrationMode={calibrationMode}
            onToggleCalibration={() => setCalibrationMode((v) => !v)}
          />
        )}
      </Box>

      <AlertBanner alertLevel={data.alertLevel} />

      <TestModeButton
        testMode={testMode}
        testIndex={testIndex}
        testHeadingIndex={testHeadingIndex}
        onToggle={() => {
          setTestMode((prev) => !prev);
          setTestIndex(0);
          setTestHeadingIndex(0);
        }}
        onNextScenario={() => setTestIndex((prev) => (prev + 1) % TEST_SCENARIOS.length)}
        onNextHeading={() => setTestHeadingIndex((prev) => (prev + 1) % HEADING_TEST_VALUES.length)}
      />
    </Box>
  );
};

export default PilotDashboard;
