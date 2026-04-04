import React, { useRef, useState, useEffect, useCallback } from "react";
import { Box, Typography } from "@mui/material";
import StatusBar from "../global/Topbar";
import CameraFeed from "../../components/CameraFeed";
import CVOverlay from "../../components/CVOverlay";
import DistanceStrip from "../../components/DistanceStrip";
import GuidancePanel from "../../components/GuidancePanel";
import ProximityOverlay from "../../components/ProximityOverlay";
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

const LiveGuidanceView = ({
  cam1Ref,
  cam2Ref,
  cam1Dims,
  cam2Dims,
  data,
}) => {
  const phase = data.phase;
  const accent = PHASE_ACCENT[phase] || "transparent";
  const isApproaching = phase === "APPROACHING";

  return (
    <Box sx={{ display: "flex", flexDirection: "column", flex: 1, minHeight: 0 }}>
      <DistanceStrip
        distance={data.distance}
        zone={data.zone}
        ladderDistance={data.ladderDistance}
      />
      <Box
        sx={{
          flex: 1,
          display: "grid",
          gridTemplateColumns: "3fr 2fr",
          gap: "2px",
          minHeight: 0,
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
          <ProximityOverlay
            distance={data.distance}
            width={cam2Dims.width}
            height={cam2Dims.height}
          />
          <GuidancePanel
            phase={phase}
            detections={data.detections}
            approachHeading={data.approachHeading}
            approachZone={data.approachZone}
            gimbalYaw={data.gimbalYaw}
            gimbalPitch={data.gimbalPitch}
            boatPitch={data.boatPitch}
            boatRoll={data.boatRoll}
          />
          <PhaseOverlay phase={phase} distance={data.distance} position="cam2" />
        </Box>
      </Box>
    </Box>
  );
};

// ── Test Mode: simulates distance/phase for indoor testing ──
const TEST_SCENARIOS = [
  { label: "6.0m APPROACHING", distance: 6.0, zone: "TOO_FAR",   phase: "APPROACHING" },
  { label: "2.0m ZONING",      distance: 2.0, zone: "TOO_FAR",   phase: "ZONING" },
  { label: "1.5m ZONING",      distance: 1.5, zone: "TOO_FAR",   phase: "ZONING" },
  { label: "1.0m ZONING",      distance: 1.0, zone: "OPTIMAL",   phase: "ZONING" },
  { label: "0.8m HOLDING",     distance: 0.8, zone: "OPTIMAL",   phase: "HOLDING" },
  { label: "0.5m HOLDING",     distance: 0.5, zone: "OPTIMAL",   phase: "HOLDING" },
  { label: "0.2m DANGER",      distance: 0.2, zone: "TOO_CLOSE", phase: "ZONING" },
];

const TestModeButton = ({ testMode, testIndex, onToggle, onNext }) => (
  <Box
    sx={{
      position: "fixed",
      bottom: 12,
      right: 12,
      zIndex: 10000,
      display: "flex",
      gap: 1,
      alignItems: "center",
    }}
  >
    {testMode && (
      <>
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
            {TEST_SCENARIOS[testIndex].label}
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
      </>
    )}
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
  </Box>
);

const PilotDashboard = () => {
  const cam1Ref = useRef(null);
  const cam2Ref = useRef(null);
  const [cam1Dims, setCam1Dims] = useState({ width: 0, height: 0 });
  const [cam2Dims, setCam2Dims] = useState({ width: 0, height: 0 });
  const [activeView, setActiveView] = useState("live");
  const [headingOffset, setHeadingOffset] = useState(0);
  const [testMode, setTestMode] = useState(false);
  const [testIndex, setTestIndex] = useState(0);

  const rawData = useMaritimeData();

  // Apply heading offset — all views see the corrected heading
  const liveData = {
    ...rawData,
    approachHeading: rawData.approachHeading !== null
      ? rawData.approachHeading - headingOffset
      : null,
  };

  // Test mode overrides distance, zone, and phase
  const data = testMode
    ? {
        ...liveData,
        distance: TEST_SCENARIOS[testIndex].distance,
        zone: TEST_SCENARIOS[testIndex].zone,
        phase: TEST_SCENARIOS[testIndex].phase,
        alertLevel: TEST_SCENARIOS[testIndex].zone === "TOO_CLOSE" ? "danger" : "none",
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
      if (cam2Ref.current?.getDimensions) {
        const dims = cam2Ref.current.getDimensions();
        setCam2Dims((prev) =>
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

      <Box sx={{ flex: 1, display: "flex", flexDirection: "column", minHeight: 0 }}>
        {activeView === "live" && (
          <LiveGuidanceView
            cam1Ref={cam1Ref}
            cam2Ref={cam2Ref}
            cam1Dims={cam1Dims}
            cam2Dims={cam2Dims}
            data={data}
          />
        )}
        {activeView === "spatial" && (
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
            onSwitchModel={(model) => rawData.send({ type: "switch_model", model })}
            onZeroHeading={handleZeroHeading}
            onResetHeading={handleResetHeading}
          />
        )}
      </Box>

      <AlertBanner alertLevel={data.alertLevel} />

      <TestModeButton
        testMode={testMode}
        testIndex={testIndex}
        onToggle={() => { setTestMode((prev) => !prev); setTestIndex(0); }}
        onNext={() => setTestIndex((prev) => (prev + 1) % TEST_SCENARIOS.length)}
      />
    </Box>
  );
};

export default PilotDashboard;
