import React, { useRef, useState, useEffect } from "react";
import { Box } from "@mui/material";
import StatusBar from "../global/Topbar";
import CameraFeed from "../../components/CameraFeed";
import CVOverlay from "../../components/CVOverlay";
import DistanceGauge from "../../components/DistanceGauge";
import AlertBanner from "../../components/AlertBanner";
import useMaritimeData from "../../hooks/useMaritimeData";

/**
 * Main Pilot Transfer Dashboard - single-screen HMI.
 *
 * Layout (CSS Grid):
 * +--------------------------------------------------+
 * | StatusBar (40px)                                  |
 * +------------------------+-------------------------+
 * | Camera 1 (Ladder View) | Camera 2 (Position View)|
 * | + CV Overlay           | + Proximity Arcs        |
 * | + Distance Gauge       | + Boat Icon             |
 * +------------------------+-------------------------+
 * | AlertBanner (60px, conditional)                   |
 * +--------------------------------------------------+
 */
const PilotDashboard = () => {
  const cam1Ref = useRef(null);
  const cam2Ref = useRef(null);
  const [cam1Dims, setCam1Dims] = useState({ width: 0, height: 0 });

  const {
    connected,
    distance,
    detections,
    sourceResolution,
    cameraUrls,
    alertLevel,
  } = useMaritimeData();

  // Track camera panel dimensions for overlays
  useEffect(() => {
    const interval = setInterval(() => {
      if (cam1Ref.current?.getContainer) {
        const dims = cam1Ref.current.getDimensions();
        setCam1Dims((prev) =>
          prev.width !== dims.width || prev.height !== dims.height
            ? dims
            : prev
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
        display: "grid",
        gridTemplateRows: "40px 1fr auto",
        backgroundColor: "#0a0e1a",
        overflow: "hidden",
      }}
    >
      {/* Status Bar */}
      <StatusBar connected={connected} alertLevel={alertLevel} />

      {/* Main Content: Two Camera Panels */}
      <Box
        sx={{
          display: "grid",
          gridTemplateColumns: "1fr 1fr",
          gap: "2px",
          minHeight: 0,
        }}
      >
        {/* Left Panel: Camera 1 - Ladder View */}
        <Box sx={{ position: "relative", overflow: "hidden" }}>
          <CameraFeed
            ref={cam1Ref}
            source={{ type: "mjpeg", url: cameraUrls.cam1 }}
            label="Camera 1 — Ladder View"
          />
          <CVOverlay
            detections={detections}
            width={cam1Dims.width}
            height={cam1Dims.height}
            sourceResolution={sourceResolution}
          />
          <DistanceGauge distance={distance} />
        </Box>

        {/* Right Panel: Camera 2 - Position View */}
        <Box sx={{ position: "relative", overflow: "hidden" }}>
          <CameraFeed
            ref={cam2Ref}
            source={{ type: "mjpeg", url: cameraUrls.cam2 }}
            label="Camera 2 — Position View"
          />
        </Box>
      </Box>

      {/* Alert Banner */}
      <AlertBanner alertLevel={alertLevel} />
    </Box>
  );
};

export default PilotDashboard;
