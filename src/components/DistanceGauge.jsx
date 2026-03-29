import React from "react";
import { Box, Typography } from "@mui/material";
import { DISTANCE_ZONES, MAX_DISPLAY_DISTANCE } from "../config";

/**
 * Large distance readout with color-coded gradient bar.
 * Overlaid on the bottom of the left camera panel.
 *
 * Props:
 *   distance: number (meters)
 */
const DistanceGauge = ({ distance = null }) => {
  // Determine zone color
  let zoneColor = DISTANCE_ZONES.safe.color;
  let zoneLabel = DISTANCE_ZONES.safe.label;
  if (distance !== null) {
    if (distance <= DISTANCE_ZONES.danger.max) {
      zoneColor = DISTANCE_ZONES.danger.color;
      zoneLabel = DISTANCE_ZONES.danger.label;
    } else if (distance <= DISTANCE_ZONES.caution.max) {
      zoneColor = DISTANCE_ZONES.caution.color;
      zoneLabel = DISTANCE_ZONES.caution.label;
    }
  }

  // Marker position (0% = danger/left, 100% = safe/right)
  const markerPercent =
    distance !== null
      ? Math.min(100, Math.max(0, (distance / MAX_DISPLAY_DISTANCE) * 100))
      : 50;

  return (
    <Box
      sx={{
        position: "absolute",
        bottom: 0,
        left: 0,
        right: 0,
        zIndex: 10,
        padding: "12px 20px 16px",
        background: "linear-gradient(transparent, rgba(0,0,0,0.85))",
      }}
    >
      {/* Distance readout */}
      <Box
        sx={{
          display: "flex",
          alignItems: "baseline",
          justifyContent: "center",
          gap: 1,
          mb: 1,
        }}
      >
        <Typography
          sx={{
            fontFamily: "'JetBrains Mono', monospace",
            fontSize: "64px",
            fontWeight: 700,
            color: distance !== null ? zoneColor : "#555",
            lineHeight: 1,
            textShadow: `0 0 20px ${zoneColor}40`,
          }}
        >
          {distance !== null ? distance.toFixed(1) : "--.-"}
        </Typography>
        <Typography
          sx={{
            fontFamily: "'JetBrains Mono', monospace",
            fontSize: "28px",
            fontWeight: 600,
            color: distance !== null ? zoneColor : "#555",
            opacity: 0.8,
          }}
        >
          m
        </Typography>
      </Box>

      {/* Zone label */}
      <Typography
        sx={{
          textAlign: "center",
          fontSize: "14px",
          fontWeight: 700,
          letterSpacing: "3px",
          color: distance !== null ? zoneColor : "#555",
          mb: 1,
          opacity: 0.9,
        }}
      >
        {distance !== null ? zoneLabel : "NO DATA"}
      </Typography>

      {/* Gradient bar */}
      <Box
        sx={{
          position: "relative",
          height: "12px",
          borderRadius: "6px",
          background:
            "linear-gradient(to right, #ff2222 0%, #ff2222 10%, #ffaa00 30%, #ffaa00 40%, #00ff88 70%, #00ff88 100%)",
          overflow: "visible",
        }}
      >
        {/* Marker */}
        <Box
          sx={{
            position: "absolute",
            left: `${markerPercent}%`,
            top: "-4px",
            transform: "translateX(-50%)",
            width: 0,
            height: 0,
            borderLeft: "8px solid transparent",
            borderRight: "8px solid transparent",
            borderTop: "10px solid #fff",
            filter: "drop-shadow(0 1px 3px rgba(0,0,0,0.5))",
            transition: "left 0.15s ease-out",
          }}
        />
      </Box>
    </Box>
  );
};

export default React.memo(DistanceGauge);
