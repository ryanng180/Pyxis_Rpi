import React from "react";
import { Box, Typography } from "@mui/material";
import { PYXIS_ACCENT } from "../config";

/**
 * Right-side guidance panel overlaid on/beside Camera 2.
 * Shows key decision-support indicators:
 *   - Ladder detected / tracking / lost
 *   - Heading alignment
 *   - Gimbal angle
 *   - Boat tilt
 */

const StatusCard = ({ label, value, unit, color, small }) => (
  <Box
    sx={{
      px: 1.5,
      py: small ? 0.8 : 1,
      backgroundColor: "#0d111780",
      borderLeft: `3px solid ${color || "#333"}`,
      backdropFilter: "blur(4px)",
    }}
  >
    <Typography
      sx={{
        fontSize: "13px",
        fontWeight: 600,
        color: "#889",
        letterSpacing: "1.5px",
        textTransform: "uppercase",
        mb: 0.3,
      }}
    >
      {label}
    </Typography>
    <Box sx={{ display: "flex", alignItems: "baseline", gap: 0.5 }}>
      <Typography
        sx={{
          fontFamily: "'JetBrains Mono', monospace",
          fontSize: small ? "20px" : "24px",
          fontWeight: 700,
          color: color || "#e0e0e0",
          lineHeight: 1,
        }}
      >
        {value}
      </Typography>
      {unit && (
        <Typography
          sx={{
            fontFamily: "'JetBrains Mono', monospace",
            fontSize: "14px",
            color: color || "#e0e0e0",
            opacity: 0.6,
          }}
        >
          {unit}
        </Typography>
      )}
    </Box>
  </Box>
);

const GuidancePanel = ({
  phase = null,
  detections = [],
  approachHeading = null,
  approachZone = null,
  gimbalYaw = null,
  gimbalPitch = null,
  boatPitch = null,
  boatRoll = null,
}) => {
  const isApproaching = phase === "APPROACHING";
  // Ladder detection status
  const ladderDetected = detections.some(
    (d) => d.label?.toLowerCase() === "ladder"
  );
  const ladderConfidence = detections.find(
    (d) => d.label?.toLowerCase() === "ladder"
  )?.confidence;

  const ladderLabel = ladderDetected
    ? `TRACKING ${(ladderConfidence * 100).toFixed(0)}%`
    : "SEARCHING";
  const ladderColor = ladderDetected ? "#00ff88" : "#ffaa00";

  // Heading alignment
  const headingLabel =
    approachHeading !== null ? `${approachHeading > 0 ? "+" : ""}${approachHeading.toFixed(1)}` : "--";
  const headingColor =
    approachHeading !== null
      ? Math.abs(approachHeading) < 3
        ? "#00ff88"
        : Math.abs(approachHeading) < 8
        ? "#ffaa00"
        : "#ff2222"
      : "#555";

  // Approach zone
  const approachColor =
    approachZone === "TOO_CLOSE" ? "#ff2222"
    : approachZone === "OPTIMAL" ? "#00ff88"
    : approachZone === "TOO_FAR" ? "#ffaa00"
    : "#555";

  return (
    <Box
      sx={{
        position: "absolute",
        top: 32,
        right: 0,
        zIndex: 8,
        width: "200px",
        display: "flex",
        flexDirection: "column",
        gap: "1px",
        pointerEvents: "none",
      }}
    >
      <StatusCard
        label="Ladder"
        value={ladderLabel}
        color={ladderColor}
      />
      <StatusCard
        label="Heading"
        value={headingLabel}
        unit="°"
        color={headingColor}
        small
      />
      <StatusCard
        label="Approach"
        value={approachZone || "---"}
        color={approachColor}
        small
      />
      {/* Gimbal + Tilt fade out during APPROACHING (not relevant yet) */}
      <Box sx={{ opacity: isApproaching ? 0.2 : 1, transition: "opacity 0.8s ease" }}>
        {gimbalYaw !== null && (
          <StatusCard
            label="Gimbal"
            value={`${gimbalYaw > 0 ? "+" : ""}${gimbalYaw.toFixed(0)}°`}
            unit={gimbalPitch !== null ? `/ ${gimbalPitch.toFixed(0)}°` : ""}
            color={PYXIS_ACCENT}
            small
          />
        )}
        {(boatRoll !== null || boatPitch !== null) && (
          <StatusCard
            label="Tilt"
            value={`R${boatRoll !== null ? boatRoll.toFixed(1) : "-"}°`}
            unit={`P${boatPitch !== null ? boatPitch.toFixed(1) : "-"}°`}
            color="#888"
            small
          />
        )}
      </Box>
    </Box>
  );
};

export default React.memo(GuidancePanel);
