import React from "react";
import { Box, Typography } from "@mui/material";

/**
 * Phase-dependent overlay on the camera feed area.
 *
 *   APPROACHING — large distance HUD on cam1 (countdown feel)
 *   ZONING      — nothing (the full dashboard IS the zoning experience)
 *   HOLDING     — green "SAFE TO BOARD" confirmation bar
 *
 * All elements use CSS transitions for smooth morphing between phases.
 */

const PHASE_COLORS = {
  APPROACHING: "#667788",
  ZONING: "#dd8800",
  HOLDING: "#00ff88",
};

// ── APPROACHING: large glanceable distance HUD ──
const ApproachingHUD = ({ distance }) => (
  <Box
    sx={{
      position: "absolute",
      bottom: 0,
      left: 0,
      right: 0,
      zIndex: 7,
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      gap: 2,
      py: 1.5,
      px: 3,
      backgroundColor: "#0d111790",
      backdropFilter: "blur(4px)",
      borderTop: "1px solid #66778830",
      pointerEvents: "none",
    }}
  >
    <Typography
      sx={{
        fontSize: "14px",
        fontWeight: 700,
        letterSpacing: "3px",
        color: "#667788",
      }}
    >
      APPROACHING
    </Typography>
    <Typography
      sx={{
        fontFamily: "'JetBrains Mono', monospace",
        fontSize: "36px",
        fontWeight: 700,
        color: "#889aab",
        lineHeight: 1,
        textShadow: "0 0 12px #66778840",
      }}
    >
      {distance !== null ? `${distance.toFixed(1)}m` : "--.-m"}
    </Typography>
  </Box>
);

// ── HOLDING: safe-to-board confirmation ──
const HoldingBanner = () => (
  <Box
    sx={{
      position: "absolute",
      top: 0,
      left: 0,
      right: 0,
      zIndex: 7,
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      py: 1,
      backgroundColor: "#00ff8812",
      borderBottom: "1px solid #00ff8840",
      pointerEvents: "none",
    }}
  >
    <Typography
      sx={{
        fontSize: "15px",
        fontWeight: 700,
        letterSpacing: "4px",
        color: "#00ff88",
        textShadow: "0 0 8px #00ff8830",
      }}
    >
      HOLDING — SAFE TO BOARD
    </Typography>
  </Box>
);

const PhaseOverlay = ({ phase, distance, position = "cam1" }) => {
  // ZONING: no overlay — the full UI is the experience
  if (!phase || phase === "ZONING") return null;

  // APPROACHING: only show on cam1 (large distance HUD)
  if (phase === "APPROACHING" && position === "cam1") {
    return <ApproachingHUD distance={distance} />;
  }

  // HOLDING: show confirmation banner on both cameras
  if (phase === "HOLDING") {
    return <HoldingBanner />;
  }

  return null;
};

export default React.memo(PhaseOverlay);
