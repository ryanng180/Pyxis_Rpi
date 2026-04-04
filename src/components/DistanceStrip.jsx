import React from "react";
import { Box, Typography } from "@mui/material";
import { MAX_DISPLAY_DISTANCE, PYXIS_ACCENT } from "../config";

/**
 * Full-width distance strip — the most glanceable element on the dashboard.
 *
 * Layout:
 *   [LADDER DIST (left, near cam1)] — [ZONE BAR (center)] — [HULL DIST (right, near cam2)]
 *
 * Zone bar has 4 visual segments with HARD boundaries (no misleading bleed):
 *   0 – 0.3m    RED          collision risk        (20% of bar)
 *   0.3 – 1.0m  GREEN        safe boarding range   (40% of bar)
 *   1.0 – 2.5m  AMBER        closing, needs focus  (25% of bar)
 *   2.5 – 5.0m  GREY         still approaching     (15% of bar)
 */

const ZONE_CLOSE = 0.3;
const ZONE_FAR = 1.0;
const ZONE_CAUTION = 2.5;

const ZONE_CONFIG = {
  TOO_CLOSE: { color: "#ff2222", label: "TOO CLOSE",    bg: "#ff222220" },
  OPTIMAL:   { color: "#00ff88", label: "OPTIMAL",       bg: "#00ff8818" },
  TOO_FAR:   { color: "#dd8800", label: "CLOSING",       bg: "#dd880015" },
  APPROACHING:{ color: "#667788", label: "APPROACHING",  bg: "#66778810" },
};

// Map distance to display zone config
const getZoneConfig = (zone, distance) => {
  if (!zone && distance === null) return { color: "#555", label: "NO DATA", bg: "transparent" };
  if (zone === "TOO_CLOSE") return ZONE_CONFIG.TOO_CLOSE;
  if (zone === "OPTIMAL") return ZONE_CONFIG.OPTIMAL;
  // TOO_FAR from ROS2 — split into CLOSING vs APPROACHING based on distance
  if (distance !== null && distance > ZONE_CAUTION) return ZONE_CONFIG.APPROACHING;
  if (zone === "TOO_FAR") return ZONE_CONFIG.TOO_FAR;
  return { color: "#555", label: "NO DATA", bg: "transparent" };
};

// Non-linear mapping: distance -> bar percentage
// Gives the critical close range more visual space
const distToPercent = (d) => {
  if (d <= ZONE_CLOSE) return (d / ZONE_CLOSE) * 20;                                          // 0-20%
  if (d <= ZONE_FAR) return 20 + ((d - ZONE_CLOSE) / (ZONE_FAR - ZONE_CLOSE)) * 40;          // 20-60%
  if (d <= ZONE_CAUTION) return 60 + ((d - ZONE_FAR) / (ZONE_CAUTION - ZONE_FAR)) * 25;      // 60-85%
  return 85 + Math.min(((d - ZONE_CAUTION) / (MAX_DISPLAY_DISTANCE - ZONE_CAUTION)) * 15, 15); // 85-100%
};

const DistanceReadout = ({ label, value, unit, color, align }) => (
  <Box sx={{ minWidth: "220px", textAlign: align, px: 2.5 }}>
    <Typography
      sx={{
        fontSize: "13px",
        color: "#667",
        fontWeight: 600,
        letterSpacing: "2px",
        mb: 0.3,
      }}
    >
      {label}
    </Typography>
    <Box
      sx={{
        display: "flex",
        alignItems: "baseline",
        justifyContent: align === "left" ? "flex-start" : "flex-end",
        gap: 0.5,
      }}
    >
      <Typography
        sx={{
          fontFamily: "'JetBrains Mono', monospace",
          fontSize: "52px",
          fontWeight: 700,
          color,
          lineHeight: 1,
          textShadow: value !== "--.-" ? `0 0 15px ${color}40` : "none",
        }}
      >
        {value}
      </Typography>
      <Typography
        sx={{
          fontFamily: "'JetBrains Mono', monospace",
          fontSize: "24px",
          fontWeight: 600,
          color,
          opacity: 0.7,
        }}
      >
        {unit}
      </Typography>
    </Box>
  </Box>
);

const DistanceStrip = ({ distance = null, zone = null, ladderDistance = null }) => {
  const zoneConf = getZoneConfig(zone, distance);
  const hullColor = zoneConf.color;
  const ladderColor = ladderDistance !== null ? PYXIS_ACCENT : "#555";

  const markerPct = distance !== null
    ? Math.min(100, Math.max(0, distToPercent(distance)))
    : -10;

  return (
    <Box
      sx={{
        height: "92px",
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        backgroundColor: "#0d1117",
        borderBottom: `2px solid ${hullColor}30`,
      }}
    >
      {/* LEFT: Ladder distance (near cam1 — ladder view) */}
      <DistanceReadout
        label="LADDER DISTANCE"
        value={ladderDistance !== null ? ladderDistance.toFixed(1) : "--.-"}
        unit="m"
        color={ladderColor}
        align="left"
      />

      {/* CENTER: Zone badge + segmented gradient bar with slider */}
      <Box
        sx={{
          flex: 1,
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          gap: 0.6,
          mx: 2,
        }}
      >
        {/* Zone label badge */}
        <Box
          sx={{
            px: 3,
            py: 0.4,
            borderRadius: "4px",
            backgroundColor: zoneConf.bg,
            border: `2px solid ${hullColor}80`,
          }}
        >
          <Typography
            sx={{
              fontSize: "17px",
              fontWeight: 700,
              letterSpacing: "4px",
              color: hullColor,
              textAlign: "center",
            }}
          >
            {zoneConf.label}
          </Typography>
        </Box>

        {/* Zone bar — 4 hard segments, no misleading colour bleed */}
        <Box
          sx={{
            position: "relative",
            width: "100%",
            height: "16px",
            borderRadius: "8px",
            overflow: "visible",
            display: "flex",
          }}
        >
          {/* Segment 1: DANGER — solid red (0-20%) */}
          <Box sx={{
            width: "20%",
            height: "100%",
            background: "linear-gradient(to right, #cc1111, #ff2222)",
            borderRadius: "8px 0 0 8px",
          }} />
          {/* Segment 2: OPTIMAL — solid green (20-60%) */}
          <Box sx={{
            width: "40%",
            height: "100%",
            background: "linear-gradient(to right, #00dd77, #00ff88, #00dd77)",
          }} />
          {/* Segment 3: CLOSING — amber fading out (60-85%) */}
          <Box sx={{
            width: "25%",
            height: "100%",
            background: "linear-gradient(to right, #dd8800, #aa6600, #887755)",
          }} />
          {/* Segment 4: APPROACHING — muted grey (85-100%) */}
          <Box sx={{
            width: "15%",
            height: "100%",
            background: "linear-gradient(to right, #667788, #445566)",
            borderRadius: "0 8px 8px 0",
          }} />

          {/* Zone divider lines */}
          {[20, 60, 85].map((pct) => (
            <Box
              key={pct}
              sx={{
                position: "absolute",
                left: `${pct}%`,
                top: 0,
                bottom: 0,
                width: "2px",
                backgroundColor: "#0a0e1a",
                zIndex: 1,
              }}
            />
          ))}

          {/* Distance slider marker */}
          {distance !== null && (
            <Box
              sx={{
                position: "absolute",
                left: `${markerPct}%`,
                top: "-7px",
                transform: "translateX(-50%)",
                width: "6px",
                height: "30px",
                backgroundColor: "#fff",
                borderRadius: "3px",
                border: "1px solid rgba(0,0,0,0.3)",
                boxShadow:
                  "0 0 10px rgba(255,255,255,0.7), " +
                  "0 0 4px rgba(255,255,255,0.9), " +
                  "0 2px 4px rgba(0,0,0,0.5)",
                transition: "left 0.15s ease-out",
                zIndex: 3,
              }}
            />
          )}
        </Box>

        {/* Zone boundary labels */}
        <Box sx={{ display: "flex", width: "100%", position: "relative", height: "14px" }}>
          <Typography sx={{
            position: "absolute", left: "20%", transform: "translateX(-50%)",
            fontSize: "11px", color: "#66aa8888", fontWeight: 600,
          }}>
            0.3m
          </Typography>
          <Typography sx={{
            position: "absolute", left: "60%", transform: "translateX(-50%)",
            fontSize: "11px", color: "#dd880088", fontWeight: 600,
          }}>
            1.0m
          </Typography>
          <Typography sx={{
            position: "absolute", left: "85%", transform: "translateX(-50%)",
            fontSize: "11px", color: "#66778888", fontWeight: 600,
          }}>
            2.5m
          </Typography>
        </Box>
      </Box>

      {/* RIGHT: Hull distance (near cam2 — position view) */}
      <DistanceReadout
        label="HULL DISTANCE"
        value={distance !== null ? distance.toFixed(2) : "--.-"}
        unit="m"
        color={hullColor}
        align="right"
      />
    </Box>
  );
};

export default React.memo(DistanceStrip);
