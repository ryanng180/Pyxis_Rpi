import React, { useState, useEffect } from "react";
import { Box, Typography } from "@mui/material";
import FiberManualRecordIcon from "@mui/icons-material/FiberManualRecord";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import { PYXIS_ACCENT } from "../../config";

/**
 * Slim status bar at the top of the dashboard.
 * Shows: connection status | Pyxis branding | view tabs | alert + clock
 *
 * Props:
 *   connected: boolean
 *   alertLevel: "none" | "caution" | "danger"
 *   activeView: string
 *   onViewChange: (view) => void
 */
const VIEWS = [
  { id: "live", label: "LIVE" },
  { id: "spatial", label: "SPATIAL" },
  { id: "diagnostics", label: "DIAG" },
];

const PHASE_STYLE = {
  APPROACHING: { color: "#667788", label: "APPROACHING" },
  ZONING:      { color: "#dd8800", label: "ZONING" },
  HOLDING:     { color: "#00ff88", label: "HOLDING" },
};

const StatusBar = ({
  connected = false,
  alertLevel = "none",
  activeView = "live",
  onViewChange,
  phase = null,
}) => {
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const interval = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(interval);
  }, []);

  const timeStr = time.toLocaleTimeString("en-GB", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });

  return (
    <Box
      sx={{
        height: "52px",
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        px: 2,
        backgroundColor: "#0d1117",
        borderBottom: "1px solid #1a2035",
      }}
    >
      {/* Left: Pyxis brand + connection */}
      <Box sx={{ display: "flex", alignItems: "center", gap: 1.5, minWidth: "200px" }}>
        <Typography
          sx={{
            fontSize: "22px",
            fontWeight: 700,
            color: PYXIS_ACCENT,
            letterSpacing: "4px",
          }}
        >
          PYXIS
        </Typography>
        <Box sx={{ display: "flex", alignItems: "center", gap: 0.5 }}>
          <FiberManualRecordIcon
            sx={{
              fontSize: 12,
              color: connected ? "#00ff88" : "#ff2222",
              animation: connected ? "none" : "blink 1.5s infinite",
              "@keyframes blink": {
                "0%, 100%": { opacity: 1 },
                "50%": { opacity: 0.3 },
              },
            }}
          />
          <Typography sx={{ fontSize: "15px", color: connected ? "#00ff8899" : "#ff222299", fontWeight: 600 }}>
            {connected ? "ONLINE" : "OFFLINE"}
          </Typography>
        </Box>
        {/* Phase badge */}
        {phase && PHASE_STYLE[phase] && (
          <Box
            sx={{
              px: 1.5,
              py: 0.3,
              borderRadius: "4px",
              border: `1px solid ${PHASE_STYLE[phase].color}66`,
              backgroundColor: `${PHASE_STYLE[phase].color}15`,
            }}
          >
            <Typography
              sx={{
                fontSize: "12px",
                fontWeight: 700,
                letterSpacing: "2px",
                color: PHASE_STYLE[phase].color,
              }}
            >
              {PHASE_STYLE[phase].label}
            </Typography>
          </Box>
        )}
      </Box>

      {/* Center: View tabs */}
      <Box sx={{ display: "flex", gap: 0 }}>
        {VIEWS.map((view) => (
          <Box
            key={view.id}
            onClick={() => onViewChange?.(view.id)}
            sx={{
              px: 2,
              py: 0.5,
              cursor: "pointer",
              borderBottom: activeView === view.id
                ? `2px solid ${PYXIS_ACCENT}`
                : "2px solid transparent",
              "&:hover": {
                backgroundColor: "#1a203520",
              },
            }}
          >
            <Typography
              sx={{
                fontSize: "16px",
                fontWeight: 700,
                color: activeView === view.id ? PYXIS_ACCENT : "#667",
                letterSpacing: "4px",
              }}
            >
              {view.label}
            </Typography>
          </Box>
        ))}
      </Box>

      {/* Right: alert indicator + clock */}
      <Box sx={{ display: "flex", alignItems: "center", gap: 1.5, minWidth: "200px", justifyContent: "flex-end" }}>
        {alertLevel !== "none" && (
          <WarningAmberIcon
            sx={{
              fontSize: 20,
              color: alertLevel === "danger" ? "#ff2222" : "#ffaa00",
              animation: "pulse 1s infinite",
              "@keyframes pulse": {
                "0%, 100%": { transform: "scale(1)" },
                "50%": { transform: "scale(1.2)" },
              },
            }}
          />
        )}
        <Typography
          sx={{
            fontFamily: "'JetBrains Mono', monospace",
            fontSize: "18px",
            color: "#888",
            fontWeight: 600,
          }}
        >
          {timeStr}
        </Typography>
      </Box>
    </Box>
  );
};

export default React.memo(StatusBar);
