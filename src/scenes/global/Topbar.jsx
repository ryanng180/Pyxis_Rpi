import React, { useState, useEffect } from "react";
import { Box, Typography } from "@mui/material";
import FiberManualRecordIcon from "@mui/icons-material/FiberManualRecord";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";

/**
 * Slim status bar at the top of the dashboard.
 * Shows connection status, clock, and alert indicator.
 *
 * Props:
 *   connected: boolean
 *   alertLevel: "none" | "caution" | "danger"
 */
const StatusBar = ({ connected = false, alertLevel = "none" }) => {
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
        height: "40px",
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        px: 2,
        backgroundColor: "#0d1117",
        borderBottom: "1px solid #1a2035",
      }}
    >
      {/* Left: connection status */}
      <Box sx={{ display: "flex", alignItems: "center", gap: 1 }}>
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
        <Typography sx={{ fontSize: "12px", color: "#888", fontWeight: 600 }}>
          {connected ? "CONNECTED" : "DISCONNECTED"}
        </Typography>
      </Box>

      {/* Center: title */}
      <Typography
        sx={{
          fontSize: "13px",
          fontWeight: 700,
          color: "#667",
          letterSpacing: "3px",
          textTransform: "uppercase",
        }}
      >
        Pilot Transfer Assist
      </Typography>

      {/* Right: alert indicator + clock */}
      <Box sx={{ display: "flex", alignItems: "center", gap: 2 }}>
        {alertLevel !== "none" && (
          <WarningAmberIcon
            sx={{
              fontSize: 18,
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
            fontSize: "14px",
            color: "#aaa",
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
