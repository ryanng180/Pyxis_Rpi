import React, { useState, useEffect, useRef } from "react";
import { Box, Typography } from "@mui/material";
import { PYXIS_ACCENT } from "../config";

/**
 * Remote control panel — rendered at /#control on a second smaller screen.
 * Sends commands to the main dashboard via BroadcastChannel.
 * Also receives state sync back so buttons reflect the current dashboard state.
 */

const CHANNEL_NAME = "pyxis-remote";

const VIEWS = [
  { id: "live", label: "LIVE" },
  { id: "spatial", label: "SPATIAL" },
  { id: "diagnostics", label: "DIAG" },
];

const BigButton = ({ label, active, color, onClick, sx }) => (
  <Box
    onClick={onClick}
    sx={{
      flex: 1,
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      minHeight: "80px",
      borderRadius: "12px",
      cursor: "pointer",
      userSelect: "none",
      WebkitTapHighlightColor: "transparent",
      border: `2px solid ${active ? color : "#333"}`,
      backgroundColor: active ? `${color}20` : "#0d1117",
      transition: "all 0.15s ease",
      "&:hover": { backgroundColor: active ? `${color}30` : "#1a2035" },
      "&:active": { transform: "scale(0.97)" },
      ...sx,
    }}
  >
    <Typography
      sx={{
        fontSize: "clamp(18px, 4vw, 28px)",
        fontWeight: 800,
        letterSpacing: "3px",
        color: active ? color : "#667",
      }}
    >
      {label}
    </Typography>
  </Box>
);

const ControlPanel = () => {
  const channelRef = useRef(null);
  const [activeView, setActiveView] = useState("live");
  const [testMode, setTestMode] = useState(false);
  const [calibrationMode, setCalibrationMode] = useState(false);
  const [connected, setConnected] = useState(false);
  const [phase, setPhase] = useState(null);

  useEffect(() => {
    const ch = new BroadcastChannel(CHANNEL_NAME);
    channelRef.current = ch;

    // Listen for state sync from the main dashboard
    ch.onmessage = (e) => {
      const msg = e.data;
      if (msg.type === "state_sync") {
        if (msg.activeView !== undefined) setActiveView(msg.activeView);
        if (msg.testMode !== undefined) setTestMode(msg.testMode);
        if (msg.calibrationMode !== undefined) setCalibrationMode(msg.calibrationMode);
        if (msg.connected !== undefined) setConnected(msg.connected);
        if (msg.phase !== undefined) setPhase(msg.phase);
      }
    };

    // Request initial state from dashboard
    ch.postMessage({ type: "request_sync" });

    return () => ch.close();
  }, []);

  const send = (msg) => {
    channelRef.current?.postMessage(msg);
  };

  const PHASE_COLOR = {
    APPROACHING: "#667788",
    ZONING: "#dd8800",
    HOLDING: "#00ff88",
  };

  return (
    <Box
      sx={{
        height: "100vh",
        width: "100vw",
        display: "flex",
        flexDirection: "column",
        backgroundColor: "#0a0e1a",
        overflow: "hidden",
        p: 2,
        gap: 2,
      }}
    >
      {/* Header */}
      <Box sx={{ display: "flex", alignItems: "center", justifyContent: "space-between" }}>
        <Box sx={{ display: "flex", alignItems: "center", gap: 1.5 }}>
          <Typography
            sx={{
              fontSize: "20px",
              fontWeight: 700,
              color: PYXIS_ACCENT,
              letterSpacing: "4px",
            }}
          >
            PYXIS
          </Typography>
          <Typography
            sx={{
              fontSize: "14px",
              fontWeight: 600,
              color: "#556",
              letterSpacing: "2px",
            }}
          >
            REMOTE
          </Typography>
        </Box>
        <Box sx={{ display: "flex", alignItems: "center", gap: 1 }}>
          {phase && PHASE_COLOR[phase] && (
            <Typography
              sx={{
                fontSize: "12px",
                fontWeight: 700,
                letterSpacing: "2px",
                color: PHASE_COLOR[phase],
              }}
            >
              {phase}
            </Typography>
          )}
          <Box
            sx={{
              width: 10,
              height: 10,
              borderRadius: "50%",
              backgroundColor: connected ? "#00ff88" : "#ff2222",
              animation: connected ? "none" : "blink 1.5s infinite",
              "@keyframes blink": {
                "0%, 100%": { opacity: 1 },
                "50%": { opacity: 0.3 },
              },
            }}
          />
        </Box>
      </Box>

      {/* View switching — primary controls */}
      <Box sx={{ display: "flex", gap: 1.5 }}>
        {VIEWS.map((v) => (
          <BigButton
            key={v.id}
            label={v.label}
            active={activeView === v.id}
            color={PYXIS_ACCENT}
            onClick={() => send({ type: "view_change", view: v.id })}
          />
        ))}
      </Box>

      {/* Test mode controls */}
      <Box sx={{ display: "flex", flexDirection: "column", gap: 1.5 }}>
        <BigButton
          label={testMode ? "TEST ON" : "TEST MODE"}
          active={testMode}
          color="#ffaa00"
          onClick={() => send({ type: "test_toggle" })}
          sx={{ minHeight: "70px" }}
        />
        {testMode && (
          <Box sx={{ display: "flex", gap: 1.5 }}>
            <BigButton
              label="NEXT SCENARIO"
              active={false}
              color="#ffaa00"
              onClick={() => send({ type: "test_next_scenario" })}
              sx={{ minHeight: "70px" }}
            />
            <BigButton
              label="NEXT HEADING"
              active={false}
              color="#ffaa00"
              onClick={() => send({ type: "test_next_heading" })}
              sx={{ minHeight: "70px" }}
            />
          </Box>
        )}
      </Box>

      {/* Calibration */}
      <BigButton
        label={calibrationMode ? "CALIBRATION ON" : "CALIBRATION"}
        active={calibrationMode}
        color={PYXIS_ACCENT}
        onClick={() => send({ type: "calibration_toggle" })}
        sx={{ minHeight: "70px" }}
      />
      {calibrationMode && (
        <BigButton
          label="RESET CALIBRATION"
          active={false}
          color="#ff6644"
          onClick={() => send({ type: "calibration_reset" })}
          sx={{ minHeight: "60px" }}
        />
      )}

      {/* Heading controls */}
      <Box sx={{ display: "flex", gap: 1.5 }}>
        <BigButton
          label="ZERO HEADING"
          active={false}
          color="#00ff88"
          onClick={() => send({ type: "zero_heading" })}
          sx={{ minHeight: "70px" }}
        />
        <BigButton
          label="RESET HEADING"
          active={false}
          color="#ff6644"
          onClick={() => send({ type: "reset_heading" })}
          sx={{ minHeight: "70px" }}
        />
      </Box>

      {/* Spacer */}
      <Box sx={{ flex: 1 }} />
    </Box>
  );
};

export default ControlPanel;
