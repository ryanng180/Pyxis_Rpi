import React, { useRef, useEffect } from "react";
import { Box, Typography } from "@mui/material";
import WarningAmberIcon from "@mui/icons-material/WarningAmber";
import ErrorIcon from "@mui/icons-material/Error";
import { ALERT_AUDIO_ENABLED } from "../config";

/**
 * Full-width safety alert banner.
 * Slides up from bottom on caution/danger.
 * Plays audio tone on zone transitions.
 *
 * Props:
 *   alertLevel: "none" | "caution" | "danger"
 *   message: string
 */
const AlertBanner = ({ alertLevel = "none", message = "" }) => {
  const prevLevelRef = useRef("none");
  const audioCtxRef = useRef(null);

  // Play alert tone on zone transition
  useEffect(() => {
    if (prevLevelRef.current === alertLevel) return;
    prevLevelRef.current = alertLevel;

    if (!ALERT_AUDIO_ENABLED || alertLevel === "none") return;

    try {
      if (!audioCtxRef.current) {
        audioCtxRef.current = new (window.AudioContext || window.webkitAudioContext)();
      }
      const ctx = audioCtxRef.current;
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();

      osc.connect(gain);
      gain.connect(ctx.destination);

      if (alertLevel === "danger") {
        osc.frequency.value = 880;
        gain.gain.value = 0.3;
        osc.start();
        osc.stop(ctx.currentTime + 0.3);
        // Double beep for danger
        setTimeout(() => {
          const osc2 = ctx.createOscillator();
          const gain2 = ctx.createGain();
          osc2.connect(gain2);
          gain2.connect(ctx.destination);
          osc2.frequency.value = 880;
          gain2.gain.value = 0.3;
          osc2.start();
          osc2.stop(ctx.currentTime + 0.3);
        }, 400);
      } else {
        osc.frequency.value = 660;
        gain.gain.value = 0.2;
        osc.start();
        osc.stop(ctx.currentTime + 0.2);
      }
    } catch {
      // Audio API may not be available
    }
  }, [alertLevel]);

  if (alertLevel === "none") return null;

  const isDanger = alertLevel === "danger";

  return (
    <Box
      sx={{
        height: "60px",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        gap: 2,
        backgroundColor: isDanger ? "#ff2222" : "#ffaa00",
        color: isDanger ? "#fff" : "#000",
        animation: isDanger
          ? "dangerFlash 0.8s ease-in-out infinite"
          : "cautionPulse 2s ease-in-out infinite",
        "@keyframes dangerFlash": {
          "0%, 100%": { opacity: 1 },
          "50%": { opacity: 0.7 },
        },
        "@keyframes cautionPulse": {
          "0%, 100%": { opacity: 1 },
          "50%": { opacity: 0.85 },
        },
      }}
    >
      {isDanger ? (
        <ErrorIcon sx={{ fontSize: 28 }} />
      ) : (
        <WarningAmberIcon sx={{ fontSize: 28 }} />
      )}
      <Typography
        sx={{
          fontSize: "18px",
          fontWeight: 700,
          letterSpacing: "2px",
          textTransform: "uppercase",
        }}
      >
        {message || (isDanger ? "DANGER - TOO CLOSE" : "CAUTION - APPROACHING LIMIT")}
      </Typography>
      {isDanger ? (
        <ErrorIcon sx={{ fontSize: 28 }} />
      ) : (
        <WarningAmberIcon sx={{ fontSize: 28 }} />
      )}
    </Box>
  );
};

export default React.memo(AlertBanner);
