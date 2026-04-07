import React, { useRef, useEffect } from "react";
import { Box, Typography } from "@mui/material";
import ErrorIcon from "@mui/icons-material/Error";
import { ALERT_AUDIO_ENABLED } from "../config";

/**
 * Full-width DANGER alert banner.
 * Fixed to bottom of screen — overlays content, never warps layout.
 * Plays a repeating double-beep while the danger condition persists, so the
 * captain keeps getting nudged even if they look away mid-alarm. Interval is
 * cleared the moment the zone drops out of danger.
 */
const DANGER_BEEP_INTERVAL_MS = 2000; // matches the banner flash cadence

const AlertBanner = ({ alertLevel = "none", message = "" }) => {
  const audioCtxRef = useRef(null);
  const intervalRef = useRef(null);

  // Pre-warm AudioContext on first user interaction (browsers require gesture)
  useEffect(() => {
    if (!ALERT_AUDIO_ENABLED) return;

    const warmUp = () => {
      if (!audioCtxRef.current) {
        audioCtxRef.current = new (window.AudioContext || window.webkitAudioContext)();
      }
      if (audioCtxRef.current.state === "suspended") {
        audioCtxRef.current.resume();
      }
      // Only need one interaction
      window.removeEventListener("click", warmUp);
      window.removeEventListener("touchstart", warmUp);
    };

    window.addEventListener("click", warmUp);
    window.addEventListener("touchstart", warmUp);
    return () => {
      window.removeEventListener("click", warmUp);
      window.removeEventListener("touchstart", warmUp);
    };
  }, []);

  useEffect(() => {
    if (!ALERT_AUDIO_ENABLED) return;

    // Leaving danger — stop the repeating alarm immediately
    if (alertLevel !== "danger") {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
      return;
    }

    // Already alarming — don't stack a second interval
    if (intervalRef.current) return;

    const playBeep = () => {
      try {
        if (!audioCtxRef.current) {
          audioCtxRef.current = new (window.AudioContext || window.webkitAudioContext)();
        }
        const ctx = audioCtxRef.current;

        const fire = () => {
          // Re-check state at fire time — Chrome can suspend idle contexts
          // after ~30s, in which case oscillators start silently. Guard here
          // instead of trusting the ref's last-known state.
          if (ctx.state === "suspended") {
            ctx.resume().catch(() => {});
          }
          const osc = ctx.createOscillator();
          const gain = ctx.createGain();
          osc.connect(gain);
          gain.connect(ctx.destination);
          osc.frequency.value = 880;
          gain.gain.value = 0.3;
          osc.start();
          osc.stop(ctx.currentTime + 0.3);

          // Double beep
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
        };

        if (ctx.state === "suspended") {
          ctx.resume().then(fire).catch(() => {});
        } else {
          fire();
        }
      } catch {
        // Audio API may not be available
      }
    };

    // Fire immediately on entering danger, then keep repeating
    playBeep();
    intervalRef.current = setInterval(playBeep, DANGER_BEEP_INTERVAL_MS);

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [alertLevel]);

  if (alertLevel !== "danger") return null;

  return (
    <Box
      sx={{
        position: "fixed",
        bottom: 0,
        left: 0,
        right: 0,
        height: "60px",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        gap: 2,
        backgroundColor: "#ff2222",
        color: "#fff",
        zIndex: 9999,
        animation: "dangerFlash 0.8s ease-in-out infinite",
        "@keyframes dangerFlash": {
          "0%, 100%": { opacity: 1 },
          "50%": { opacity: 0.7 },
        },
      }}
    >
      <ErrorIcon sx={{ fontSize: 28 }} />
      <Typography
        sx={{
          fontSize: "18px",
          fontWeight: 700,
          letterSpacing: "2px",
          textTransform: "uppercase",
        }}
      >
        {message || "DANGER — TOO CLOSE TO VESSEL"}
      </Typography>
      <ErrorIcon sx={{ fontSize: 28 }} />
    </Box>
  );
};

export default React.memo(AlertBanner);
