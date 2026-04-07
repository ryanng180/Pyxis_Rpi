import React, { useState, useEffect, useRef } from "react";
import { Box, Typography } from "@mui/material";
import { DISTANCE_ZONES } from "../config";

/**
 * Cam2 overlays — replaces old parking-sensor semicircles.
 *
 *   Cam2StatusFrame    — full-frame inner border that reacts to zone + phase.
 *                        Subtle pulse only when TOO_CLOSE (alert banner handles
 *                        the loud danger cue). Muted during APPROACHING phase.
 *
 *   Cam2ReferenceLine  — vertical reference line marking where the cargo hull
 *                        sits in the frame. Fixed camera, but the hull position
 *                        in view differs between lab rig and ship trials, so
 *                        the line is draggable in calibration mode and its
 *                        position persists to localStorage.
 */

// ── Cam2StatusFrame ─────────────────────────────────────────────────────────

const frameColorFor = (zone, phase) => {
  // No telemetry yet — neutral grey (don't imply a zone we don't have)
  if (!phase && !zone) return "#44556630";
  if (phase === "APPROACHING") return "#66778840";
  if (phase === "HOLDING") return DISTANCE_ZONES.optimal.color + "cc";
  // ZONING — color by distance zone
  if (zone === "TOO_CLOSE") return DISTANCE_ZONES.danger.color + "dd";
  if (zone === "OPTIMAL") return DISTANCE_ZONES.optimal.color + "aa";
  if (zone === "TOO_FAR") return DISTANCE_ZONES.closing.color + "aa";
  return "#44556630";
};

// Solid status color (no alpha) — used by the reference line so it reads
// boldly on top of the camera image.
const statusColorFor = (zone, phase) => {
  if (phase === "APPROACHING") return "#889aab";
  if (phase === "HOLDING") return DISTANCE_ZONES.optimal.color;
  if (zone === "TOO_CLOSE") return DISTANCE_ZONES.danger.color;
  if (zone === "OPTIMAL") return DISTANCE_ZONES.optimal.color;
  if (zone === "TOO_FAR") return DISTANCE_ZONES.closing.color;
  return "#889aab";
};

export const Cam2StatusFrame = ({ zone, phase }) => {
  const color = frameColorFor(zone, phase);
  const pulsing = zone === "TOO_CLOSE" && phase !== "APPROACHING";

  return (
    <>
      <style>
        {`@keyframes cam2FramePulse {
            0%, 100% { opacity: 0.55; }
            50%      { opacity: 1;    }
          }`}
      </style>
      <Box
        sx={{
          position: "absolute",
          inset: 0,
          pointerEvents: "none",
          border: `3px solid ${color}`,
          boxShadow: `inset 0 0 24px ${color}`,
          transition: "border-color 0.8s ease, box-shadow 0.8s ease",
          animation: pulsing ? "cam2FramePulse 1.6s ease-in-out infinite" : "none",
          zIndex: 4,
        }}
      />
    </>
  );
};

// ── Cam2ReferenceLine ───────────────────────────────────────────────────────
// Two freely-positionable anchor points — each handle can be placed anywhere
// in the frame. Line is drawn between them. Lets the operator anchor against
// any reference on the hull (top-edge to waterline, corner to corner, etc.).

const POINTS_KEY = "pyxis_cam2_ref_points";
const LEGACY_TOP = "pyxis_cam2_ref_top_x";
const LEGACY_BOT = "pyxis_cam2_ref_bot_x";
const LEGACY_X   = "pyxis_cam2_ref_x";

const clamp = (v) => Math.max(0.02, Math.min(0.98, v));

const DEFAULT_POINTS = [
  { x: 0.5, y: 0.1 },
  { x: 0.5, y: 0.9 },
];

const loadPoints = () => {
  // Preferred: JSON array in POINTS_KEY
  try {
    const raw = localStorage.getItem(POINTS_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      if (
        Array.isArray(parsed) && parsed.length === 2 &&
        parsed.every((p) => typeof p?.x === "number" && typeof p?.y === "number")
      ) {
        return parsed.map((p) => ({ x: clamp(p.x), y: clamp(p.y) }));
      }
    }
  } catch (e) {}
  // Migrate from older two-x schema
  try {
    const topX = parseFloat(localStorage.getItem(LEGACY_TOP));
    const botX = parseFloat(localStorage.getItem(LEGACY_BOT));
    if (!isNaN(topX) && !isNaN(botX)) {
      return [{ x: clamp(topX), y: 0.1 }, { x: clamp(botX), y: 0.9 }];
    }
  } catch (e) {}
  // Migrate from original single-x schema
  try {
    const x = parseFloat(localStorage.getItem(LEGACY_X));
    if (!isNaN(x)) {
      return [{ x: clamp(x), y: 0.1 }, { x: clamp(x), y: 0.9 }];
    }
  } catch (e) {}
  return DEFAULT_POINTS.map((p) => ({ ...p }));
};

export const Cam2ReferenceLine = ({
  calibrationMode = false,
  zone,
  phase,
  resetSignal = 0,
}) => {
  const [points, setPoints] = useState(loadPoints);
  const [draggingIdx, setDraggingIdx] = useState(null); // 0 | 1 | null
  const containerRef = useRef(null);

  // Persist after drag finishes
  useEffect(() => {
    if (draggingIdx !== null) return;
    try {
      localStorage.setItem(POINTS_KEY, JSON.stringify(points));
    } catch (e) {}
  }, [points, draggingIdx]);

  // Reset-to-defaults trigger from parent
  const firstResetSkip = useRef(true);
  useEffect(() => {
    if (firstResetSkip.current) {
      firstResetSkip.current = false;
      return;
    }
    setPoints(DEFAULT_POINTS.map((p) => ({ ...p })));
  }, [resetSignal]);

  useEffect(() => {
    if (draggingIdx === null) return;
    const onMove = (e) => {
      const clientX = e.clientX ?? e.touches?.[0]?.clientX;
      const clientY = e.clientY ?? e.touches?.[0]?.clientY;
      if (clientX === undefined || clientY === undefined) return;
      const el = containerRef.current;
      if (!el) return;
      const rect = el.getBoundingClientRect();
      const rx = clamp((clientX - rect.left) / rect.width);
      const ry = clamp((clientY - rect.top) / rect.height);
      setPoints((prev) =>
        prev.map((p, i) => (i === draggingIdx ? { x: rx, y: ry } : p))
      );
    };
    const onUp = () => setDraggingIdx(null);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    window.addEventListener("touchmove", onMove);
    window.addEventListener("touchend", onUp);
    return () => {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      window.removeEventListener("touchmove", onMove);
      window.removeEventListener("touchend", onUp);
    };
  }, [draggingIdx]);

  // Line color follows zone/phase status — red/green/amber/grey.
  // Thicker and bolder than before for better glance-readability.
  const lineColor = statusColorFor(zone, phase);
  const lineOpacity = calibrationMode ? 1.0 : 0.85;
  const lineWidth = calibrationMode ? 6 : 5;

  return (
    <Box
      ref={containerRef}
      sx={{
        position: "absolute",
        inset: 0,
        pointerEvents: "none",
        zIndex: 6,
      }}
    >
      {/* SVG line between the two free-floating anchor points */}
      <svg
        style={{
          position: "absolute",
          top: 0,
          left: 0,
          width: "100%",
          height: "100%",
          pointerEvents: "none",
          overflow: "visible",
        }}
        preserveAspectRatio="none"
      >
        <line
          x1={`${points[0].x * 100}%`}
          y1={`${points[0].y * 100}%`}
          x2={`${points[1].x * 100}%`}
          y2={`${points[1].y * 100}%`}
          stroke={lineColor}
          strokeWidth={lineWidth}
          opacity={lineOpacity}
          strokeLinecap="round"
          style={{
            transition: "stroke 0.5s ease, opacity 0.5s ease, stroke-width 0.3s ease",
            filter: `drop-shadow(0 0 8px ${lineColor}aa)`,
          }}
        />
      </svg>

      {/* Drag handles — only in calibration mode */}
      {calibrationMode &&
        points.map((p, i) => (
          <RefHandle
            key={i}
            point={p}
            label={i === 0 ? "A" : "B"}
            onStart={() => setDraggingIdx(i)}
          />
        ))}

      {/* Calibration hint */}
      {calibrationMode && (
        <Box
          sx={{
            position: "absolute",
            bottom: 12,
            left: "50%",
            transform: "translateX(-50%)",
            pointerEvents: "none",
            backgroundColor: "#0d1117ee",
            border: "1px solid #00b4d860",
            borderRadius: "4px",
            px: 2,
            py: 0.5,
          }}
        >
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "12px",
              color: "#00b4d8",
              letterSpacing: "1px",
            }}
          >
            CALIBRATION — DRAG A &amp; B TO ANY POINTS ALONG THE HULL EDGE
          </Typography>
        </Box>
      )}
    </Box>
  );
};

// Free-floating drag handle — positions itself anywhere in the frame
const RefHandle = ({ point, label, onStart }) => (
  <Box
    onMouseDown={(e) => { e.preventDefault(); onStart(); }}
    onTouchStart={(e) => { e.preventDefault(); onStart(); }}
    sx={{
      position: "absolute",
      left: `${point.x * 100}%`,
      top: `${point.y * 100}%`,
      transform: "translate(-50%, -50%)",
      width: "28px",
      height: "28px",
      borderRadius: "50%",
      backgroundColor: "#00b4d8",
      border: "2px solid #ffffff",
      boxShadow: "0 0 12px #00b4d8",
      cursor: "move",
      display: "flex",
      alignItems: "center",
      justifyContent: "center",
      pointerEvents: "auto",
      touchAction: "none",
    }}
  >
    <Typography sx={{ fontSize: "12px", color: "#0a0e1a", fontWeight: 900, lineHeight: 1 }}>
      {label}
    </Typography>
  </Box>
);
