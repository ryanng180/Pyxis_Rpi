import React, { useRef, useEffect } from "react";
import { DISTANCE_ZONES } from "../config";

/**
 * Canvas overlay for Camera 2 showing parking-sensor-style proximity arcs.
 * Draws concentric colored arcs at the top (bow) and bottom (stern).
 *
 * Props:
 *   distance: number (current distance in meters)
 *   width: number (canvas width)
 *   height: number (canvas height)
 */
const ProximityOverlay = ({ distance = 3.0, width, height }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !width || !height) return;

    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, width, height);

    const zones = [
      { max: DISTANCE_ZONES.closing.max, color: DISTANCE_ZONES.closing.color },
      { max: DISTANCE_ZONES.optimal.max, color: DISTANCE_ZONES.optimal.color },
      { max: DISTANCE_ZONES.danger.max, color: DISTANCE_ZONES.danger.color },
    ];

    // Determine active zone (outer=closing, middle=optimal, inner=danger)
    let activeZoneIdx = 0; // closing
    if (distance <= DISTANCE_ZONES.danger.max) activeZoneIdx = 2;
    else if (distance <= DISTANCE_ZONES.optimal.max) activeZoneIdx = 1;

    const drawArcs = (cx, cy, startAngle, endAngle, flip) => {
      const arcCount = zones.length;
      const maxRadius = Math.min(width, height) * 0.22;
      const minRadius = maxRadius * 0.4;
      const radiusStep = (maxRadius - minRadius) / arcCount;
      const gap = 4;

      for (let i = 0; i < arcCount; i++) {
        // Outer arcs = safe (green), inner arcs = danger (red)
        const zoneIdx = flip ? arcCount - 1 - i : i;
        const radius = maxRadius - i * radiusStep;
        const innerRadius = radius - radiusStep + gap;

        const isActive = zoneIdx >= activeZoneIdx;

        ctx.beginPath();
        ctx.arc(cx, cy, radius, startAngle, endAngle);
        ctx.arc(cx, cy, innerRadius, endAngle, startAngle, true);
        ctx.closePath();

        ctx.fillStyle = zones[zoneIdx].color;
        ctx.globalAlpha = isActive ? 0.7 : 0.15;
        ctx.fill();
      }
      ctx.globalAlpha = 1.0;
    };

    // Bow arcs (top of frame)
    const bowCx = width / 2;
    const bowCy = height * 0.08;
    drawArcs(bowCx, bowCy, Math.PI * 0.15, Math.PI * 0.85, false);

    // Stern arcs (bottom of frame)
    const sternCx = width / 2;
    const sternCy = height * 0.92;
    drawArcs(sternCx, sternCy, -Math.PI * 0.85, -Math.PI * 0.15, false);
  }, [distance, width, height]);

  if (!width || !height) return null;

  return (
    <canvas
      ref={canvasRef}
      style={{
        position: "absolute",
        top: 0,
        left: 0,
        width: "100%",
        height: "100%",
        pointerEvents: "none",
        zIndex: 5,
      }}
    />
  );
};

export default React.memo(ProximityOverlay);
