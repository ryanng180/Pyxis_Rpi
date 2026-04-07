import React, { useRef, useEffect, useState } from "react";
import { Box, Typography } from "@mui/material";
import { PYXIS_ACCENT } from "../config";

/**
 * Spatial / Bird's-eye guidance view.
 *
 * Renders the pilot boat's operational picture in a pilot-boat-frame
 * world view:
 *   - Cargo hull drawn as a contour whose vertical offset from the pilot
 *     boat reflects the measured per-segment gap (closer segments sit
 *     closer to the boat, farther segments pull away). Line is colour-
 *     coded by zone. This is the faithful spatial reading of
 *     /approach/profile.hull_profile.
 *   - Pilot boat outline rotated by the measured heading error around
 *     its mount point. Heading IS the boat's skew — no separate arrow.
 *   - Ladder sightline from the gimbal mount using gimbalYaw as bearing
 *     and ladderDistance as length, terminating in a marker whose lock
 *     ring is green when gimbal reports target_locked, amber while
 *     searching.
 *
 * All numeric readouts are owned by the top strip — the canvas is
 * purely spatial.
 */

const ZONE_COLORS = {
  TOO_CLOSE: "#ff2222",
  OPTIMAL: "#00ff88",
  TOO_FAR: "#dd8800",
  NO_DATA: "#333",
};

// Pilot boat geometry (metres) — from cargo_approach_node parameters
const HULL_LEN_M = 13.053; // hull_bow_x - hull_rear_x
const MOUNT_FRAC_FROM_BOW = 0.17; // gimbal mount ~17% aft of bow along hull
const MAX_GAP_M = 2.5; // max gap rendered on the vertical axis

// The boat is drawn with NON-UNIFORM scale: true metric along the hull axis
// (so gap readings line up with real-world positions), but a fixed pixel beam
// perpendicular so the boat reads as a schematic, not a leviathan. Using true
// scale on both axes makes a 13m × 2.5m boat look ~19% as tall as wide, which
// dominates the canvas on any realistic aspect ratio.
const BOAT_BEAM_PX = 48;

// Gap thresholds — mirrored from cargo_approach_node defaults
const TOO_CLOSE_M = 0.3;
const TOO_FAR_M = 1.0;

// Default boat geometry used when ws_server hasn't forwarded it yet
// (matches cargo_approach_node params in sensors_launch.py)
const DEFAULT_BOAT_GEOMETRY = {
  bow_x: 0.987,
  rear_x: -12.066,
  starboard_y: -1.123,
  lidar_x: -0.01161,
  lidar_y: -0.0457,
};

const SpatialView = ({
  distance = null,
  zone = null,
  ladderDistance = null,
  approachHeading = null,
  approachZone = null,
  approachProfile = null,
  gimbalYaw = null,
  boatPitch = null,
  boatRoll = null,
  gimbalTargetLocked = false,
  phase = null,
  cargoPoints = null,
  boatGeometry = null,
}) => {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  // Persistent view bounds for the Spatial ZONING/HOLDING plot. Smoothed
  // across frames so the axes don't jitter every time a single point
  // enters or leaves the filter. Grows fast (never clip data), shrinks
  // slow (no disorienting zoom-out flicker).
  const viewRef = useRef({ hullMin: null, hullMax: null, gapMax: null });

  // Track container size for DPR-correct redraws on resize
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        setSize({ w: Math.floor(width), h: Math.floor(height) });
      }
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !size.w || !size.h) return;

    // ── HiDPI-aware backing store ────────────────────────────────────
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size.w * dpr;
    canvas.height = size.h * dpr;
    canvas.style.width = `${size.w}px`;
    canvas.style.height = `${size.h}px`;
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = size.w;
    const h = size.h;

    // ── Background ───────────────────────────────────────────────────
    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(0, 0, w, h);

    // ── ZONING / HOLDING layout: gap ribbon from cargo-side points ───
    // This is the "real" spatial view used when close enough that the
    // captain cares about the gap along each part of the boat's hull.
    // It renders raw LiDAR points (forwarded from /scan/filtered) plus
    // a derived gap curve at 0.25m resolution. Y-axis auto-zooms; X-axis
    // is locked to the true boat length so the captain's mental ruler
    // never changes.
    const isGapView = phase === "ZONING" || phase === "HOLDING";
    if (isGapView) {
      const geom = boatGeometry || DEFAULT_BOAT_GEOMETRY;
      // Server forwards points in the same coordinates cargo_approach_node
      // uses: hull_x = projected position along the boat's hull line
      // (metres, bow positive), gap = perpendicular distance from the hull
      // line (metres, always positive). The server has already applied the
      // bow/stern + own-boat-edge filters; we render every point verbatim.
      const hullXsIn = cargoPoints?.hull_x || [];
      const gapsIn = cargoPoints?.gap || [];

      // ── Layout: TOP-DOWN ISOTROPIC view ─────────────────────────────
      //   canvas Y axis  = boat length   (bow at top, stern at bottom)
      //   canvas X axis  = athwartships  (port left, starboard centre,
      //                                   open water/cargo to the right)
      // 1 metre vertical = 1 metre horizontal. The boat is drawn at its
      // real beam (2 × |starboard_y|) so the captain can directly compare
      // "my beam" to "gap to cargo" — a critical spatial ratio when
      // deciding whether to close in or back off.
      const marginL2 = 20;
      const marginR2 = 60;   // room for hull-axis tick labels on the right
      const marginTop2 = 30;
      const marginBottom2 = 44;
      const plotLeft = marginL2;
      const plotRight = w - marginR2;
      const plotTop = marginTop2;
      const plotBottom = h - marginBottom2;
      const plotW = plotRight - plotLeft;
      const plotH = plotBottom - plotTop;

      const boatLenM = geom.bow_x - geom.rear_x;
      // Half-beam from centreline — |starboard_y| is the distance from
      // centreline to our own starboard edge. Assume symmetric hull.
      const halfBeamM = Math.abs(geom.starboard_y);
      const beamM = 2 * halfBeamM;

      // ── Raw target window from current frame ─────────────────────
      const HULL_PAD_M = 0.8;
      const HULL_MIN_WINDOW_M = 3.0;
      let tHullMax = geom.bow_x;
      let tHullMin = geom.rear_x;
      if (hullXsIn.length > 0) {
        tHullMin = hullXsIn[0];
        tHullMax = hullXsIn[0];
        for (let i = 1; i < hullXsIn.length; i++) {
          const hx = hullXsIn[i];
          if (hx < tHullMin) tHullMin = hx;
          if (hx > tHullMax) tHullMax = hx;
        }
        tHullMin -= HULL_PAD_M;
        tHullMax += HULL_PAD_M;
        const span = tHullMax - tHullMin;
        if (span < HULL_MIN_WINDOW_M) {
          const mid = (tHullMin + tHullMax) / 2;
          tHullMin = mid - HULL_MIN_WINDOW_M / 2;
          tHullMax = mid + HULL_MIN_WINDOW_M / 2;
        }
      }

      const GAP_MIN_WINDOW_M = 1.5;
      const GAP_PAD_M = 0.4;
      let tGapMax = GAP_MIN_WINDOW_M;
      for (let i = 0; i < gapsIn.length; i++) {
        if (gapsIn[i] + GAP_PAD_M > tGapMax) {
          tGapMax = gapsIn[i] + GAP_PAD_M;
        }
      }
      tGapMax = Math.min(Math.max(tGapMax, GAP_MIN_WINDOW_M), 4.0);

      // ── Asymmetric low-pass smoothing ─────────────────────────────
      // ATTACK (expand) is fast: 0.6 per frame at 5 Hz ≈ settle in ~4
      // frames, so a new point that extends the bounds is visible almost
      // immediately and never clipped.
      // DECAY (contract) is slow: 0.04 per frame ≈ ~5 s to fully shrink.
      // Slow enough that a stray dropout doesn't yank the axes, but
      // responsive enough to follow a real departure.
      const ATTACK = 0.6;
      const DECAY = 0.04;
      const v = viewRef.current;
      // hullMax: expand when target > current (attack), shrink when target < current (decay)
      const nextHullMax = v.hullMax === null
        ? tHullMax
        : (tHullMax > v.hullMax
            ? v.hullMax + (tHullMax - v.hullMax) * ATTACK
            : v.hullMax + (tHullMax - v.hullMax) * DECAY);
      // hullMin: expand when target < current (attack), shrink when target > current (decay)
      const nextHullMin = v.hullMin === null
        ? tHullMin
        : (tHullMin < v.hullMin
            ? v.hullMin + (tHullMin - v.hullMin) * ATTACK
            : v.hullMin + (tHullMin - v.hullMin) * DECAY);
      // gapMax: expand = attack, shrink = decay
      const nextGapMax = v.gapMax === null
        ? tGapMax
        : (tGapMax > v.gapMax
            ? v.gapMax + (tGapMax - v.gapMax) * ATTACK
            : v.gapMax + (tGapMax - v.gapMax) * DECAY);

      v.hullMax = nextHullMax;
      v.hullMin = nextHullMin;
      v.gapMax = nextGapMax;

      // Quantize the window to 0.25 m steps so tiny sub-decimetre
      // wobbles don't cause pixel-level rescaling at all. The smoother
      // handles slow drift; quantization kills residual twitch.
      const Q = 0.25;
      const hullMax = Math.ceil(nextHullMax / Q) * Q;
      const hullMin = Math.floor(nextHullMin / Q) * Q;
      const maxGapSeen = Math.ceil(nextGapMax / Q) * Q;
      const hullWindowM = hullMax - hullMin;

      // ── ISOTROPIC pixels-per-metre ──────────────────────────────────
      // Horizontal content = port-half of boat + starboard-half + gap
      //                    = beamM + maxGapSeen  (plus a little left pad)
      // Vertical content   = hullWindowM
      // Use the limiting dimension so both fit on screen at the same
      // metric scale. 1 m on X == 1 m on Y after this.
      const leftPadM = 0.2; // tiny breathing room left of port edge
      const contentWM = leftPadM + beamM + maxGapSeen;
      const contentHM = hullWindowM;
      const pxPerM = Math.min(plotW / contentWM, plotH / contentHM);

      // Place the content horizontally, centred within the plot area when
      // vertical is the limiting dimension (otherwise usedW == plotW and
      // the extra shift is 0).
      const usedW = contentWM * pxPerM;
      const hOffset = (plotW - usedW) / 2;
      const portEdgeX = plotLeft + hOffset + leftPadM * pxPerM;
      const centreX = portEdgeX + halfBeamM * pxPerM;
      const starboardEdgeX = centreX + halfBeamM * pxPerM;

      // Vertical centring within plot area
      const usedH = contentHM * pxPerM;
      const contentTop = plotTop + (plotH - usedH) / 2;
      const boatXToPy = (bx) =>
        contentTop + (hullMax - bx) * pxPerM;

      const gapToPxX = (g) =>
        starboardEdgeX + Math.min(Math.max(g, 0), maxGapSeen) * pxPerM;

      const gapColor = (g) => {
        if (g < TOO_CLOSE_M) return "#ff2222";
        if (g > TOO_FAR_M) return "#ffaa00";
        return "#00ff88";
      };

      // ── 1 m × 1 m isotropic grid (square cells — since pxPerM is the
      //    same on both axes, the grid is visually square and the
      //    captain can instantly "count squares" in any direction) ───
      const gridStep = 1.0;
      const portEdgeM = -halfBeamM;                 // gap = -halfBeam at port
      const gridGapMin = Math.floor(portEdgeM / gridStep) * gridStep;
      const gridGapMax = Math.ceil(maxGapSeen / gridStep) * gridStep;
      ctx.strokeStyle = "#141a2a";
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 4]);
      // Vertical grid lines (constant gap)
      for (let g = gridGapMin; g <= gridGapMax + 1e-6; g += gridStep) {
        const x = centreX + g * pxPerM;
        if (x < plotLeft || x > plotRight) continue;
        ctx.beginPath();
        ctx.moveTo(x, plotTop);
        ctx.lineTo(x, plotBottom);
        ctx.stroke();
      }
      // Horizontal grid lines (constant hull_x)
      const gridHullMin = Math.floor(hullMin / gridStep) * gridStep;
      const gridHullMax = Math.ceil(hullMax / gridStep) * gridStep;
      for (let hx = gridHullMin; hx <= gridHullMax + 1e-6; hx += gridStep) {
        const y = boatXToPy(hx);
        if (y < plotTop || y > plotBottom) continue;
        ctx.beginPath();
        ctx.moveTo(plotLeft, y);
        ctx.lineTo(plotRight, y);
        ctx.stroke();
      }
      ctx.setLineDash([]);

      // Gap-axis tick labels along the bottom (distance from starboard)
      ctx.fillStyle = "#445566";
      ctx.font = "11px 'JetBrains Mono', monospace";
      ctx.textAlign = "center";
      for (let g = 0; g <= maxGapSeen + 1e-6; g += gridStep) {
        const x = gapToPxX(g);
        ctx.fillText(`${g.toFixed(0)}m`, x, plotBottom + 14);
      }

      // ── Zone bands: coloured vertical strips across the gap axis ───
      // Filled at low alpha so points stay clearly visible on top.
      //   [0 .. TOO_CLOSE_M)       → red    (TOO CLOSE)
      //   [TOO_CLOSE_M .. TOO_FAR_M) → green  (OPTIMAL)
      //   [TOO_FAR_M .. maxGap]    → amber  (TOO FAR)
      const zoneBandTop = plotTop;
      const zoneBandBot = plotBottom;
      const zones = [
        { from: 0,           to: TOO_CLOSE_M, fill: "#ff2222", label: "TOO CLOSE" },
        { from: TOO_CLOSE_M, to: TOO_FAR_M,   fill: "#00ff88", label: "OPTIMAL"   },
        { from: TOO_FAR_M,   to: maxGapSeen,  fill: "#ffaa00", label: "TOO FAR"   },
      ];
      zones.forEach(({ from, to, fill, label }) => {
        const clampedTo = Math.min(to, maxGapSeen);
        if (clampedTo <= from) return;
        const xFrom = gapToPxX(from);
        const xTo = gapToPxX(clampedTo);
        // Band fill — low alpha over plot area
        ctx.fillStyle = fill + "1c"; // ~11% alpha
        ctx.fillRect(xFrom, zoneBandTop, xTo - xFrom, zoneBandBot - zoneBandTop);
        // Label centred in the band, just below the top edge
        const midX = (xFrom + xTo) / 2;
        if (xTo - xFrom > 36) {
          ctx.fillStyle = fill + "dd";
          ctx.font = "bold 11px 'JetBrains Mono', monospace";
          ctx.textAlign = "center";
          ctx.fillText(label, midX, zoneBandTop - 6);
        }
      });
      // Crisp dividers at the zone edges for a clean readable boundary
      [TOO_CLOSE_M, TOO_FAR_M].forEach((g) => {
        if (g > maxGapSeen) return;
        const x = gapToPxX(g);
        ctx.strokeStyle = "#ffffff22";
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x, zoneBandTop);
        ctx.lineTo(x, zoneBandBot);
        ctx.stroke();
        ctx.setLineDash([]);
      });

      // ── Pilot boat silhouette — TRUE METRIC beam and length ────────
      // Drawn at the shared pxPerM so the captain can visually compare
      // "my beam" to "gap to cargo" at a glance.
      const bowVisible = geom.bow_x <= hullMax && geom.bow_x >= hullMin;
      const sternVisible = geom.rear_x <= hullMax && geom.rear_x >= hullMin;
      const byBowFull = boatXToPy(geom.bow_x);
      const byBowTaperFull = boatXToPy(geom.bow_x - 0.08 * boatLenM);
      const bySternFull = boatXToPy(geom.rear_x);
      const yTop = Math.max(plotTop, byBowTaperFull);
      const yBot = Math.min(plotBottom, bySternFull);

      ctx.save();
      ctx.beginPath();
      ctx.rect(portEdgeX - 2, plotTop, (starboardEdgeX - portEdgeX) + 4,
               plotBottom - plotTop);
      ctx.clip();

      ctx.fillStyle = "#141a2a";
      ctx.strokeStyle = PYXIS_ACCENT + "cc";
      ctx.lineWidth = 2;
      ctx.beginPath();
      if (bowVisible) {
        ctx.moveTo(centreX, byBowFull);                 // bow tip
        ctx.lineTo(starboardEdgeX, byBowTaperFull);      // stbd taper
        ctx.lineTo(starboardEdgeX, yBot);                // stbd stern
        ctx.lineTo(portEdgeX, yBot);                     // port stern
        ctx.lineTo(portEdgeX, byBowTaperFull);           // port taper
        ctx.closePath();
      } else {
        ctx.rect(portEdgeX, yTop, starboardEdgeX - portEdgeX, yBot - yTop);
      }
      ctx.fill();
      ctx.stroke();

      // Starboard edge highlight = gap 0 reference
      ctx.strokeStyle = PYXIS_ACCENT;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(starboardEdgeX, bowVisible ? byBowTaperFull : yTop);
      ctx.lineTo(starboardEdgeX, yBot);
      ctx.stroke();

      // Centreline (soft, for visual balance)
      ctx.strokeStyle = PYXIS_ACCENT + "33";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(centreX, bowVisible ? byBowFull : yTop);
      ctx.lineTo(centreX, yBot);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();

      // Off-window arrows
      ctx.fillStyle = PYXIS_ACCENT + "99";
      if (!bowVisible) {
        ctx.beginPath();
        ctx.moveTo(centreX, plotTop - 2);
        ctx.lineTo(centreX - 6, plotTop + 8);
        ctx.lineTo(centreX + 6, plotTop + 8);
        ctx.closePath();
        ctx.fill();
      }
      if (!sternVisible) {
        ctx.beginPath();
        ctx.moveTo(centreX, plotBottom + 2);
        ctx.lineTo(centreX - 6, plotBottom - 8);
        ctx.lineTo(centreX + 6, plotBottom - 8);
        ctx.closePath();
        ctx.fill();
      }

      // LiDAR mount marker (only if visible)
      if (geom.lidar_x <= hullMax && geom.lidar_x >= hullMin) {
        const lidarPy = boatXToPy(geom.lidar_x);
        const lidarPx = centreX + geom.lidar_y * pxPerM;
        ctx.fillStyle = "#ffaa00";
        ctx.beginPath();
        ctx.arc(lidarPx, lidarPy, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      // Bow / stern labels next to the centreline at top/bottom of boat
      ctx.fillStyle = "#88a0b8";
      ctx.font = "bold 11px 'JetBrains Mono', monospace";
      ctx.textAlign = "center";
      if (bowVisible) {
        ctx.fillText("BOW", centreX, byBowFull - 4);
      } else {
        ctx.fillText("↑ BOW", centreX, plotTop + 12);
      }
      if (sternVisible) {
        ctx.fillText("STERN", centreX, bySternFull + 12);
      } else {
        ctx.fillText("↓ STERN", centreX, plotBottom - 4);
      }

      // Hull-axis tick labels on the far right (metres aft of bow)
      const hullTickStep = hullWindowM > 6 ? 1.0 : 0.5;
      const tickStart = Math.ceil(hullMin / hullTickStep) * hullTickStep;
      ctx.fillStyle = "#445566";
      ctx.font = "11px 'JetBrains Mono', monospace";
      ctx.textAlign = "left";
      for (let hx = tickStart; hx <= hullMax + 1e-6; hx += hullTickStep) {
        const py = boatXToPy(hx);
        if (py < plotTop || py > plotBottom) continue;
        const fromBow = geom.bow_x - hx;
        ctx.fillText(`${fromBow.toFixed(1)}m`, plotRight + 4, py + 3);
      }

      // ── Cargo points — rendered at their true metric position ─────
      let drawn = 0;
      for (let i = 0; i < hullXsIn.length; i++) {
        const hx = hullXsIn[i];
        const g = gapsIn[i];
        if (hx < hullMin || hx > hullMax) continue;
        if (g > maxGapSeen) continue;
        const px = gapToPxX(g);
        const py = boatXToPy(hx);
        ctx.fillStyle = gapColor(g) + "dd";
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);
        ctx.fill();
        drawn++;
      }

      // ── Scale bar (1 m — identical on both axes, isotropic) ──────
      const oneM = pxPerM;
      const scaleY = h - 16;
      const scaleStart = plotLeft + 10;
      ctx.strokeStyle = "#556677";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(scaleStart, scaleY);
      ctx.lineTo(scaleStart + oneM, scaleY);
      ctx.moveTo(scaleStart, scaleY - 4);
      ctx.lineTo(scaleStart, scaleY + 4);
      ctx.moveTo(scaleStart + oneM, scaleY - 4);
      ctx.lineTo(scaleStart + oneM, scaleY + 4);
      ctx.stroke();
      ctx.fillStyle = "#889aa8";
      ctx.font = "11px 'JetBrains Mono', monospace";
      ctx.textAlign = "left";
      ctx.fillText("1 m (true scale)", scaleStart + oneM + 6, scaleY + 3);

      // ── Top caption ────────────────────────────────────────────────
      const nPoints = hullXsIn.length;
      const beamPx = beamM * pxPerM;
      ctx.fillStyle = "#556677";
      ctx.font = "bold 13px 'Source Sans Pro', sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(
        `TOP-DOWN  •  ${phase}  •  ${drawn}/${nPoints} pts  •  ${pxPerM.toFixed(0)} px/m  •  beam ${beamM.toFixed(1)}m = ${beamPx.toFixed(0)}px`,
        w / 2,
        16
      );

      if (nPoints === 0) {
        ctx.fillStyle = "#445566";
        ctx.font = "13px 'Source Sans Pro', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText(
          "Awaiting cargo-side LiDAR points...",
          (starboardEdgeX + plotRight) / 2,
          (plotTop + plotBottom) / 2
        );
      }

      return; // skip the APPROACHING fallback layout below
    }

    // Not in a gap view → reset smoother so next entry starts fresh.
    viewRef.current.hullMin = null;
    viewRef.current.hullMax = null;
    viewRef.current.gapMax = null;

    // ── APPROACHING / unknown phase: original heading-oriented view ──

    // ── Layout maths ─────────────────────────────────────────────────
    const marginL = 70;
    const marginR = 70;
    const hullLeft = marginL;
    const hullRight = w - marginR;
    const hullPxLen = hullRight - hullLeft;
    const mToPx = hullPxLen / HULL_LEN_M;
    const posToX = (posM) => hullLeft + posM * mToPx;

    // Pilot boat starboard edge Y (world frame, horizontal reference).
    // Bottom of canvas reserves 46px for the PILOT BOAT label + scale bar,
    // then the boat sits directly above that.
    const beamPx = BOAT_BEAM_PX;
    const bottomReserve = 46;
    const boatStarboardY = h - bottomReserve - beamPx;

    // Gap area: between top of canvas and boat starboard edge
    const gapAreaTop = 36;
    const gapAreaBottom = boatStarboardY - 8;
    const gapAreaH = Math.max(gapAreaBottom - gapAreaTop, 10);
    const gapToPx = gapAreaH / MAX_GAP_M;

    // Mount pivot (for boat rotation + ladder sightline origin)
    const mountPosM = HULL_LEN_M * MOUNT_FRAC_FROM_BOW;
    const mountX = posToX(mountPosM);
    const mountY = boatStarboardY;

    const headingRad = ((approachHeading || 0) * Math.PI) / 180;

    // ── Gap axis grid + labels (subtle) ──────────────────────────────
    const gapContourY = (gapM) =>
      gapAreaBottom - Math.min(Math.max(gapM, 0), MAX_GAP_M) * gapToPx;

    ctx.strokeStyle = "#141a2a";
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 4]);
    [0.5, 1.0, 1.5, 2.0].forEach((g) => {
      const y = gapContourY(g);
      ctx.beginPath();
      ctx.moveTo(hullLeft - 10, y);
      ctx.lineTo(hullRight + 10, y);
      ctx.stroke();
    });
    ctx.setLineDash([]);
    ctx.fillStyle = "#334055";
    ctx.font = "12px 'JetBrains Mono', monospace";
    ctx.textAlign = "left";
    [0.5, 1.0, 1.5, 2.0].forEach((g) => {
      ctx.fillText(`${g.toFixed(1)} m`, hullRight + 14, gapContourY(g) + 3);
    });

    // ── Cargo hull contour (horizontal, world-frame reference) ───────
    const profile = approachProfile?.hull_profile;
    const cargoDetected = approachProfile?.cargo_detected ?? false;

    if (cargoDetected && profile && profile.length > 0) {
      const valid = profile
        .filter((s) => s.zone !== "NO_DATA" && s.gap_m >= 0)
        .sort((a, b) => a.position_m - b.position_m);

      if (valid.length >= 2) {
        const pts = valid.map((s) => ({
          x: posToX(s.position_m),
          y: gapContourY(s.gap_m),
          zone: s.zone,
        }));

        // Fill "cargo body" (the mass above the contour) as a dark region
        ctx.beginPath();
        ctx.moveTo(pts[0].x, 0);
        pts.forEach((p) => ctx.lineTo(p.x, p.y));
        ctx.lineTo(pts[pts.length - 1].x, 0);
        ctx.closePath();
        const gradient = ctx.createLinearGradient(0, 0, 0, gapAreaBottom);
        gradient.addColorStop(0, "#0f1524");
        gradient.addColorStop(1, "#1a2338");
        ctx.fillStyle = gradient;
        ctx.fill();

        // Contour stroke, segment-coloured by per-bin zone
        ctx.lineWidth = 3;
        ctx.lineCap = "round";
        ctx.lineJoin = "round";
        for (let i = 0; i < pts.length - 1; i++) {
          const a = pts[i];
          const b = pts[i + 1];
          ctx.strokeStyle = ZONE_COLORS[a.zone] || "#555";
          ctx.beginPath();
          ctx.moveTo(a.x, a.y);
          ctx.lineTo(b.x, b.y);
          ctx.stroke();
        }

        // Segment tick-dots at each sample
        pts.forEach((p) => {
          ctx.fillStyle = ZONE_COLORS[p.zone] || "#555";
          ctx.beginPath();
          ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
          ctx.fill();
        });

        // "CARGO HULL" label
        ctx.fillStyle = "#556677";
        ctx.font = "bold 13px 'Source Sans Pro', sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("CARGO HULL", w / 2, 20);
      }
    } else {
      // Placeholder strip
      ctx.strokeStyle = "#22304a";
      ctx.lineWidth = 1;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.moveTo(hullLeft, gapContourY(1.0));
      ctx.lineTo(hullRight, gapContourY(1.0));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#445566";
      ctx.font = "13px 'Source Sans Pro', sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(
        "Awaiting hull profile — cargo not detected",
        w / 2,
        gapContourY(1.0) - 10
      );
    }

    // ── Pilot boat (rotated by heading around mount pivot) ───────────
    ctx.save();
    ctx.translate(mountX, mountY);
    ctx.rotate(headingRad);

    // Local frame: origin at mount, starboard edge at y=0, +y=port side,
    // -x=bow direction, +x=stern direction
    const bowX = -mountPosM * mToPx;
    const sternX = (HULL_LEN_M - mountPosM) * mToPx;
    const bowTaperEnd = bowX + 0.12 * HULL_LEN_M * mToPx;

    // Hull outline
    ctx.fillStyle = "#141a2a";
    ctx.strokeStyle = PYXIS_ACCENT + "cc";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sternX, 0); // stern starboard
    ctx.lineTo(bowTaperEnd, 0); // along starboard to bow taper
    ctx.lineTo(bowX, beamPx / 2); // bow tip (mid-beam)
    ctx.lineTo(bowTaperEnd, beamPx); // bow taper to port
    ctx.lineTo(sternX, beamPx); // stern port
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Cabin (toward stern, darker rectangle)
    ctx.fillStyle = "#1e2740";
    ctx.strokeStyle = "#2a3550";
    ctx.lineWidth = 1;
    const cabinX = sternX - 0.35 * HULL_LEN_M * mToPx;
    const cabinW = 0.25 * HULL_LEN_M * mToPx;
    const cabinY = beamPx * 0.22;
    const cabinH = beamPx * 0.56;
    ctx.fillRect(cabinX, cabinY, cabinW, cabinH);
    ctx.strokeRect(cabinX, cabinY, cabinW, cabinH);

    // Bow centerline mark (visual confirmation of heading direction)
    ctx.strokeStyle = PYXIS_ACCENT + "88";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(bowX, beamPx / 2);
    ctx.lineTo(bowTaperEnd + 0.1 * HULL_LEN_M * mToPx, beamPx / 2);
    ctx.stroke();

    // Mount pivot marker
    ctx.fillStyle = PYXIS_ACCENT;
    ctx.beginPath();
    ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
    ctx.fill();

    // ── Ladder sightline + lock marker (drawn in boat-local frame) ──
    // Sightline origin: mount (0,0). Bearing convention:
    //   yaw = 0 → directly starboard (-y in local = toward cargo)
    //   yaw > 0 → rotated toward bow (−x direction)
    // Length scaled by gapToPx so it reads naturally against the gap axis.
    if (gimbalYaw !== null && ladderDistance !== null && ladderDistance > 0) {
      const yawRad = (gimbalYaw * Math.PI) / 180;
      const rangePx =
        Math.min(Math.max(ladderDistance, 0.1), MAX_GAP_M * 1.4) * gapToPx;
      const lx = -Math.sin(yawRad) * rangePx;
      const ly = -Math.cos(yawRad) * rangePx;

      // Dashed sightline from mount
      ctx.strokeStyle = PYXIS_ACCENT + "55";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(lx, ly);
      ctx.stroke();
      ctx.setLineDash([]);

      // Ladder marker with lock ring
      const lockColor = gimbalTargetLocked ? "#00ff88" : "#ffaa00";
      ctx.fillStyle = lockColor;
      ctx.beginPath();
      ctx.arc(lx, ly, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = lockColor;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(lx, ly, 9, 0, Math.PI * 2);
      ctx.stroke();
      if (!gimbalTargetLocked) {
        // Dashed outer ring = searching
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(lx, ly, 14, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    }

    ctx.restore();

    // ── Parallel reference line (shows "perfectly parallel" when skewed)
    if (approachHeading !== null && Math.abs(approachHeading) > 0.3) {
      ctx.strokeStyle = "#2a3550";
      ctx.lineWidth = 1;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(hullLeft - 30, boatStarboardY);
      ctx.lineTo(hullRight + 30, boatStarboardY);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // ── Pilot boat label (between boat and scale bar) ────────────────
    ctx.fillStyle = PYXIS_ACCENT + "bb";
    ctx.font = "bold 12px 'Source Sans Pro', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("PILOT BOAT", w / 2, boatStarboardY + beamPx + 16);

    // ── Scale bar (true metric along hull axis) ──────────────────────
    const oneM = mToPx;
    const scaleY = h - 10;
    const scaleStart = w / 2 - oneM / 2;
    ctx.strokeStyle = "#445566";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(scaleStart, scaleY);
    ctx.lineTo(scaleStart + oneM, scaleY);
    ctx.moveTo(scaleStart, scaleY - 4);
    ctx.lineTo(scaleStart, scaleY + 4);
    ctx.moveTo(scaleStart + oneM, scaleY - 4);
    ctx.lineTo(scaleStart + oneM, scaleY + 4);
    ctx.stroke();
    ctx.fillStyle = "#556677";
    ctx.font = "11px 'JetBrains Mono', monospace";
    ctx.textAlign = "left";
    ctx.fillText("1 m (hull axis)", scaleStart + oneM + 6, scaleY + 3);
  }, [
    size,
    distance,
    zone,
    ladderDistance,
    approachHeading,
    approachZone,
    approachProfile,
    gimbalYaw,
    boatPitch,
    boatRoll,
    gimbalTargetLocked,
    phase,
    cargoPoints,
    boatGeometry,
  ]);

  return (
    <Box
      sx={{
        flex: 1,
        display: "flex",
        flexDirection: "column",
        minHeight: 0,
      }}
    >
      {/* Distance strip at top — single source of truth for numerics */}
      <Box
        sx={{
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
          gap: 4,
          py: 1,
          backgroundColor: "#0d1117",
          borderBottom: "1px solid #1a2035",
        }}
      >
        <Box sx={{ textAlign: "center" }}>
          <Typography sx={{ fontSize: "14px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            HULL DISTANCE
          </Typography>
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "28px",
              fontWeight: 700,
              color:
                zone === "TOO_CLOSE"
                  ? "#ff2222"
                  : zone === "OPTIMAL"
                  ? "#00ff88"
                  : zone === "TOO_FAR"
                  ? "#ffaa00"
                  : "#555",
            }}
          >
            {distance !== null ? `${distance.toFixed(2)} m` : "--.- m"}
          </Typography>
        </Box>
        <Box sx={{ textAlign: "center" }}>
          <Typography sx={{ fontSize: "14px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            LADDER DISTANCE
          </Typography>
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "28px",
              fontWeight: 700,
              color: ladderDistance !== null ? PYXIS_ACCENT : "#555",
            }}
          >
            {ladderDistance !== null ? `${ladderDistance.toFixed(1)} m` : "--.- m"}
          </Typography>
        </Box>
        <Box sx={{ textAlign: "center" }}>
          <Typography sx={{ fontSize: "14px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            HEADING
          </Typography>
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "28px",
              fontWeight: 700,
              color:
                approachHeading !== null
                  ? Math.abs(approachHeading) < 3
                    ? "#00ff88"
                    : Math.abs(approachHeading) < 8
                    ? "#ffaa00"
                    : "#ff2222"
                  : "#555",
            }}
          >
            {approachHeading !== null
              ? `${approachHeading > 0 ? "+" : ""}${approachHeading.toFixed(1)}°`
              : "--.-°"}
          </Typography>
        </Box>
        <Box sx={{ textAlign: "center" }}>
          <Typography sx={{ fontSize: "14px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            GIMBAL LOCK
          </Typography>
          <Typography
            sx={{
              fontFamily: "'Source Sans Pro', sans-serif",
              fontSize: "22px",
              fontWeight: 700,
              color: gimbalTargetLocked ? "#00ff88" : "#ffaa00",
              letterSpacing: "1px",
            }}
          >
            {gimbalTargetLocked ? "LOCKED" : "SEARCH"}
          </Typography>
        </Box>
      </Box>

      {/* Canvas schematic */}
      <Box ref={containerRef} sx={{ flex: 1, position: "relative", minHeight: 0 }}>
        <canvas
          ref={canvasRef}
          style={{
            display: "block",
          }}
        />
      </Box>
    </Box>
  );
};

export default React.memo(SpatialView);
