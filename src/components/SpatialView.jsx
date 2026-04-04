import React, { useRef, useEffect } from "react";
import { Box, Typography } from "@mui/material";
import { PYXIS_ACCENT } from "../config";

/**
 * Spatial / Bird's Eye guidance view.
 * Top-down schematic showing:
 *   - Cargo ship hull (top, long rectangle)
 *   - Pilot boat (bottom, boat outline)
 *   - Hull gap profile segments (colour-coded bars between them)
 *   - Heading error indicator
 *   - Ladder position
 *   - Distance annotations
 *
 * All data comes from /approach/profile JSON via useMaritimeData.
 */

const ZONE_COLORS = {
  TOO_CLOSE: "#ff2222",
  OPTIMAL: "#00ff88",
  TOO_FAR: "#dd8800",
  NO_DATA: "#333",
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
}) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const parent = canvas.parentElement;
    const w = parent.clientWidth;
    const h = parent.clientHeight;
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d");

    // Clear
    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(0, 0, w, h);

    // Layout constants
    const margin = 60;
    const cargoY = h * 0.12;           // cargo ship top
    const cargoH = 40;                  // cargo ship height (thickness)
    const boatY = h * 0.78;            // pilot boat center Y
    const boatH = 50;                   // pilot boat height
    const hullLen = 13.053;             // ferry hull length (m)
    const profileStep = 0.5;

    // Horizontal mapping: hull position (m) → canvas X
    const hullLeft = margin + 80;
    const hullRight = w - margin - 80;
    const hullPxLen = hullRight - hullLeft;
    const mToPx = hullPxLen / hullLen;

    const posToX = (posM) => hullLeft + posM * mToPx;

    // ── Draw cargo ship ──
    ctx.fillStyle = "#1a2035";
    ctx.strokeStyle = "#334";
    ctx.lineWidth = 2;
    // Rounded rect
    const cr = 6;
    ctx.beginPath();
    ctx.roundRect(hullLeft - 20, cargoY, hullPxLen + 40, cargoH, cr);
    ctx.fill();
    ctx.stroke();

    // Cargo label
    ctx.fillStyle = "#667";
    ctx.font = "bold 14px Source Sans Pro";
    ctx.textAlign = "center";
    ctx.fillText("CARGO VESSEL", w / 2, cargoY - 12);

    // ── Draw hull gap profile segments ──
    const profile = approachProfile?.hull_profile;
    const gapAreaTop = cargoY + cargoH + 8;
    const gapAreaBottom = boatY - boatH / 2 - 8;
    const gapAreaH = gapAreaBottom - gapAreaTop;

    if (profile && profile.length > 0) {
      profile.forEach((seg) => {
        if (seg.zone === "NO_DATA" || seg.gap_m < 0) return;
        const x = posToX(seg.position_m);
        const segW = Math.max(profileStep * mToPx - 2, 4);
        const color = ZONE_COLORS[seg.zone] || ZONE_COLORS.NO_DATA;

        // Bar height proportional to gap (max ~2m displayed)
        const barH = Math.min(seg.gap_m / 2.0, 1.0) * gapAreaH;

        ctx.fillStyle = color + "60"; // semi-transparent fill
        ctx.fillRect(x, gapAreaTop, segW, barH);

        ctx.strokeStyle = color + "aa";
        ctx.lineWidth = 1;
        ctx.strokeRect(x, gapAreaTop, segW, barH);
      });

      // Draw the gap scale on the left
      ctx.fillStyle = "#556";
      ctx.font = "12px JetBrains Mono";
      ctx.textAlign = "right";
      ctx.fillText("0m", hullLeft - 30, gapAreaTop + 5);
      ctx.fillText("1m", hullLeft - 30, gapAreaTop + gapAreaH * 0.5 + 5);
      ctx.fillText("2m", hullLeft - 30, gapAreaTop + gapAreaH + 5);
    } else {
      // No profile data — show placeholder
      ctx.fillStyle = "#334";
      ctx.setLineDash([6, 6]);
      ctx.strokeStyle = "#445";
      ctx.lineWidth = 1;
      for (let i = 0; i < hullLen / profileStep; i++) {
        const x = posToX(i * profileStep);
        const segW = Math.max(profileStep * mToPx - 2, 4);
        ctx.strokeRect(x, gapAreaTop, segW, gapAreaH * 0.3);
      }
      ctx.setLineDash([]);

      ctx.fillStyle = "#556";
      ctx.font = "16px Source Sans Pro";
      ctx.textAlign = "center";
      ctx.fillText("Awaiting hull profile data...", w / 2, gapAreaTop + gapAreaH * 0.5);
    }

    // ── Draw pilot boat (simplified) ──
    // Geometry from ROS frame: mount at (0,0), bow at (-1.22, 2.94), hull 13m
    // In spatial view: bow LEFT, starboard (camera side) UP toward cargo
    const boatCx = w / 2;
    const boatW = hullPxLen * 0.65;
    const boatLeft = boatCx - boatW / 2;

    ctx.fillStyle = "#1a2035";
    ctx.strokeStyle = PYXIS_ACCENT + "88";
    ctx.lineWidth = 2;

    // Bow points LEFT, stern RIGHT, starboard faces UP toward cargo
    const bowTipX = boatLeft;
    const bowTipY = boatY;
    ctx.beginPath();
    ctx.moveTo(boatLeft + boatW, boatY - boatH / 2);                // stern top (starboard)
    ctx.lineTo(boatLeft + boatW * 0.15, boatY - boatH / 2);         // along starboard edge
    ctx.lineTo(bowTipX, bowTipY);                                    // bow point
    ctx.lineTo(boatLeft + boatW * 0.15, boatY + boatH / 2);         // along port edge
    ctx.lineTo(boatLeft + boatW, boatY + boatH / 2);                // stern bottom (port)
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Cabin (toward stern)
    ctx.fillStyle = "#2a3050";
    ctx.fillRect(boatLeft + boatW * 0.55, boatY - boatH * 0.3, boatW * 0.25, boatH * 0.6);

    // Mount position marker (small dot, roughly 17% from bow along hull)
    const mountX = boatLeft + boatW * 0.17;
    ctx.fillStyle = "#556";
    ctx.beginPath();
    ctx.arc(mountX, boatY - boatH / 2 + 4, 3, 0, Math.PI * 2);
    ctx.fill();

    // Pilot boarding marker — ~2.2m forward of mount, near front starboard
    // In the boat shape this is ~13% from bow tip (forward of mount)
    const boardingX = boatLeft + boatW * 0.12;
    ctx.strokeStyle = PYXIS_ACCENT;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(boardingX, boatY - boatH / 2 - 5);
    ctx.lineTo(boardingX, cargoY + cargoH + 5);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = PYXIS_ACCENT;
    ctx.font = "bold 11px Source Sans Pro";
    ctx.textAlign = "center";
    ctx.fillText("PILOT", boardingX, boatY - boatH / 2 - 14);
    ctx.fillText("BOARDING", boardingX, boatY - boatH / 2 - 3);

    // Boat label
    ctx.fillStyle = PYXIS_ACCENT;
    ctx.font = "bold 13px Source Sans Pro";
    ctx.textAlign = "center";
    ctx.fillText("PILOT BOAT", boatCx, boatY + boatH / 2 + 42);

    // ── Heading error indicator (at bow) ──
    if (approachHeading !== null) {
      const headingColor = Math.abs(approachHeading) < 3 ? "#00ff88"
        : Math.abs(approachHeading) < 8 ? "#ffaa00" : "#ff2222";

      // Reference line: straight up from bow (parallel to cargo = 0° heading)
      const refLineLen = 45;
      ctx.strokeStyle = "#334";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(bowTipX, bowTipY);
      ctx.lineTo(bowTipX, bowTipY - refLineLen);
      ctx.stroke();
      ctx.setLineDash([]);

      // Heading arrow from bow tip — angle shows deviation from parallel
      // In spatial view: 0° = straight up (parallel), positive = rotated clockwise
      const lineLen = 55;
      const angleRad = (approachHeading * Math.PI) / 180;
      const endX = bowTipX - Math.sin(angleRad) * lineLen;  // negative because bow points left
      const endY = bowTipY - Math.cos(angleRad) * lineLen;

      ctx.strokeStyle = headingColor;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(bowTipX, bowTipY);
      ctx.lineTo(endX, endY);
      ctx.stroke();

      // Arrowhead
      const arrowLen = 10;
      const arrowAngle = 0.4;
      ctx.beginPath();
      ctx.moveTo(endX, endY);
      ctx.lineTo(
        endX + arrowLen * Math.sin(angleRad - arrowAngle),
        endY + arrowLen * Math.cos(angleRad - arrowAngle)
      );
      ctx.moveTo(endX, endY);
      ctx.lineTo(
        endX + arrowLen * Math.sin(angleRad + arrowAngle),
        endY + arrowLen * Math.cos(angleRad + arrowAngle)
      );
      ctx.stroke();

      // Heading value label next to bow
      ctx.fillStyle = headingColor;
      ctx.font = "bold 15px JetBrains Mono";
      ctx.textAlign = "left";
      ctx.fillText(
        `${approachHeading > 0 ? "+" : ""}${approachHeading.toFixed(1)}°`,
        bowTipX + 8,
        boatY + boatH / 2 + 18
      );
    }

    // ── Distance annotations ──
    // Main distance (right side)
    const infoX = w - margin + 10;
    ctx.textAlign = "left";

    // Hull distance
    const distColor = zone === "TOO_CLOSE" ? "#ff2222"
      : zone === "OPTIMAL" ? "#00ff88"
      : zone === "TOO_FAR" ? "#ffaa00" : "#555";

    ctx.fillStyle = "#667";
    ctx.font = "bold 13px Source Sans Pro";
    ctx.fillText("HULL DIST", infoX - 50, cargoY + cargoH + 30);
    ctx.fillStyle = distColor;
    ctx.font = "bold 32px JetBrains Mono";
    ctx.fillText(
      distance !== null ? `${distance.toFixed(2)}m` : "--.-m",
      infoX - 50,
      cargoY + cargoH + 62
    );

    // Zone
    ctx.fillStyle = distColor;
    ctx.font = "bold 14px Source Sans Pro";
    ctx.fillText(
      zone || "---",
      infoX - 50,
      cargoY + cargoH + 80
    );

    // Ladder distance
    ctx.fillStyle = "#667";
    ctx.font = "bold 13px Source Sans Pro";
    ctx.fillText("LADDER DIST", infoX - 50, cargoY + cargoH + 115);
    ctx.fillStyle = ladderDistance !== null ? PYXIS_ACCENT : "#555";
    ctx.font = "bold 24px JetBrains Mono";
    ctx.fillText(
      ladderDistance !== null ? `${ladderDistance.toFixed(1)}m` : "--.-m",
      infoX - 50,
      cargoY + cargoH + 142
    );

    // ── Gimbal indicator (left side info) ──
    const leftInfoX = margin - 20;
    ctx.textAlign = "left";
    ctx.fillStyle = "#667";
    ctx.font = "bold 13px Source Sans Pro";

    if (gimbalYaw !== null) {
      ctx.fillText("GIMBAL YAW", leftInfoX, boatY - 30);
      ctx.fillStyle = PYXIS_ACCENT;
      ctx.font = "bold 20px JetBrains Mono";
      ctx.fillText(`${gimbalYaw > 0 ? "+" : ""}${gimbalYaw.toFixed(0)}°`, leftInfoX, boatY - 8);
    }

    ctx.fillStyle = "#667";
    ctx.font = "bold 13px Source Sans Pro";
    if (boatRoll !== null) {
      ctx.fillText("TILT", leftInfoX, boatY + 20);
      ctx.fillStyle = "#aaa";
      ctx.font = "16px JetBrains Mono";
      ctx.fillText(
        `R${boatRoll.toFixed(1)}° P${(boatPitch || 0).toFixed(1)}°`,
        leftInfoX,
        boatY + 40
      );
    }

    // ── Approach zone indicator ──
    if (approachProfile) {
      const azColor = ZONE_COLORS[approachZone] || "#555";
      ctx.fillStyle = "#667";
      ctx.font = "bold 13px Source Sans Pro";
      ctx.textAlign = "left";
      ctx.fillText("APPROACH", infoX - 50, boatY - 10);
      ctx.fillStyle = azColor;
      ctx.font = "bold 18px Source Sans Pro";
      ctx.fillText(approachZone || "---", infoX - 50, boatY + 12);

      // R² quality
      if (approachProfile.fit_r_squared != null) {
        ctx.fillStyle = "#556";
        ctx.font = "12px JetBrains Mono";
        ctx.fillText(
          `R² ${approachProfile.fit_r_squared.toFixed(3)}`,
          infoX - 50,
          boatY + 30
        );
      }
    }

    // ── Scale bar at bottom ──
    ctx.strokeStyle = "#445";
    ctx.lineWidth = 1;
    ctx.fillStyle = "#556";
    ctx.font = "12px JetBrains Mono";
    ctx.textAlign = "center";
    const scaleY2 = h - 25;
    // 1m scale bar
    const oneM = mToPx;
    const scaleStart = w / 2 - oneM / 2;
    ctx.beginPath();
    ctx.moveTo(scaleStart, scaleY2);
    ctx.lineTo(scaleStart + oneM, scaleY2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(scaleStart, scaleY2 - 5);
    ctx.lineTo(scaleStart, scaleY2 + 5);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(scaleStart + oneM, scaleY2 - 5);
    ctx.lineTo(scaleStart + oneM, scaleY2 + 5);
    ctx.stroke();
    ctx.fillText("1 metre", w / 2, scaleY2 - 10);

  }, [distance, zone, ladderDistance, approachHeading, approachZone, approachProfile, gimbalYaw, boatPitch, boatRoll]);

  // Re-render on resize
  useEffect(() => {
    const handleResize = () => {
      const canvas = canvasRef.current;
      if (canvas) {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
      }
    };
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  return (
    <Box
      sx={{
        flex: 1,
        display: "flex",
        flexDirection: "column",
        minHeight: 0,
      }}
    >
      {/* Distance strip at top */}
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
          <Typography sx={{ fontSize: "12px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            HULL DISTANCE
          </Typography>
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "28px",
              fontWeight: 700,
              color: zone === "TOO_CLOSE" ? "#ff2222" : zone === "OPTIMAL" ? "#00ff88" : zone === "TOO_FAR" ? "#ffaa00" : "#555",
            }}
          >
            {distance !== null ? `${distance.toFixed(2)} m` : "--.- m"}
          </Typography>
        </Box>
        <Box sx={{ textAlign: "center" }}>
          <Typography sx={{ fontSize: "12px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
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
          <Typography sx={{ fontSize: "12px", color: "#667", fontWeight: 600, letterSpacing: "2px" }}>
            HEADING
          </Typography>
          <Typography
            sx={{
              fontFamily: "'JetBrains Mono', monospace",
              fontSize: "28px",
              fontWeight: 700,
              color: approachHeading !== null
                ? Math.abs(approachHeading) < 3 ? "#00ff88" : Math.abs(approachHeading) < 8 ? "#ffaa00" : "#ff2222"
                : "#555",
            }}
          >
            {approachHeading !== null ? `${approachHeading > 0 ? "+" : ""}${approachHeading.toFixed(1)}°` : "--.-°"}
          </Typography>
        </Box>
      </Box>

      {/* Canvas schematic */}
      <Box sx={{ flex: 1, position: "relative", minHeight: 0 }}>
        <canvas
          ref={canvasRef}
          style={{
            width: "100%",
            height: "100%",
            display: "block",
          }}
        />
      </Box>
    </Box>
  );
};

export default React.memo(SpatialView);
