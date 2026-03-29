import React, { useRef, useEffect } from "react";

/**
 * Canvas overlay for YOLO bounding box detections on Camera 1.
 *
 * Props:
 *   detections: [{ label, confidence, bbox: [x, y, w, h] }]
 *   width: number (canvas width)
 *   height: number (canvas height)
 *   sourceResolution: { w, h } (original frame resolution from Jetson)
 */
const LABEL_COLORS = {
  ladder: "#00ff88",
  pilot: "#ffaa00",
  person: "#ffaa00",
  default: "#2676ff",
};

const CVOverlay = ({ detections = [], width, height, sourceResolution }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !width || !height) return;

    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, width, height);

    if (!detections.length || !sourceResolution) return;

    const scaleX = width / sourceResolution.w;
    const scaleY = height / sourceResolution.h;

    detections.forEach(({ label, confidence, bbox }) => {
      const [bx, by, bw, bh] = bbox;
      const x = bx * scaleX;
      const y = by * scaleY;
      const w = bw * scaleX;
      const h = bh * scaleY;

      const color = LABEL_COLORS[label?.toLowerCase()] || LABEL_COLORS.default;

      // Bounding box
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.strokeRect(x, y, w, h);

      // Corner brackets for emphasis
      const cornerLen = Math.min(w, h) * 0.2;
      ctx.lineWidth = 4;
      // Top-left
      ctx.beginPath();
      ctx.moveTo(x, y + cornerLen);
      ctx.lineTo(x, y);
      ctx.lineTo(x + cornerLen, y);
      ctx.stroke();
      // Top-right
      ctx.beginPath();
      ctx.moveTo(x + w - cornerLen, y);
      ctx.lineTo(x + w, y);
      ctx.lineTo(x + w, y + cornerLen);
      ctx.stroke();
      // Bottom-left
      ctx.beginPath();
      ctx.moveTo(x, y + h - cornerLen);
      ctx.lineTo(x, y + h);
      ctx.lineTo(x + cornerLen, y + h);
      ctx.stroke();
      // Bottom-right
      ctx.beginPath();
      ctx.moveTo(x + w - cornerLen, y + h);
      ctx.lineTo(x + w, y + h);
      ctx.lineTo(x + w, y + h - cornerLen);
      ctx.stroke();

      // Label background
      const text = `${label} ${(confidence * 100).toFixed(0)}%`;
      ctx.font = "bold 14px Source Sans Pro";
      const textWidth = ctx.measureText(text).width;
      ctx.fillStyle = color;
      ctx.fillRect(x, y - 22, textWidth + 10, 22);

      // Label text
      ctx.fillStyle = "#000";
      ctx.fillText(text, x + 5, y - 6);
    });
  }, [detections, width, height, sourceResolution]);

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

export default React.memo(CVOverlay);
