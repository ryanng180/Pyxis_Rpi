import React, { useRef, useEffect, useState, forwardRef, useImperativeHandle } from "react";
import { Box, Typography } from "@mui/material";
import VideocamOffIcon from "@mui/icons-material/VideocamOff";

/**
 * Camera feed via snapshot polling.
 * Fetches /snapshot at ~5fps with cache-busting query param.
 * Works reliably in Firefox, Chrome, and all browsers.
 */
const CameraFeed = forwardRef(({ source, label }, ref) => {
  const containerRef = useRef(null);
  const [error, setError] = useState(true); // start as error until first frame loads
  const [dimensions, setDimensions] = useState({ width: 0, height: 0 });
  const [imgSrc, setImgSrc] = useState("");

  useImperativeHandle(ref, () => ({
    getDimensions: () => dimensions,
    getContainer: () => containerRef.current,
  }));

  // Track container size
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver((entries) => {
      const { width, height } = entries[0].contentRect;
      setDimensions({ width, height });
    });
    observer.observe(container);
    return () => observer.disconnect();
  }, []);

  // Snapshot polling — simple cache-busting img src update
  useEffect(() => {
    if (source?.type !== "mjpeg" || !source.url) return;

    // Convert stream URL to snapshot URL
    const base = source.url.replace(/\/stream\/?$/, "").replace(/\/$/, "") + "/snapshot";

    // Set first frame immediately
    setImgSrc(base + "?t=" + Date.now());

    const interval = setInterval(() => {
      setImgSrc(base + "?t=" + Date.now());
    }, 200); // ~5fps polling

    return () => clearInterval(interval);
  }, [source?.url, source?.type]);

  return (
    <Box
      ref={containerRef}
      sx={{
        position: "relative",
        width: "100%",
        height: "100%",
        backgroundColor: "#000",
        overflow: "hidden",
      }}
    >
      {/* Camera label */}
      <Typography
        sx={{
          position: "absolute",
          top: 8,
          left: 12,
          zIndex: 10,
          color: "#fff",
          fontSize: "13px",
          fontWeight: 600,
          textTransform: "uppercase",
          letterSpacing: "0.5px",
          opacity: 0.7,
          textShadow: "0 1px 3px rgba(0,0,0,0.8)",
        }}
      >
        {label}
      </Typography>

      {/* Snapshot image */}
      {source?.type === "mjpeg" && imgSrc && (
        <img
          src={imgSrc}
          alt={label}
          onLoad={() => setError(false)}
          onError={() => setError(true)}
          style={{
            width: "100%",
            height: "100%",
            objectFit: "cover",
            display: error ? "none" : "block",
          }}
        />
      )}

      {/* No signal fallback */}
      {error && (
        <Box
          sx={{
            position: "absolute",
            inset: 0,
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "center",
            backgroundColor: "#0a0e1a",
          }}
        >
          <VideocamOffIcon sx={{ fontSize: 48, color: "#555", mb: 1 }} />
          <Typography
            sx={{
              color: "#555",
              fontSize: "18px",
              fontWeight: 600,
              letterSpacing: "2px",
            }}
          >
            NO SIGNAL
          </Typography>
        </Box>
      )}
    </Box>
  );
});

CameraFeed.displayName = "CameraFeed";

export default React.memo(CameraFeed);
