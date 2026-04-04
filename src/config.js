// Maritime Dashboard Configuration

// Auto-detect WS host from browser location so the dashboard works whether
// the browser is on the RPi itself (localhost) or accessed remotely.
const _wsHost = process.env.REACT_APP_WS_URL
  ? null
  : (typeof window !== "undefined" ? window.location.hostname : "localhost");

export const WS_URL =
  process.env.REACT_APP_WS_URL ||
  `ws://${_wsHost}:8765`;

export const CAMERA_DEFAULTS = {
  cam1: process.env.REACT_APP_CAM1_URL || "http://localhost:8080/stream",
  cam2: process.env.REACT_APP_CAM2_URL || "http://localhost:8081/stream",
};

// Distance zones in meters — matches ROS2 proximity_node thresholds
// ROS2 publishes 3 zones: TOO_CLOSE / OPTIMAL / TOO_FAR
// Dashboard splits TOO_FAR into CLOSING (1-2.5m) and APPROACHING (>2.5m) for visual clarity
export const DISTANCE_ZONES = {
  danger:      { max: 0.3, color: "#ff2222", label: "TOO CLOSE" },
  optimal:     { max: 1.0, color: "#00ff88", label: "OPTIMAL" },
  closing:     { max: 2.5, color: "#dd8800", label: "CLOSING" },
  approaching: { max: 5.0, color: "#667788", label: "APPROACHING" },
};


export const ALERT_AUDIO_ENABLED = true;

// Max distance displayed on gauge (meters)
export const MAX_DISPLAY_DISTANCE = 5.0;

// Pyxis brand accent
export const PYXIS_ACCENT = "#00b4d8";
