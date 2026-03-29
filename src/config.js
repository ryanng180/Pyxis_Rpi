// Maritime Dashboard Configuration
// All values configurable via environment variables for deployment flexibility

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

// Distance zones in meters
export const DISTANCE_ZONES = {
  danger: { max: 0.5, color: "#ff2222", label: "DANGER" },
  caution: { max: 1.5, color: "#ffaa00", label: "CAUTION" },
  safe: { max: 3.0, color: "#00ff88", label: "SAFE" },
};

export const ALERT_AUDIO_ENABLED = true;

// Max distance displayed on gauge (meters)
export const MAX_DISPLAY_DISTANCE = 5.0;
