# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A **Maritime Pilot Transfer Safety Dashboard** — a full-screen React HMI for pilot boat captains. The React frontend is served from a Raspberry Pi 5 and displays real-time camera feeds, YOLO detections (from a Jetson Orin Nano), and LiDAR distance data from ROS2.

This repo is the **source repo**. The production deployment lives at `~/pyxis_dashboard/` on the RPi — the built `build/` directory must be copied there.

## Commands

```bash
# Development (from repo root)
npm install            # install dependencies
npm start              # dev server with hot reload (CRA, port 3000)
npm run build          # production build → build/
npm test               # Jest tests (watch mode)

# Deploy to RPi production
scp -r build/ pyxispi5@<rpi-ip>:/home/pyxispi5/pyxis_dashboard/build/

# On RPi: start all services
cd ~/pyxis_dashboard && ./start_dashboard.sh
```

## Port Map

| Port | Service |
|------|---------|
| 3000 | React static build (Python http.server) |
| 8080 | `cam1_relay.py` — Jetson H264 RTP → MJPEG |
| 8081 | `cam2_stream.py` — Local Arducam `/dev/video0` → MJPEG |
| 8765 | `ws_server.py` — WebSocket bridge (LiDAR + YOLO detections) |
| 5700 | UDP in — Jetson H264 RTP stream |
| 5106 | UDP in — Jetson YOLO detection JSON |

## Architecture

### Data Flow

```
Jetson Orin Nano
  ├── H264 RTP → UDP:5700 → cam1_relay.py → MJPEG HTTP:8080
  └── YOLO JSON → UDP:5106 ┐
                            ├→ ws_server.py → WebSocket:8765
ROS2 /proximity/distance ──┘       ↓
                              React Dashboard (browser)
                                ├── CameraFeed polling :8080 & :8081
                                └── useMaritimeData hook consuming WS
```

### `ws_server.py` — Central Data Hub

Runs two background threads:
- **UDP thread** — binds `:5106`, drains buffer keeping freshest Jetson packet, clears stale detections after 1s
- **ROS2 thread** — subscribes to `/proximity/distance` (Float32); gracefully skips if ROS2 unavailable

Broadcasts at **5 Hz** to all WebSocket clients:
```json
{
  "type": "frame_update",
  "lidar": { "distance": 1.23 },
  "cv_detections": [{ "label": "ladder", "confidence": 0.92, "bbox": [x, y, w, h] }],
  "source_resolution": { "w": 640, "h": 480 },
  "camera_urls": { "cam1": "...", "cam2": "..." }
}
```

### React Frontend (`src/`)

- **`config.js`** — Distance zone thresholds (danger ≤0.5m, caution ≤1.5m, safe ≤3.0m) and camera URLs. `WS_URL` auto-detects from `window.location.hostname`.
- **`theme.js`** — Dark maritime theme: safe=`#00ff88`, caution=`#ffaa00`, danger=`#ff2222`.
- **`services/WebSocketService.js`** — Singleton WS client with exponential backoff reconnect (1s→10s).
- **`hooks/useMaritimeData.js`** — Consumes WS messages, throttles to ~20fps via `requestAnimationFrame`, computes `alertLevel` from distance zones.
- **`scenes/dashboard/index.jsx`** — Main 2-column grid layout: left=cam1+CVOverlay+DistanceGauge, right=cam2.
- **`components/CVOverlay.jsx`** — Canvas overlay that scales YOLO `bbox` from `sourceResolution` to display resolution.

### Camera Services

Both `cam1_relay.py` and `cam2_stream.py` share the same pattern:
- GStreamer pipeline spawned via `subprocess.Popen`, stdout is JPEG frames
- Watchdog thread kills stalled pipeline (cam1: 10s timeout, cam2: 5s)
- Auto-restarts on error with 3s backoff
- `/stream` endpoint: multipart MJPEG; `/snapshot` endpoint: single JPEG
- FPS capped at 10fps to limit RPi CPU; JPEG quality=55

## Key Behaviors

- **Camera polling**: `CameraFeed` polls `/snapshot?t=<ms>` every 200ms (5fps) using cache-busting query param.
- **Alert audio**: Generated via Web Audio API `OscillatorNode` — 880Hz double-beep for danger, 660Hz single-beep for caution.
- **ROS2 optional**: `ws_server.py` catches `ImportError`/`ModuleNotFoundError` on `rclpy` import and falls back to distance=null.

## Environment

`.env` (not committed) sets camera URLs for development:
```
REACT_APP_WS_URL=ws://localhost:8765
REACT_APP_CAM1_URL=http://10.42.0.1:8080/stream
REACT_APP_CAM2_URL=http://localhost:8081/stream
```
