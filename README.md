# Pyxis Dashboard

Maritime pilot transfer safety dashboard — full-screen HMI for pilot boat captains.

**Stack:** React 18 + MUI v5 + Emotion · GStreamer · Python 3 · ROS2 Jazzy

## Run (Production on RPi)

> The production build is deployed at `~/pyxis_dashboard/`. Run **after** the ROS2 stack is up.

```bash
cd ~/pyxis_dashboard
bash start_dashboard.sh
```

Open `http://localhost:3000` on the RPi. Ctrl+C to stop.

### Services

| Service | Port | Description |
|---|---|---|
| `cam1_relay.py` | 8080 | Jetson H264 RTP → MJPEG (GStreamer, 10fps) |
| `cam2_stream.py` | 8081 | Arducam `/dev/video0` → MJPEG (GStreamer, 10fps) |
| `ws_server.py` | 8765 | WebSocket bridge: ROS2 topics + Jetson UDP detections → 5Hz broadcast |
| `http.server` | 3000 | Serves React production build |

### View from another machine

```
http://<rpi-ip>:3000
```

## Build

Node.js is not on the RPi. Build on a dev machine, then copy:

```bash
# Dev machine
npm install
npm run build

# Copy to RPi
scp -r build/ pyxispi5@<rpi-ip>:~/pyxis_dashboard/build
```

Or if building directly on the RPi:

```bash
cd ~/pyxis-dashboard-repo
npm run build
cp -r build/ ~/pyxis_dashboard/build
```

## Troubleshooting

**Camera shows NO SIGNAL** — check hardware is connected (`lsusb`), wait 5-10s for GStreamer watchdog restart, or restart the stack.

**Port already in use:**
```bash
fuser -k 3000/tcp 8080/tcp 8081/tcp 8765/tcp
```

**Browser shows stale UI** — hard refresh with Ctrl+Shift+R (builds use hashed filenames, but browser may cache old version).
