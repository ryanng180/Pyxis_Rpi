# Pyxis Dashboard

Maritime pilot transfer safety dashboard — full-screen HMI for pilot boat captains.

**Features:** Dual camera feeds (Jetson YOLO + Arducam) · LiDAR distance gauge · CV bounding box overlays · Safety alerts · ROS2 WebSocket bridge

---

## Launching the Dashboard on the RPi

> The production build is already deployed at `/home/pyxispi5/pyxis_dashboard/`.
> Run this **after** the ROS2 stack is up.

```bash
cd /home/pyxispi5/pyxis_dashboard
bash start_dashboard.sh
```

Then open Firefox on the RPi and go to:

```
http://localhost:3000
```

Press **Ctrl+C** in the terminal to stop all dashboard processes.

---

## What `start_dashboard.sh` Starts

| Service | Port | Description |
|---|---|---|
| `cam1_relay.py` | 8080 | Receives Jetson H264 RTP stream, serves MJPEG snapshots |
| `cam2_stream.py` | 8081 | Reads Arducam B0589 (`/dev/video0`), serves MJPEG snapshots |
| `ws_server.py` | 8765 | WebSocket bridge — aggregates ROS2 `/proximity/distance` + UDP detections |
| Python HTTP server | 3000 | Serves the React production build as static files |

---

## Viewing from Another Machine on the Network

Find the RPi's IP:
```bash
hostname -I
```

Then on any browser on the same network:
```
http://<rpi-ip>:3000
```

---

## If You Need to Rebuild After Code Changes

The RPi does not have Node.js. Build on your dev machine, then transfer the build:

```bash
# On your dev machine (in the repo root)
npm install
npm run build

# Copy build to RPi
scp -r build/ pyxis-rpi:/home/pyxispi5/pyxis_dashboard/build
```

---

## Repo

```bash
# Clone (read-only, no GitHub account needed)
git clone https://github.com/Jungstershark/pyxis-dashboard.git

# Pull latest changes
cd pyxis-dashboard
git pull
```

The repo is cloned at `/home/pyxispi5/pyxis-dashboard-repo/` on the RPi for reference.

---

## Troubleshooting

**Cam2 shows NO SIGNAL / green screen**
- Check that Arducam is plugged in: `lsusb | grep Arducam`
- The script auto-recovers — wait ~5s for GStreamer to restart
- If it stays green, restart the stack: `Ctrl+C`, then `bash start_dashboard.sh` again

**Cam1 shows NO SIGNAL**
- Confirm the Jetson is streaming: it must be running its YOLO pipeline and sending RTP to `udp://10.42.0.1:5700`

**Port already in use**
```bash
fuser -k 3000/tcp 8080/tcp 8081/tcp 8765/tcp
```

**WebSocket not connecting**
```bash
# Check ws_server is running
ps aux | grep ws_server
# Restart if needed
python3 /home/pyxispi5/pyxis_dashboard/ws_server.py
```

---

## Architecture

```
Jetson (YOLO H264 RTP :5700)
        └─► cam1_relay.py (:8080) ──┐
                                    ├─► React Dashboard (:3000)
Arducam B0589 (/dev/video0)        │
        └─► cam2_stream.py (:8081) ─┘

ROS2 /proximity/distance
UDP detections (:5106)
        └─► ws_server.py (:8765) ──► React Dashboard (WebSocket)
```

**Stack:** React 18 + MUI v5 (production build) · GStreamer · Python 3 · ROS2 Jazzy

## Team Pyxis
