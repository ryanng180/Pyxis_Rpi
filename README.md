# Pyxis Dashboard

Maritime pilot transfer safety dashboard — a full-screen HMI for pilot boat captains.

## Features
- Dual camera feeds (Jetson YOLO inference + local Arducam)
- Real-time distance monitoring via LiDAR
- CV bounding box overlays
- Safety alert system (danger/caution zones)
- WebSocket bridge for ROS2 sensor data
- Optimised for Raspberry Pi 5 deployment

## Architecture
- **Frontend:** React 18 + MUI v5, production build served as static files
- **Camera Streams:** GStreamer pipelines with Python MJPEG/snapshot HTTP servers
- **Data Bridge:** Python WebSocket server aggregating ROS2 topics and UDP detections
- **Inference:** YOLO v11m on Jetson, streamed via H264 RTP over LAN

## Team Pyxis
