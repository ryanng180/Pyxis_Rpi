#!/bin/bash
# ============================================================
# Maritime Dashboard - Clean Startup Script
# Serves the PRODUCTION build (no webpack, near-zero CPU)
# ============================================================

DASHBOARD_DIR="/home/pyxispi5/pyxis_dashboard"
LOG_DIR="$DASHBOARD_DIR/logs"
BUILD_DIR="$DASHBOARD_DIR/build"

mkdir -p "$LOG_DIR"

echo "========================================"
echo " Maritime Dashboard Startup"
echo "========================================"

# ---- 1. Kill old processes cleanly ----
echo "[1/5] Stopping old processes..."
pkill -f "ws_server.py"         2>/dev/null
pkill -f "cam1_relay.py"        2>/dev/null
pkill -f "cam2_stream.py"       2>/dev/null
pkill -f "http.server"          2>/dev/null
pkill -f "npx serve"            2>/dev/null
pkill -f "react-scripts"        2>/dev/null
# Kill GStreamer pipelines spawned by cam scripts
pkill -f "gst-launch-1.0" 2>/dev/null
sleep 2

# ---- 2. Verify build exists ----
echo "[2/5] Checking production build..."
if [ ! -f "$BUILD_DIR/index.html" ]; then
    echo "[ERROR] Production build not found at $BUILD_DIR/index.html"
    echo "        Run: cd $DASHBOARD_DIR && npm run build"
    exit 1
fi
echo "       Build OK."

# ---- 3. Start camera streams ----
echo "[3/5] Starting camera streams..."

# Cam1: H264 RTP relay from Jetson (port 8080)
nohup python3 "$DASHBOARD_DIR/cam1_relay.py" \
    > "$LOG_DIR/cam1.log" 2>&1 &
CAM1_PID=$!
echo "       cam1_relay.py started (PID $CAM1_PID) → port 8080"

sleep 1

# Cam2: Local Arducam B0589 (port 8081)
nohup python3 "$DASHBOARD_DIR/cam2_stream.py" \
    > "$LOG_DIR/cam2.log" 2>&1 &
CAM2_PID=$!
echo "       cam2_stream.py started (PID $CAM2_PID) → port 8081"

sleep 1

# ---- 4. Start WebSocket bridge ----
echo "[4/5] Starting WebSocket bridge..."
nohup python3 "$DASHBOARD_DIR/ws_server.py" \
    > "$LOG_DIR/ws.log" 2>&1 &
WS_PID=$!
echo "       ws_server.py started (PID $WS_PID) → port 8765"

sleep 1

# ---- 5. Serve production build (lightweight, ~0% CPU) ----
echo "[5/5] Serving production build on port 3000..."
nohup python3 -m http.server 3000 \
    --directory "$BUILD_DIR" \
    > "$LOG_DIR/http.log" 2>&1 &
HTTP_PID=$!
echo "       Static server started (PID $HTTP_PID) → port 3000"

sleep 2

# ---- Status check ----
echo ""
echo "========================================"
echo " Service Status"
echo "========================================"
for port in 3000 8080 8081 8765; do
    if ss -tlnp | grep -q ":$port "; then
        echo "  ✓ Port $port OPEN"
    else
        echo "  ✗ Port $port NOT open"
    fi
done

echo ""
echo " Dashboard: http://localhost:3000"
echo " Cam1 test: curl -s -I http://localhost:8080/stream | head -3"
echo " Cam2 test: curl -s -I http://localhost:8081/stream | head -3"
echo ""
echo " Logs in: $LOG_DIR/"
echo "========================================"
