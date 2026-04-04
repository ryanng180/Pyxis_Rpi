#!/bin/bash
# ============================================================
# Pyxis Maritime Dashboard - Startup Script
# Starts all dashboard backend processes alongside the ROS2 stack.
# Run AFTER launching ROS2 (ros2 launch sensors_bringup sensors_launch.py)
# ============================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$SCRIPT_DIR/logs"
BUILD_DIR="$SCRIPT_DIR/build"
LIBS="$HOME/pyxis_dashboard_libs"

mkdir -p "$LOG_DIR"

echo "========================================"
echo " Pyxis Maritime Dashboard Startup"
echo "========================================"

# ---- 0. Source ROS2 so ws_server.py can subscribe to topics ----
echo "[0/5] Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export PYTHONPATH="$PYTHONPATH:$LIBS"

# ---- 1. Kill old processes cleanly ----
echo "[1/5] Stopping old processes..."
pkill -f "ws_server.py"         2>/dev/null || true
pkill -f "cam1_relay.py"        2>/dev/null || true
pkill -f "cam2_stream.py"       2>/dev/null || true
pkill -f "http.server"          2>/dev/null || true
pkill -f "gst-launch-1.0"      2>/dev/null || true
sleep 2

# ---- 2. Verify build exists ----
echo "[2/5] Checking production build..."
if [ ! -f "$BUILD_DIR/index.html" ]; then
    echo "[ERROR] Production build not found at $BUILD_DIR/index.html"
    echo "        Run: cd $(dirname "$0") && npm run build"
    exit 1
fi
echo "       Build OK."

# ---- 3. Start camera streams ----
echo "[3/5] Starting camera streams..."

nohup python3 "$SCRIPT_DIR/cam1_relay.py" \
    > "$LOG_DIR/cam1.log" 2>&1 &
PID_CAM1=$!
echo "       cam1_relay.py started (PID $PID_CAM1) → port 8080"

sleep 1

nohup python3 "$SCRIPT_DIR/cam2_stream.py" \
    > "$LOG_DIR/cam2.log" 2>&1 &
PID_CAM2=$!
echo "       cam2_stream.py started (PID $PID_CAM2) → port 8081"

sleep 1

# ---- 4. Start WebSocket bridge ----
echo "[4/5] Starting WebSocket bridge..."
nohup python3 "$SCRIPT_DIR/ws_server.py" \
    > "$LOG_DIR/ws.log" 2>&1 &
PID_WS=$!
echo "       ws_server.py started (PID $PID_WS) → port 8765"

sleep 1

# ---- 5. Serve production build ----
echo "[5/5] Serving production build on port 3000..."
nohup python3 -m http.server 3000 \
    --directory "$BUILD_DIR" \
    > "$LOG_DIR/http.log" 2>&1 &
PID_HTTP=$!
echo "       Static server started (PID $PID_HTTP) → port 3000"

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
echo " WS:        ws://localhost:8765"
echo " Cam1:      http://localhost:8080/stream"
echo " Cam2:      http://localhost:8081/stream"
echo " Logs:      $LOG_DIR/"
echo ""
echo " Press Ctrl+C to stop all dashboard processes."
echo "========================================"

cleanup() {
    echo ""
    echo "Stopping dashboard stack..."
    kill $PID_CAM1 $PID_CAM2 $PID_WS $PID_HTTP 2>/dev/null || true
    pkill -f "gst-launch-1.0" 2>/dev/null || true
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

wait
