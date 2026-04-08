#!/bin/bash
# ============================================================
#  Pyxis Maritime Pilot Transfer System — Unified Startup
#
#  Launches all services on RPi5 in staggered groups to avoid
#  CPU spikes on edge hardware. Checks Jetson health but never
#  interferes with it.
#
#  Usage:
#    ./pyxis_startup.sh          # start everything
#    ./pyxis_startup.sh --no-rviz  # skip RViz (saves ~18% CPU)
#
#  Ctrl+C → graceful shutdown of ALL services in reverse order.
# ============================================================

set -eo pipefail

# ── Configuration ──
JETSON_IP="10.42.0.1"
JETSON_USER="pixysjetson2"
JETSON_RTP_PORT=5700
JETSON_DET_PORT=5106
JETSON_CAM2_RTP_PORT=5701
# Stable symlinks created by /etc/udev/rules.d/99-pyxis-cameras.rules on the
# Jetson. The rules pin USB port 2.2 -> /dev/pyxis_cam1 and USB port 1.3 ->
# /dev/pyxis_cam2. Port-keyed (not identity-keyed) because both cameras are
# identical Arducam B0589 4K HDR units with no unique serials — see memory
# file project_arducam_identity_limitation.md. The preflight check below
# refuses to start if either symlink is missing.
JETSON_CAM1_DEVICE="/dev/pyxis_cam1"
JETSON_CAM2_DEVICE="/dev/pyxis_cam2"

ROS2_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"
DASHBOARD_DIR="$HOME/pyxis_dashboard"
LIBS_DIR="$HOME/pyxis_dashboard_libs"
LOG_DIR="$DASHBOARD_DIR/logs"

CPU_MAX_LOAD=5.0                  # pause between groups if load exceeds this (4-core RPi5)
SETTLE_TIME=2                     # seconds between launch groups
MAX_RETRIES=2                     # retry a failed service once
HEALTH_TIMEOUT=10                 # seconds to wait for a service to become healthy

SKIP_RVIZ=false
[[ "${1:-}" == "--no-rviz" ]] && SKIP_RVIZ=true

# ── Colours ──
RED='\033[0;31m'
GRN='\033[0;32m'
YLW='\033[1;33m'
CYN='\033[0;36m'
RST='\033[0m'

# ── Track all PIDs for cleanup ──
declare -a PIDS=()
declare -a PID_LABELS=()
SENSOR_LAUNCH_PID=""
GIMBAL_LAUNCH_PID=""
GIMBAL_TRACKER_PID=""
RVIZ_PID=""

# ============================================================
#  Utilities
# ============================================================

log()   { echo -e "${CYN}[PYXIS]${RST} $*"; }
ok()    { echo -e "  ${GRN}✓${RST} $*"; }
warn()  { echo -e "  ${YLW}!${RST} $*"; }
fail()  { echo -e "  ${RED}✗${RST} $*"; }

get_load() {
    awk '{print $1}' /proc/loadavg
}

wait_for_cpu() {
    local load
    load=$(get_load)
    local attempts=0
    while (( $(echo "$load > $CPU_MAX_LOAD" | bc -l) )); do
        if [[ $attempts -eq 0 ]]; then
            warn "CPU load is ${load} (threshold ${CPU_MAX_LOAD}) — waiting for it to settle..."
        fi
        sleep 3
        load=$(get_load)
        attempts=$((attempts + 1))
        if [[ $attempts -ge 20 ]]; then
            warn "CPU still high (${load}) after 60s — proceeding anyway"
            break
        fi
    done
}

wait_for_port() {
    local port=$1 label=$2 timeout=${3:-$HEALTH_TIMEOUT}
    local elapsed=0
    while ! ss -tlnp 2>/dev/null | grep -q ":${port} "; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [[ $elapsed -ge $timeout ]]; then
            return 1
        fi
    done
    return 0
}

wait_for_ros_node() {
    local node_name=$1 timeout=${2:-$HEALTH_TIMEOUT}
    local elapsed=0
    while ! ros2 node list 2>/dev/null | grep -q "$node_name"; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [[ $elapsed -ge $timeout ]]; then
            return 1
        fi
    done
    return 0
}

check_pid_alive() {
    kill -0 "$1" 2>/dev/null
}

register_pid() {
    PIDS+=("$1")
    PID_LABELS+=("$2")
}

# ============================================================
#  Cleanup — reverse order, thorough
# ============================================================

cleanup() {
    echo ""
    log "Shutting down Pyxis system..."

    # 1. RViz
    if [[ -n "$RVIZ_PID" ]] && check_pid_alive "$RVIZ_PID"; then
        log "Stopping RViz..."
        kill "$RVIZ_PID" 2>/dev/null || true
    fi

    # 2. Gimbal tracker
    if [[ -n "$GIMBAL_TRACKER_PID" ]] && check_pid_alive "$GIMBAL_TRACKER_PID"; then
        log "Stopping gimbal tracker..."
        kill "$GIMBAL_TRACKER_PID" 2>/dev/null || true
    fi

    # 3. Gimbal launch
    if [[ -n "$GIMBAL_LAUNCH_PID" ]] && check_pid_alive "$GIMBAL_LAUNCH_PID"; then
        log "Stopping gimbal driver..."
        kill "$GIMBAL_LAUNCH_PID" 2>/dev/null || true
    fi

    # 4. Sensor launch
    if [[ -n "$SENSOR_LAUNCH_PID" ]] && check_pid_alive "$SENSOR_LAUNCH_PID"; then
        log "Stopping sensor stack..."
        kill "$SENSOR_LAUNCH_PID" 2>/dev/null || true
    fi

    # 5. Dashboard processes
    log "Stopping dashboard services..."
    for i in "${!PIDS[@]}"; do
        if check_pid_alive "${PIDS[$i]}"; then
            kill "${PIDS[$i]}" 2>/dev/null || true
        fi
    done

    # 6. Orphan GStreamer pipelines
    pkill -f "gst-launch-1.0" 2>/dev/null || true

    # Wait for graceful exit
    sleep 2

    # 7. Force kill anything that survived
    local stragglers=0
    for pid in "$SENSOR_LAUNCH_PID" "$GIMBAL_LAUNCH_PID" "$GIMBAL_TRACKER_PID" "$RVIZ_PID" "${PIDS[@]}"; do
        if [[ -n "$pid" ]] && check_pid_alive "$pid"; then
            kill -9 "$pid" 2>/dev/null || true
            stragglers=$((stragglers + 1))
        fi
    done

    # Kill any remaining ROS2 child processes from our launches
    pkill -9 -f "sensors_bringup" 2>/dev/null || true
    pkill -9 -f "gimbal_tracker" 2>/dev/null || true
    pkill -9 -f "storm32" 2>/dev/null || true
    pkill -9 -f "sllidar_node" 2>/dev/null || true
    pkill -9 -f "imu_filter_madgwick" 2>/dev/null || true
    pkill -9 -f "gst-launch-1.0" 2>/dev/null || true

    if [[ $stragglers -gt 0 ]]; then
        warn "Force-killed $stragglers stubborn processes"
    fi

    # 7. Stop Jetson inference (only if WE started it this session)
    if $JETSON_AVAILABLE && ! $JETSON_INFERENCE_WAS_RUNNING; then
        log "Stopping Jetson inference (started by this session)..."
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f 'jetson_inference_sender_stream_final'" 2>/dev/null || true
    else
        echo -e "  ${GRN}Jetson inference left running (was already active or unreachable)${RST}"
    fi

    # 7b. Stop Jetson cam2_sender (only if WE started it this session).
    # Match by script name AND the specific RTP port to avoid touching cam1's
    # pipeline, which also ends in a udpsink.
    if $JETSON_AVAILABLE && ! $JETSON_CAM2_WAS_RUNNING; then
        log "Stopping Jetson cam2_sender (started by this session)..."
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f 'cam2_sender.py' 2>/dev/null; \
             pkill -9 -f 'port=${JETSON_CAM2_RTP_PORT}' 2>/dev/null; true" 2>/dev/null || true
    else
        echo -e "  ${GRN}Jetson cam2_sender left running (was already active or unreachable)${RST}"
    fi

    log "All Pyxis services stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# ============================================================
#  Pre-flight checks
# ============================================================

echo ""
echo -e "${CYN}========================================${RST}"
echo -e "${CYN} Pyxis Maritime System — Startup${RST}"
echo -e "${CYN}========================================${RST}"
echo ""

# Kill any leftover Pyxis processes from previous runs
log "Cleaning up stale processes..."
pkill -f "sensors_bringup" 2>/dev/null || true
pkill -f "gimbal_tracker_pitch_yaw" 2>/dev/null || true
pkill -f "storm32" 2>/dev/null || true
pkill -f "sllidar_node" 2>/dev/null || true
pkill -f "imu_filter_madgwick" 2>/dev/null || true
pkill -f "ws_server.py" 2>/dev/null || true
pkill -f "cam1_relay" 2>/dev/null || true
pkill -f "cam2_stream" 2>/dev/null || true   # legacy (pre-Jetson-migration local USB reader)
pkill -f "cam2_relay"  2>/dev/null || true   # current (RTP receiver from Jetson)
pkill -f "http.server.*3000" 2>/dev/null || true
pkill -f "gst-launch-1.0" 2>/dev/null || true
sleep 2
ok "Old processes cleared"

# Source ROS2
log "Sourcing ROS2 environment..."
source "$ROS2_SETUP"
if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
    ok "ROS2 Jazzy + workspace sourced"
else
    fail "Workspace not built: $WS_SETUP not found"
    echo "  Run: cd ~/ros2_ws && colcon build"
    exit 1
fi
export PYTHONPATH="${PYTHONPATH:-}:$LIBS_DIR"

# Check build exists
if [[ ! -f "$DASHBOARD_DIR/build/index.html" ]]; then
    fail "Dashboard build not found at $DASHBOARD_DIR/build/"
    echo "  Run: cd ~/pyxis-dashboard-repo && npm run build && cp -r build ~/pyxis_dashboard/"
    exit 1
fi
ok "Dashboard build found"

mkdir -p "$LOG_DIR"

# ============================================================
#  Group 0: Jetson — check + start inference if needed
# ============================================================

JETSON_INFERENCE_SCRIPT="/home/$JETSON_USER/pyxis/main_script/jetson_inference_sender_stream_final.py"
JETSON_INFERENCE_DIR="/home/$JETSON_USER/pyxis/main_script"
JETSON_CAM2_SCRIPT="/home/$JETSON_USER/pyxis/main_script/cam2_sender.py"
JETSON_AVAILABLE=false
JETSON_INFERENCE_WAS_RUNNING=false  # was it already running before we started?
JETSON_CAM2_WAS_RUNNING=false       # was cam2_sender already running before we started?

log "Checking Jetson status..."

if ping -c 1 -W 2 "$JETSON_IP" &>/dev/null; then
    ok "Jetson reachable at $JETSON_IP"
    JETSON_AVAILABLE=true

    # ── Layer 2 camera preflight ──
    # Both cameras are identical Arducam B0589 units with no unique serials —
    # /dev/pyxis_cam1 and /dev/pyxis_cam2 are port-keyed udev symlinks on the
    # Jetson (99-pyxis-cameras.rules). If either is missing the corresponding
    # camera is unplugged (or plugged into the wrong USB port), and we MUST
    # fail loud rather than silently let inference bind the wrong device.
    # This exact failure bit us on 2026-04-05 — cam1 was unplugged and
    # inference silently ran YOLO on cam2's feed.
    log "Checking Jetson camera symlinks (Layer 2 preflight)..."
    CAM_PREFLIGHT_MISSING=""
    if ! ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "test -e '$JETSON_CAM1_DEVICE'" 2>/dev/null; then
        CAM_PREFLIGHT_MISSING="$CAM_PREFLIGHT_MISSING $JETSON_CAM1_DEVICE"
    fi
    if ! ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "test -e '$JETSON_CAM2_DEVICE'" 2>/dev/null; then
        CAM_PREFLIGHT_MISSING="$CAM_PREFLIGHT_MISSING $JETSON_CAM2_DEVICE"
    fi
    if [[ -n "$CAM_PREFLIGHT_MISSING" ]]; then
        fail "Missing camera symlink(s) on Jetson:${CAM_PREFLIGHT_MISSING}"
        echo -e "    ${YLW}Both cameras must be plugged into their labelled USB ports${RST}"
        echo -e "    ${YLW}  Port 2.2 = cam1 (YOLO inference)${RST}"
        echo -e "    ${YLW}  Port 1.3 = cam2 (observation feed)${RST}"
        echo -e "    ${YLW}Check: ssh $JETSON_USER@$JETSON_IP 'ls -la /dev/pyxis_cam*'${RST}"
        echo -e "    ${YLW}Refusing to start — cameras are NOT identifiable by serial${RST}"
        echo -e "    ${YLW}(both are Arducam B0589, identical identity fields)${RST}"
        exit 2
    fi
    ok "Jetson camera symlinks present (/dev/pyxis_cam1, /dev/pyxis_cam2)"

    # ── Recording retention (Stage 10d) ──
    # Deletes day-bucket directories older than 5 days from ~/pyxis_recordings
    # on the Jetson SSD. Runs on every startup rather than via cron because:
    #   - Maritime sessions are operator-driven (not 24/7), so a 3 AM cron
    #     would miss runs whenever the Jetson is powered off at that time
    #   - Running here makes retention visible in the startup log
    #   - Keeps behavior in one place (this script is the single orchestrator)
    # Safety: -maxdepth 1 + -type d + -name "20*-*-*" means the find can ONLY
    # match YYYY-MM-DD day-bucket directories — never the recordings root, the
    # retention log, a future detections/ subdir, or any stray file. -mtime +5
    # requires the directory to be OLDER than 5 days by mtime.
    log "Running recording retention (>5 days old)..."
    RETAIN_OUT=$(ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "mkdir -p ~/pyxis_recordings && \
         find ~/pyxis_recordings -maxdepth 1 -type d -name '20*-*-*' -mtime +5 -print -exec rm -rf {} + 2>/dev/null; \
         df -BG ~/pyxis_recordings 2>/dev/null | awk 'NR==2 {print \$4}'" 2>/dev/null || true)
    RETAIN_FREE=$(echo "$RETAIN_OUT" | tail -1)
    RETAIN_DELETED=$(echo "$RETAIN_OUT" | sed '$d' | grep -c '.' || true)
    if [[ "${RETAIN_DELETED:-0}" -gt 0 ]]; then
        ok "Retention: deleted $RETAIN_DELETED old day(s), free: ${RETAIN_FREE:-?}"
    else
        ok "Retention: nothing to delete, free: ${RETAIN_FREE:-?}"
    fi

    # Check if inference is already running AND healthy (cam1 by-path open).
    # When PYXIS_RECORD=1 we force a restart so the new env takes effect —
    # same rationale as cam2_sender: an already-running process from a
    # previous non-recording launch would silently ignore the new request
    # and the operator would wonder why no cam1 MKV files are appearing.
    if [[ "${PYXIS_RECORD:-0}" == "1" ]]; then
        INFERENCE_ALREADY_HEALTHY=false
    elif ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "pgrep -f 'jetson_inference_sender_stream_final' >/dev/null 2>&1" && \
       ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "fuser '$JETSON_CAM1_DEVICE' 2>/dev/null | grep -q ." ; then
        INFERENCE_ALREADY_HEALTHY=true
    else
        INFERENCE_ALREADY_HEALTHY=false
    fi
    if $INFERENCE_ALREADY_HEALTHY; then
        ok "Jetson inference already running and healthy"
        JETSON_INFERENCE_WAS_RUNNING=true
    else
        log "Starting Jetson inference..."

        # Read persisted user preference for default YOLO model.
        # Set via dashboard Diagnostics → ★ button. Written by ws_server.py.
        # If missing/invalid, Jetson inference script picks first non-deprecated model.
        PREFS_FILE="$HOME/pyxis_dashboard/config/user_prefs.json"
        DEFAULT_MODEL=""
        if [ -f "$PREFS_FILE" ]; then
            DEFAULT_MODEL=$(python3 -c "
import json
try:
    with open('$PREFS_FILE') as f:
        print(json.load(f).get('default_model', '') or '')
except Exception:
    pass
" 2>/dev/null)
        fi
        if [ -n "$DEFAULT_MODEL" ]; then
            log "  Boot default model from prefs: $DEFAULT_MODEL"
        else
            log "  No boot default set — Jetson will use its fallback"
        fi

        # Kill any stale inference processes first
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f 'jetson_inference_sender_stream_final'" 2>/dev/null || true
        sleep 2
        # Start inference in background via SSH (inside venv).
        # PYXIS_DEFAULT_MODEL env var is read by the Jetson inference script
        # at startup; the running start_inference.sh wrapper just passes it
        # through to the python process (env vars inherit naturally).
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "PYXIS_DEFAULT_MODEL='$DEFAULT_MODEL' \
             PYXIS_RECORD='${PYXIS_RECORD:-0}' \
             PYXIS_RECORD_DIR='${PYXIS_RECORD_DIR:-~/pyxis_recordings}' \
             PYXIS_RECORD_MIN_GB='${PYXIS_RECORD_MIN_GB:-20}' \
             PYXIS_SEGMENT_SEC='${PYXIS_SEGMENT_SEC:-300}' \
             bash ~/pyxis/start_inference.sh" \
            && ok "Jetson inference start command sent" \
            || warn "Failed to start Jetson inference via SSH"

        # Wait for it to start producing data
        sleep 5
        if ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pgrep -f 'jetson_inference_sender_stream_final' >/dev/null 2>&1"; then
            ok "Jetson inference confirmed running"
        else
            fail "Jetson inference did not start"
            # Show last few lines of log to diagnose (e.g. missing module)
            warn "  Last log lines from Jetson:"
            ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
                "tail -5 ~/pyxis/inference.log 2>/dev/null" | while read -r line; do
                echo -e "    ${RED}${line}${RST}"
            done
        fi
    fi
else
    warn "Jetson not reachable at $JETSON_IP — cam1 will show no feed"
    warn "Start Jetson services manually before connecting"
fi

# ── cam2_sender on Jetson (RTP JPEG → Pi UDP:5701) ──
# Mirrors the cam1 pattern: capture happens on the Jetson, Pi just relays.
if $JETSON_AVAILABLE; then
    log "Checking Jetson cam2_sender status..."
    # If PYXIS_RECORD=1, always force a restart — otherwise an already-running
    # cam2_sender launched without the recording branch would silently ignore
    # the newly-requested recording and the operator would wonder why no MKV
    # files are appearing. Forcing the restart makes the behavior predictable.
    if [[ "${PYXIS_RECORD:-0}" == "1" ]]; then
        log "  PYXIS_RECORD=1 — forcing cam2_sender restart to apply recording"
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f 'cam2_sender.py' 2>/dev/null; \
             pkill -9 -f 'port=${JETSON_CAM2_RTP_PORT}' 2>/dev/null; true" 2>/dev/null || true
        sleep 1
        CAM2_ALREADY_HEALTHY=false
    elif ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "pgrep -f 'cam2_sender.py' >/dev/null 2>&1" && \
       ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
        "fuser '$JETSON_CAM2_DEVICE' 2>/dev/null | grep -q ." ; then
        CAM2_ALREADY_HEALTHY=true
    else
        CAM2_ALREADY_HEALTHY=false
    fi
    if $CAM2_ALREADY_HEALTHY; then
        ok "Jetson cam2_sender already running and healthy"
        JETSON_CAM2_WAS_RUNNING=true
    else
        log "Starting Jetson cam2_sender..."
        # Kill any stale cam2_sender (and its gst-launch child on the same port) first.
        # We match by script name AND by port to avoid collateral damage to cam1's pipeline.
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pkill -9 -f 'cam2_sender.py' 2>/dev/null; \
             pkill -9 -f 'port=${JETSON_CAM2_RTP_PORT}' 2>/dev/null; true" 2>/dev/null || true
        sleep 1

        # Recording env vars are forwarded from THIS shell if set. Enable with:
        #   PYXIS_RECORD=1 ./pyxis_startup.sh
        # Optional overrides (rarely needed — defaults match Stage 10 design):
        #   PYXIS_RECORD_DIR, PYXIS_RECORD_MIN_GB, PYXIS_SEGMENT_SEC
        # cam2_sender.py's prepare_recording_path() handles the day-bucket,
        # disk preflight, and graceful "recording disabled" fallback if the
        # SSD is full or the directory is unwritable.
        if [[ "${PYXIS_RECORD:-0}" == "1" ]]; then
            log "  Recording ENABLED (PYXIS_RECORD=1) — cam2 → Jetson SSD"
        fi
        # Launch via nohup+setsid so it survives the ssh session.
        ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "CAM2_DEVICE='$JETSON_CAM2_DEVICE' \
             CAM2_RPI_HOST='10.42.0.2' \
             CAM2_RPI_PORT='$JETSON_CAM2_RTP_PORT' \
             CAM2_WIDTH=640 CAM2_HEIGHT=480 CAM2_FPS=10 CAM2_FORMAT=mjpeg \
             PYXIS_RECORD='${PYXIS_RECORD:-0}' \
             PYXIS_RECORD_DIR='${PYXIS_RECORD_DIR:-~/pyxis_recordings}' \
             PYXIS_RECORD_MIN_GB='${PYXIS_RECORD_MIN_GB:-20}' \
             PYXIS_SEGMENT_SEC='${PYXIS_SEGMENT_SEC:-300}' \
             nohup setsid python3 '$JETSON_CAM2_SCRIPT' \
               > /tmp/cam2_sender.log 2>&1 < /dev/null &" \
            && ok "Jetson cam2_sender start command sent" \
            || warn "Failed to start Jetson cam2_sender via SSH"

        sleep 3
        if ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
            "pgrep -f 'cam2_sender.py' >/dev/null 2>&1"; then
            ok "Jetson cam2_sender confirmed running"
        else
            fail "Jetson cam2_sender did not start"
            warn "  Last log lines from Jetson /tmp/cam2_sender.log:"
            ssh -o ConnectTimeout=5 "$JETSON_USER@$JETSON_IP" \
                "tail -10 /tmp/cam2_sender.log 2>/dev/null" | while read -r line; do
                echo -e "    ${RED}${line}${RST}"
            done
        fi
    fi
fi

wait_for_cpu

# ============================================================
#  Group 1: Sensor Stack (9+ nodes)
# ============================================================

log "Starting sensor stack..."

ros2 launch sensors_bringup sensors_launch.py \
    > "$LOG_DIR/sensors.log" 2>&1 &
SENSOR_LAUNCH_PID=$!

# Wait for key nodes to appear
sleep 3
SENSOR_OK=true

if wait_for_ros_node "proximity_node" 15; then
    ok "proximity_node running"
else
    fail "proximity_node did not start (check $LOG_DIR/sensors.log)"
    SENSOR_OK=false
fi

if wait_for_ros_node "phase_manager_node" 5; then
    ok "phase_manager_node running"
else
    warn "phase_manager_node not found — phase features disabled"
fi

if $SENSOR_OK; then
    ok "Sensor stack launched (PID $SENSOR_LAUNCH_PID)"
else
    warn "Sensor stack partially failed — continuing anyway"
fi

log "Waiting ${SETTLE_TIME}s for CPU to settle..."
sleep "$SETTLE_TIME"
wait_for_cpu

# ============================================================
#  Group 2: Gimbal Stack
# ============================================================

log "Starting gimbal stack..."

# Gimbal driver
ros2 launch storm32_gimbal gimbal_launch.py \
    > "$LOG_DIR/gimbal_driver.log" 2>&1 &
GIMBAL_LAUNCH_PID=$!
sleep 3

if wait_for_ros_node "controller" 10; then
    ok "Gimbal driver running (PID $GIMBAL_LAUNCH_PID)"
else
    warn "Gimbal driver did not start — check USB connection to STorM32"
    warn "  Log: $LOG_DIR/gimbal_driver.log"
fi

wait_for_cpu

# Gimbal tracker
ros2 run gimbal_tracker gimbal_tracker_pitch_yaw \
    > "$LOG_DIR/gimbal_tracker.log" 2>&1 &
GIMBAL_TRACKER_PID=$!
sleep 2

if wait_for_ros_node "gimbal_tracker" 8; then
    ok "Gimbal tracker running (PID $GIMBAL_TRACKER_PID)"
else
    warn "Gimbal tracker did not start"
    warn "  Log: $LOG_DIR/gimbal_tracker.log"
fi

log "Waiting ${SETTLE_TIME}s for CPU to settle..."
sleep "$SETTLE_TIME"
wait_for_cpu

# ============================================================
#  Group 3: Dashboard Stack (cameras + WS bridge + HTTP)
# ============================================================

log "Starting dashboard services..."

# Camera 1 relay (Jetson H264 → MJPEG)
python3 "$DASHBOARD_DIR/cam1_relay.py" \
    > "$LOG_DIR/cam1.log" 2>&1 &
register_pid $! "cam1_relay"
sleep 1

# Camera 2 relay (Jetson JPEG RTP → MJPEG) — was cam2_stream.py before
# the cam2→Jetson migration. The dashboard still hits :8081 unchanged.
python3 "$DASHBOARD_DIR/cam2_relay.py" \
    > "$LOG_DIR/cam2.log" 2>&1 &
register_pid $! "cam2_relay"
sleep 1

wait_for_cpu

# WebSocket bridge
python3 "$DASHBOARD_DIR/ws_server.py" \
    > "$LOG_DIR/ws.log" 2>&1 &
register_pid $! "ws_server"
sleep 1

# Static HTTP server for React build
python3 -m http.server 3000 --directory "$DASHBOARD_DIR/build" \
    > "$LOG_DIR/http.log" 2>&1 &
register_pid $! "http_server"
sleep 2

# Verify ports
DASH_OK=true
for entry in "8080:cam1_relay" "8081:cam2_relay" "8765:ws_server" "3000:http_server"; do
    port="${entry%%:*}"
    name="${entry##*:}"
    if wait_for_port "$port" "$name" 5; then
        ok "$name → port $port"
    else
        fail "$name not listening on port $port (check $LOG_DIR/)"
        DASH_OK=false
    fi
done

if $DASH_OK; then
    ok "Dashboard stack fully operational"
fi

# ============================================================
#  Group 4: RViz (optional, heavy)
# ============================================================

if ! $SKIP_RVIZ; then
    log "Starting RViz2 (use --no-rviz to skip)..."
    wait_for_cpu
    ros2 run rviz2 rviz2 \
        > "$LOG_DIR/rviz.log" 2>&1 &
    RVIZ_PID=$!
    sleep 2
    if check_pid_alive "$RVIZ_PID"; then
        ok "RViz2 running (PID $RVIZ_PID)"
    else
        warn "RViz2 failed to start"
    fi
else
    ok "RViz2 skipped (--no-rviz)"
fi

# ============================================================
#  Final Status Report
# ============================================================

sleep 2
echo ""
echo -e "${CYN}========================================${RST}"
echo -e "${CYN} System Status${RST}"
echo -e "${CYN}========================================${RST}"

# ROS2 nodes
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
echo -e "  ROS2 nodes:  ${GRN}${NODE_COUNT}${RST}"

# Ports
for port in 3000 8080 8081 8765; do
    if ss -tlnp 2>/dev/null | grep -q ":${port} "; then
        echo -e "  Port $port:    ${GRN}OPEN${RST}"
    else
        echo -e "  Port $port:    ${RED}CLOSED${RST}"
    fi
done

# Jetson status
if $JETSON_AVAILABLE; then
    if ssh -o ConnectTimeout=3 "$JETSON_USER@$JETSON_IP" \
        "pgrep -f 'jetson_inference_sender_stream_final' >/dev/null 2>&1" 2>/dev/null; then
        echo -e "  Jetson:      ${GRN}INFERENCE RUNNING${RST}"
    else
        echo -e "  Jetson:      ${RED}INFERENCE NOT RUNNING${RST}"
    fi
else
    echo -e "  Jetson:      ${RED}UNREACHABLE${RST}"
fi

# System resources
LOAD=$(get_load)
MEM_USED=$(free -m | awk '/Mem:/ {printf "%.0f%%", $3/$2*100}')
echo -e "  CPU load:    ${LOAD}"
echo -e "  RAM used:    ${MEM_USED}"

echo ""
echo -e "  Dashboard:   ${GRN}http://localhost:3000${RST}"
echo -e "  WebSocket:   ws://localhost:8765"
echo -e "  Cam1 feed:   http://localhost:8080/stream"
echo -e "  Cam2 feed:   http://localhost:8081/stream"
echo -e "  Logs:        $LOG_DIR/"
echo ""
echo -e "  ${YLW}Press Ctrl+C to shut down all services${RST}"
echo -e "${CYN}========================================${RST}"
echo ""

# ── Keep alive — monitor health every 30s ──
while true; do
    sleep 30

    # Check for crashed services
    CRASHED=""

    if ! check_pid_alive "$SENSOR_LAUNCH_PID"; then
        CRASHED="$CRASHED sensor_stack"
    fi

    for i in "${!PIDS[@]}"; do
        if ! check_pid_alive "${PIDS[$i]}"; then
            CRASHED="$CRASHED ${PID_LABELS[$i]}"
        fi
    done

    if [[ -n "$CRASHED" ]]; then
        echo -e "${RED}[PYXIS] CRASHED:${CRASHED}${RST}"
        echo -e "${YLW}[PYXIS] Check logs at $LOG_DIR/ — press Ctrl+C to shut down${RST}"
    fi
done
