#!/usr/bin/env python3
"""
ws_server.py — WebSocket bridge for Maritime Dashboard (port 8765)

Sources:
  - UDP port 5106 : JSON detection packets from Jetson (mirror of gimbal port)
  - ROS2 topics   :
      /proximity/distance   Float32           hull-to-cargo distance (m)
      /proximity/zone       String            TOO_CLOSE / OPTIMAL / TOO_FAR
      /proximity/status     String (JSON)     full proximity telemetry
      /gimbal/actual_angles Float32MultiArray  [yaw_deg, pitch_deg]
      /ladder/distance      Float32           boarding gate distance (m)
      /ladder/status        String (JSON)     full ladder telemetry
      /approach/heading     Float32           heading error (degrees)
      /approach/zone        String            approach zone
      /approach/profile     String (JSON)     hull gap profile
      /imu/data             Imu              boat orientation (pitch/roll)

Broadcasts to all WebSocket clients at BROADCAST_HZ.
"""

import asyncio
import json
import math
import socket
import threading
import time
import os

import websockets

# ---- Config ---------------------------------------------------------------
WS_PORT       = 8765
UDP_DET_PORT  = 5106      # Jetson sends detection mirror here
UDP_BUFFER    = 65535
BROADCAST_HZ  = 5         # updates per second to dashboard (keep RPi cool)

# Jetson control channel
JETSON_IP     = "10.42.0.1"
JETSON_CTRL_PORT = 5108   # UDP control port on Jetson for model switching

# Camera URLs served from THIS machine
RPI_HOST = os.environ.get("RPI_HOST", "localhost")
CAM1_URL = os.environ.get("CAM1_URL", f"http://{RPI_HOST}:8080/stream")
CAM2_URL = os.environ.get("CAM2_URL", f"http://{RPI_HOST}:8081/stream")

# ---- Shared state (thread-safe via lock) ----------------------------------
_state_lock = threading.Lock()
_state = {
    # Existing
    "distance": None,
    "detections": [],
    "source_resolution": {"w": 640, "h": 480},
    "last_detection_ts": 0.0,
    # Proximity
    "proximity_zone": None,
    "proximity_status": None,
    # Gimbal
    "gimbal_angles": None,          # [yaw_deg, pitch_deg]
    "target_locked": False,
    # Ladder
    "ladder_distance": None,
    "ladder_status": None,
    # Approach
    "approach_heading": None,
    "approach_zone": None,
    "approach_profile": None,
    # IMU / boat tilt
    "boat_pitch_deg": None,
    "boat_roll_deg": None,
    # System phase
    "system_phase": None,
    # Jetson model
    "current_model": None,
}

# ---- UDP Detection Receiver -----------------------------------------------
def udp_receiver():
    """Receives JSON detection packets from Jetson on UDP port 5106."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
    sock.bind(("0.0.0.0", UDP_DET_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening for detections on :{UDP_DET_PORT}")

    while True:
        try:
            # Drain buffer — keep only freshest packet
            data = None
            sock.settimeout(0)
            try:
                while True:
                    data, _ = sock.recvfrom(UDP_BUFFER)
            except (BlockingIOError, socket.error):
                pass
            finally:
                sock.settimeout(1.0)

            if data is None:
                data, _ = sock.recvfrom(UDP_BUFFER)

            payload = json.loads(data.decode("utf-8"))
            detections = payload.get("ladder_detections", [])
            res = payload.get("source_resolution", {"w": 640, "h": 480})
            model_name = payload.get("current_model")

            with _state_lock:
                _state["detections"] = detections
                _state["source_resolution"] = res
                _state["last_detection_ts"] = time.time()
                if model_name:
                    _state["current_model"] = model_name

        except socket.timeout:
            with _state_lock:
                if time.time() - _state["last_detection_ts"] > 1.0:
                    _state["detections"] = []
        except (json.JSONDecodeError, KeyError):
            pass
        except Exception as e:
            print(f"[UDP] Error: {e}")
            time.sleep(0.5)


# ---- Quaternion to Euler helper -------------------------------------------
def _quat_to_pitch_roll_deg(q):
    """Extract pitch and roll (degrees) from a quaternion (x, y, z, w)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    return math.degrees(pitch), math.degrees(roll)


# ---- ROS2 Subscriber (all topics) ----------------------------------------
def ros2_subscriber():
    """Subscribes to all ROS2 topics and updates shared state."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float32, String
        from std_msgs.msg import Float32MultiArray
        from sensor_msgs.msg import Imu

        class DashboardBridge(Node):
            def __init__(self):
                super().__init__("ws_bridge")

                # Proximity
                self.create_subscription(
                    Float32, "/proximity/distance", self._proximity_dist_cb, 10)
                self.create_subscription(
                    String, "/proximity/zone", self._proximity_zone_cb, 10)
                self.create_subscription(
                    String, "/proximity/status", self._proximity_status_cb, 10)

                # Gimbal
                self.create_subscription(
                    Float32MultiArray, "/gimbal/actual_angles",
                    self._gimbal_cb, 10)
                self.create_subscription(
                    Bool, "/gimbal/target_locked",
                    self._target_locked_cb, 10)

                # Ladder
                self.create_subscription(
                    Float32, "/ladder/distance", self._ladder_dist_cb, 10)
                self.create_subscription(
                    String, "/ladder/status", self._ladder_status_cb, 10)

                # Approach
                self.create_subscription(
                    Float32, "/approach/heading", self._approach_heading_cb, 10)
                self.create_subscription(
                    String, "/approach/zone", self._approach_zone_cb, 10)
                self.create_subscription(
                    String, "/approach/profile", self._approach_profile_cb, 10)

                # IMU (boat tilt)
                self.create_subscription(
                    Imu, "/imu/data", self._imu_cb, 10)

                # System phase
                self.create_subscription(
                    String, "/system/phase", self._phase_cb, 10)

            # -- Proximity --
            def _proximity_dist_cb(self, msg):
                with _state_lock:
                    _state["distance"] = float(msg.data)

            def _proximity_zone_cb(self, msg):
                with _state_lock:
                    _state["proximity_zone"] = msg.data

            def _proximity_status_cb(self, msg):
                with _state_lock:
                    try:
                        _state["proximity_status"] = json.loads(msg.data)
                    except json.JSONDecodeError:
                        pass

            # -- Gimbal --
            def _gimbal_cb(self, msg):
                with _state_lock:
                    if len(msg.data) >= 2:
                        _state["gimbal_angles"] = [
                            round(msg.data[0], 1),  # yaw
                            round(msg.data[1], 1),  # pitch
                        ]

            def _target_locked_cb(self, msg):
                with _state_lock:
                    _state["target_locked"] = msg.data

            # -- Ladder --
            def _ladder_dist_cb(self, msg):
                with _state_lock:
                    _state["ladder_distance"] = round(float(msg.data), 2)

            def _ladder_status_cb(self, msg):
                with _state_lock:
                    try:
                        _state["ladder_status"] = json.loads(msg.data)
                    except json.JSONDecodeError:
                        pass

            # -- Approach --
            def _approach_heading_cb(self, msg):
                with _state_lock:
                    _state["approach_heading"] = round(float(msg.data), 1)

            def _approach_zone_cb(self, msg):
                with _state_lock:
                    _state["approach_zone"] = msg.data

            def _approach_profile_cb(self, msg):
                with _state_lock:
                    try:
                        _state["approach_profile"] = json.loads(msg.data)
                    except json.JSONDecodeError:
                        pass

            # -- IMU --
            def _imu_cb(self, msg):
                pitch, roll = _quat_to_pitch_roll_deg(msg.orientation)
                with _state_lock:
                    _state["boat_pitch_deg"] = round(pitch, 1)
                    _state["boat_roll_deg"] = round(roll, 1)

            # -- Phase --
            def _phase_cb(self, msg):
                with _state_lock:
                    _state["system_phase"] = msg.data

        rclpy.init()
        node = DashboardBridge()
        print("[ROS2] Subscribed to all dashboard topics")
        rclpy.spin(node)
    except ImportError:
        print("[ROS2] rclpy not available — sensor data will show as null")
    except Exception as e:
        print(f"[ROS2] Error: {e} — sensor data will show as null")


# ---- WebSocket Server ------------------------------------------------------
_clients: set = set()
_clients_lock = asyncio.Lock()


def _send_jetson_control(cmd_dict):
    """Send a UDP control command to Jetson inference script."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(3.0)
        s.sendto(json.dumps(cmd_dict).encode(), (JETSON_IP, JETSON_CTRL_PORT))
        resp, _ = s.recvfrom(4096)
        s.close()
        return json.loads(resp.decode("utf-8"))
    except Exception as e:
        print(f"[CTRL] Jetson control error: {e}")
        return {"status": "error", "msg": str(e)}


async def handler(websocket):
    async with _clients_lock:
        _clients.add(websocket)
    try:
        async for message in websocket:
            try:
                msg = json.loads(message)
                if msg.get("type") == "switch_model":
                    model = msg.get("model", "")
                    print(f"[WS] Model switch request: {model}")
                    loop = asyncio.get_event_loop()
                    result = await loop.run_in_executor(
                        None, _send_jetson_control,
                        {"cmd": "switch_model", "model": model}
                    )
                    await websocket.send(json.dumps({
                        "type": "model_switch_result", **result
                    }))
                elif msg.get("type") == "get_models":
                    loop = asyncio.get_event_loop()
                    result = await loop.run_in_executor(
                        None, _send_jetson_control,
                        {"cmd": "get_models"}
                    )
                    await websocket.send(json.dumps({
                        "type": "model_list", **result
                    }))
            except (json.JSONDecodeError, KeyError):
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        async with _clients_lock:
            _clients.discard(websocket)


async def broadcaster():
    """Push state to all connected clients at BROADCAST_HZ."""
    interval = 1.0 / BROADCAST_HZ
    while True:
        await asyncio.sleep(interval)

        with _state_lock:
            snap = {
                "type": "frame_update",
                "timestamp": time.time(),

                # LiDAR / proximity
                "lidar": {
                    "distance": _state["distance"],
                    "zone": _state["proximity_zone"],
                    "status": _state["proximity_status"],
                },

                # CV detections from Jetson
                "cv_detections": list(_state["detections"]),
                "source_resolution": dict(_state["source_resolution"]),

                # Gimbal
                "gimbal": {
                    "yaw": _state["gimbal_angles"][0] if _state["gimbal_angles"] else None,
                    "pitch": _state["gimbal_angles"][1] if _state["gimbal_angles"] else None,
                    "target_locked": _state["target_locked"],
                },

                # Ladder (parallax-corrected)
                "ladder": {
                    "distance": _state["ladder_distance"],
                    "status": _state["ladder_status"],
                },

                # Approach / hull profile
                "approach": {
                    "heading": _state["approach_heading"],
                    "zone": _state["approach_zone"],
                    "profile": _state["approach_profile"],
                },

                # Boat tilt
                "boat": {
                    "pitch": _state["boat_pitch_deg"],
                    "roll": _state["boat_roll_deg"],
                },

                # System phase
                "phase": _state["system_phase"],

                # Jetson model
                "current_model": _state["current_model"],

                # Camera URLs
                "camera_urls": {
                    "cam1": CAM1_URL,
                    "cam2": CAM2_URL,
                },
            }

        msg = json.dumps(snap)

        async with _clients_lock:
            clients_snapshot = set(_clients)

        if clients_snapshot:
            await asyncio.gather(
                *[c.send(msg) for c in clients_snapshot],
                return_exceptions=True
            )


async def main():
    print(f"[WS]  WebSocket server on ws://0.0.0.0:{WS_PORT}")
    print(f"[WS]  Camera URLs → {CAM1_URL}  |  {CAM2_URL}")

    async with websockets.serve(handler, "0.0.0.0", WS_PORT):
        await broadcaster()   # runs forever


if __name__ == "__main__":
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=ros2_subscriber, daemon=True).start()
    asyncio.run(main())
