#!/usr/bin/env python3
"""
ws_server.py — WebSocket bridge for Maritime Dashboard (port 8765)

Sources:
  - UDP port 5106 : JSON detection packets from Jetson (mirror of gimbal port)
  - ROS2 topic    : /proximity/distance  (float32 in metres)

Sends to dashboard every frame_update:
  {
    "type": "frame_update",
    "timestamp": <float>,
    "lidar": { "distance": <float|null> },
    "cv_detections": [ {"label":..,"confidence":..,"bbox":[x,y,w,h]} ],
    "source_resolution": { "w": 640, "h": 480 },
    "camera_urls": {
      "cam1": "http://<rpi_ip>:8080/stream",
      "cam2": "http://<rpi_ip>:8081/stream"
    }
  }
"""

import asyncio
import json
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

# Camera URLs served from THIS machine — sent to dashboard so it always has
# the correct host (works whether browser is on RPi or remote).
# Override via env vars if needed.
RPI_HOST = os.environ.get("RPI_HOST", "localhost")
CAM1_URL = os.environ.get("CAM1_URL", f"http://{RPI_HOST}:8080/stream")
CAM2_URL = os.environ.get("CAM2_URL", f"http://{RPI_HOST}:8081/stream")

# ---- Shared state (thread-safe via lock) ----------------------------------
_state_lock = threading.Lock()
_state = {
    "distance": None,
    "detections": [],
    "source_resolution": {"w": 640, "h": 480},
    "last_detection_ts": 0.0,
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
                # No fresh packet yet — blocking wait
                data, _ = sock.recvfrom(UDP_BUFFER)

            payload = json.loads(data.decode("utf-8"))
            detections = payload.get("ladder_detections", [])
            res = payload.get("source_resolution", {"w": 640, "h": 480})

            with _state_lock:
                _state["detections"] = detections
                _state["source_resolution"] = res
                _state["last_detection_ts"] = time.time()

        except socket.timeout:
            # No packet arrived — clear stale detections after 1s
            with _state_lock:
                if time.time() - _state["last_detection_ts"] > 1.0:
                    _state["detections"] = []
        except (json.JSONDecodeError, KeyError):
            pass
        except Exception as e:
            print(f"[UDP] Error: {e}")
            time.sleep(0.5)


# ---- ROS2 Distance Subscriber (optional — graceful if ROS2 not available) --
def ros2_subscriber():
    """Subscribes to /proximity/distance and updates shared state."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32

        class DistanceNode(Node):
            def __init__(self):
                super().__init__("ws_bridge_distance")
                self.create_subscription(
                    Float32, "/proximity/distance",
                    self.cb, 10
                )

            def cb(self, msg):
                with _state_lock:
                    _state["distance"] = float(msg.data)

        rclpy.init()
        node = DistanceNode()
        print("[ROS2] Subscribed to /proximity/distance")
        rclpy.spin(node)
    except ImportError:
        print("[ROS2] rclpy not available — distance will show as null")
    except Exception as e:
        print(f"[ROS2] Error: {e} — distance will show as null")


# ---- WebSocket Server ------------------------------------------------------
_clients: set = set()
_clients_lock = asyncio.Lock()


async def handler(websocket):
    async with _clients_lock:
        _clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        async with _clients_lock:
            _clients.discard(websocket)


async def broadcaster():
    """Push state to all connected clients at BROADCAST_HZ."""
    interval = 1.0 / BROADCAST_HZ
    while True:
        await asyncio.sleep(interval)

        with _state_lock:
            snap_distance   = _state["distance"]
            snap_detections = list(_state["detections"])
            snap_res        = dict(_state["source_resolution"])

        msg = json.dumps({
            "type": "frame_update",
            "timestamp": time.time(),
            "lidar": {"distance": snap_distance},
            "cv_detections": snap_detections,
            "source_resolution": snap_res,
            "camera_urls": {
                "cam1": CAM1_URL,
                "cam2": CAM2_URL,
            },
        })

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
    # Start background threads
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=ros2_subscriber, daemon=True).start()

    # Run async WebSocket server
    asyncio.run(main())
