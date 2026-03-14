"""
gimbal_tracker_yaw.py
---------------------
Runs on: Raspberry Pi 5
Role:    Receives bounding box detections from Jetson (via UDP),
         converts normalised pixel X coordinate -> yaw angle -> quaternion,
         and commands a Storm32 gimbal through ROS2.

         X and Y axes are physically locked via Storm32 calibration software.
         This script controls YAW ONLY. Pitch and roll are always 0.

         If multiple detections are present, the highest confidence one is used.
         If target_valid is false, the gimbal holds its last position.

Input JSON format:
    {
        "timestamp": 1712345678.12,
        "source": "cam1",
        "frame_width": 1280,
        "frame_height": 720,
        "target_class": "pilot_ladder",
        "target_valid": true,
        "num_ladders": 2,
        "detections": [
            {
                "confidence": 0.91,
                "x1": 410, "y1": 110, "x2": 500, "y2": 620,
                "cx": 455, "cy": 365,
                "cx_norm": 0.355,
                "cy_norm": 0.507
            }
        ]
    }

Architecture
------------
  Jetson (CV model) --UDP JSON--> Pi 5 (this script) --ROS2--> Storm32 gimbal

Dependencies (Pi 5):
    ROS2 Jazzy + rclpy
    arcros_interface, std_msgs  (standard ROS2 packages)

Usage:
    ros2 run gimbal_tracker gimbal_tracker
"""

import math
import socket
import json
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from arcros_interface.msg import GimbalOrientation
from std_msgs.msg import Float32MultiArray


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

class Config:
    # --- Network (Jetson -> Pi UDP stream) ---
    UDP_HOST            = "0.0.0.0"
    UDP_PORT            = 5105
    UDP_BUFFER          = 4096

    # --- Gimbal yaw limit (degrees) ---
    PAN_LIMIT_DEG       = 180.0

    # --- Horizontal field of view (degrees) - adjust to your lens ---
    HFOV_DEG            = 60.0

    # --- PID gains (tune on vessel) ---
    PAN_KP              = 0.8
    PAN_KI              = 0.01
    PAN_KD              = 0.15

    # --- Sea-state low-pass filter (0 < alpha <= 1, lower = smoother) ---
    SMOOTHING_ALPHA     = 0.3

    # --- Confidence gate: ignore detections below this threshold ---
    MIN_CONFIDENCE      = 0.30

    # --- Staleness: ignore packets older than this (seconds) ---
    MAX_DETECTION_AGE_S = 0.5

    # --- ROS2 topic names ---
    TOPIC_GIMBAL_CMD    = "/gimbal/controller/target_orientation"
    TOPIC_GIMBAL_DEBUG  = "/storm32/debug"                # [yaw_cmd, confidence, cx_norm, yaw_error]

    # --- Gimbal unlimited mode (removes Storm32 angle limits) ---
    GIMBAL_UNLIMITED    = False

    # --- ROS2 publish rate (Hz) ---
    PUBLISH_RATE_HZ     = 30


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class BBox:
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    cx: int
    cy: int
    cx_norm: float
    cy_norm: float

    @staticmethod
    def from_dict(d: dict) -> "BBox":
        return BBox(
            confidence = float(d["confidence"]),
            x1         = int(d["x1"]),
            y1         = int(d["y1"]),
            x2         = int(d["x2"]),
            y2         = int(d["y2"]),
            cx         = int(d["cx"]),
            cy         = int(d["cy"]),
            cx_norm    = float(d["cx_norm"]),
            cy_norm    = float(d["cy_norm"]),
        )


@dataclass
class DetectionFrame:
    timestamp:    float
    source:       str
    frame_width:  int
    frame_height: int
    target_class: str
    target_valid: bool
    num_ladders:  int
    detections:   List[BBox] = field(default_factory=list)

    @staticmethod
    def from_dict(d: dict) -> "DetectionFrame":
        return DetectionFrame(
            timestamp    = float(d["timestamp"]),
            source       = str(d.get("source", "")),
            frame_width  = int(d.get("frame_width", 1280)),
            frame_height = int(d.get("frame_height", 720)),
            target_class = str(d.get("target_class", "")),
            target_valid = bool(d.get("target_valid", False)),
            num_ladders  = int(d.get("num_ladders", 0)),
            detections   = [BBox.from_dict(b) for b in d.get("detections", [])],
        )

    def is_fresh(self) -> bool:
        return (time.time() - self.timestamp) <= Config.MAX_DETECTION_AGE_S

    def best_detection(self) -> Optional[BBox]:
        """Return highest confidence detection above the confidence gate.
        Returns None if target_valid is False or no detection passes the gate."""
        if not self.target_valid or not self.detections:
            return None
        candidates = [b for b in self.detections if b.confidence >= Config.MIN_CONFIDENCE]
        if not candidates:
            return None
        return max(candidates, key=lambda b: b.confidence)


# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 90.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time: Optional[float] = None

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def update(self, error: float, now: Optional[float] = None) -> float:
        now = now or time.monotonic()
        dt  = (now - self._prev_time) if self._prev_time else 0.033
        dt  = max(dt, 1e-4)

        self._integral += error * dt
        i_limit = self.limit / self.ki if self.ki else 1e9
        self._integral = max(-i_limit, min(i_limit, self._integral))

        derivative = (error - self._prev_error) / dt
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(-self.limit, min(self.limit, output))

        self._prev_error = error
        self._prev_time  = now
        return output


# ---------------------------------------------------------------------------
# Coordinate math
# ---------------------------------------------------------------------------

def cx_norm_to_yaw(cx_norm: float) -> float:
    """
    Normalised image X (0..1) -> yaw error in degrees.
    0.5 = dead centre = 0 error.
    Positive = target right of centre -> rotate gimbal right.
    """
    return -(cx_norm - 0.5) * Config.HFOV_DEG


def yaw_to_quaternion(yaw_deg: float) -> Tuple[float, float, float, float]:
    """
    Pure yaw -> quaternion (x, y, z, w). Pitch=0, Roll=0.
    Storm32 ZYX convention.
    """
    yaw = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# UDP receiver (background thread)
# ---------------------------------------------------------------------------

class UDPReceiver(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self._latest: Optional[DetectionFrame] = None
        self._lock    = threading.Lock()
        self._sock    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((Config.UDP_HOST, Config.UDP_PORT))
        self._sock.settimeout(1.0)
        self._running = True

    def run(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(Config.UDP_BUFFER)
                payload = json.loads(data.decode("utf-8"))
                frame = DetectionFrame.from_dict(payload)
                with self._lock:
                    self._latest = frame
            except socket.timeout:
                pass
            except (json.JSONDecodeError, KeyError, ValueError):
                pass  # malformed packet -- discard silently

    def get_latest(self) -> Optional[DetectionFrame]:
        with self._lock:
            return self._latest

    def stop(self):
        self._running = False
        self._sock.close()


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class GimbalTrackerNode(Node):
    def __init__(self):
        super().__init__("gimbal_tracker")

        qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        # Publisher — arcros_interface/msg/GimbalOrientation
        self._pub_cmd   = self.create_publisher(
            GimbalOrientation, Config.TOPIC_GIMBAL_CMD, qos)
        self._pub_debug = self.create_publisher(
            Float32MultiArray, Config.TOPIC_GIMBAL_DEBUG, qos)

        # UDP receiver
        self._udp = UDPReceiver()
        self._udp.start()
        self.get_logger().info(
            f"Listening for detections on UDP {Config.UDP_HOST}:{Config.UDP_PORT}")

        # Yaw PID
        self._pid = PID(Config.PAN_KP, Config.PAN_KI, Config.PAN_KD,
                        output_limit=Config.PAN_LIMIT_DEG)

        # Smoothed yaw error (EMA)
        self._smooth_yaw = 0.0
        self._first_det  = True

        # Control loop timer
        period = 1.0 / Config.PUBLISH_RATE_HZ
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"GimbalTrackerNode started. Publishing to {Config.TOPIC_GIMBAL_CMD}")

    # ------------------------------------------------------------------
    def _control_loop(self):
        frame = self._udp.get_latest()

        # Guard: no frame or stale
        if frame is None or not frame.is_fresh():
            return

        best = frame.best_detection()
        if best is None:
            self.get_logger().debug(
                f"No valid detection — holding position. "
                f"target_valid={frame.target_valid} "
                f"detections={len(frame.detections)}"
            )
            return

        now = time.monotonic()

        # 1. cx_norm -> yaw error (degrees)
        yaw_error = cx_norm_to_yaw(best.cx_norm)

        # 2. Low-pass filter
        alpha = Config.SMOOTHING_ALPHA
        if self._first_det:
            self._smooth_yaw = yaw_error
            self._first_det  = False
        else:
            self._smooth_yaw = alpha * yaw_error + (1 - alpha) * self._smooth_yaw

        # 3. PID -> yaw command
        yaw_cmd = self._pid.update(self._smooth_yaw, now)

        # 4. Clamp to physical limits
        yaw_cmd = clamp(yaw_cmd, Config.PAN_LIMIT_DEG)

        # 5. Yaw -> quaternion (pitch=0, roll=0)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_cmd)

        # 6. Publish GimbalOrientation to Storm32 driver
        msg = GimbalOrientation()
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.unlimited     = Config.GIMBAL_UNLIMITED
        self._pub_cmd.publish(msg)

        # 7. Publish debug [yaw_cmd, confidence, cx_norm, yaw_error]
        dmsg = Float32MultiArray()
        dmsg.data = [
            float(yaw_cmd),
            float(best.confidence),
            float(best.cx_norm),
            float(yaw_error),
        ]
        self._pub_debug.publish(dmsg)

        self.get_logger().debug(
            f"[{frame.source}] class={frame.target_class} "
            f"best_conf={best.confidence:.2f} cx_norm={best.cx_norm:.3f} "
            f"yaw_err={yaw_error:+.2f} smooth={self._smooth_yaw:+.2f} "
            f"cmd={yaw_cmd:+.2f} q=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})"
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._udp.stop()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GimbalTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
