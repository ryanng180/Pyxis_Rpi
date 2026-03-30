"""
gimbal_tracker_pitch_yaw.py
---------------------------
Runs on: Raspberry Pi 5
Role:    Receives bounding box detections from Jetson (via UDP),
         converts normalised pixel X/Y coordinates -> yaw/pitch angles -> quaternion,
         and commands a Storm32 gimbal through ROS2.

         Controls YAW (pan) and PITCH (tilt) via independent PID loops.
         Roll is always 0 (physically locked via Storm32 calibration).
         Yaw range: ±90° (matching STorM32 limits configured via OlliW).

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
    ros2 run gimbal_tracker gimbal_tracker_pitch_yaw
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

    # --- Gimbal yaw limit (degrees) — matches OlliW-configured ±90° range ---
    PAN_LIMIT_DEG       = 90.0

    # --- Gimbal pitch limit (degrees) — Yuntai 3-axis gimbal hardware max ---
    TILT_LIMIT_DEG      = 25.0

    # --- Arducam B0589 145° diagonal FOV @ 1280x720 (16:9) ---
    # Calculated: HFOV = 2*atan(tan(72.5°)*1280/1469) ≈ 140.2°
    #             VFOV = 2*atan(tan(72.5°)* 720/1469) ≈ 114.5°
    # NOTE: original yaw-only node used HFOV=60° — update if yaw tracking
    # feels over-sensitive at wide angles (wide-angle lenses are non-linear
    # at the edges; tune HFOV_DEG down if overshoot is observed).
    HFOV_DEG            = 140.2                 #140.2    

    # --- Vertical field of view (degrees) ---
    VFOV_DEG            = 114.5                 #114.5

    # --- Yaw PID gains (tune on vessel) ---
    PAN_KP              = 0.24
    PAN_KI              = 1.00
    PAN_KD              = 0.20

    # --- Pitch PID gains (tune on vessel) ---
    TILT_KP             = 0.15
    TILT_KI             = 0.70
    TILT_KD             = 0.10

    # --- Sea-state low-pass filter (0 < alpha <= 1, lower = smoother) ---
    SMOOTHING_ALPHA     = 0.5

    # --- Confidence gate: ignore detections below this threshold ---
    MIN_CONFIDENCE      = 0.30

    # --- Staleness: ignore packets older than this (seconds) ---
    MAX_DETECTION_AGE_S = 0.2

    # --- ROS2 topic names ---
    TOPIC_GIMBAL_CMD    = "/gimbal/controller/target_orientation"
    TOPIC_GIMBAL_DEBUG  = "/storm32/debug"   # [yaw_cmd, pitch_cmd, confidence, cx_norm, cy_norm, yaw_error, pitch_error]

    # --- Gimbal unlimited mode: False = respect OlliW-configured STorM32 limits ---
    GIMBAL_UNLIMITED    = False

    # --- ROS2 publish rate (Hz) ---
    PUBLISH_RATE_HZ     = 60


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


def cy_norm_to_pitch(cy_norm: float) -> float:
    """
    Normalised image Y (0..1) -> pitch error in degrees.
    0.5 = dead centre = 0 error.
    Positive = target below centre -> tilt gimbal down.
    Image Y increases downward, so subtract 0.5 directly.
    """
    return (cy_norm - 0.5) * Config.VFOV_DEG


def angles_to_quaternion(pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    """
    Combined pitch + yaw -> quaternion (x, y, z, w). Roll = 0.

    Encodes extrinsic rotations: pitch about Y-axis, then yaw about Z-axis.
    This matches the 'syxz' Euler convention used by storm32_node to decode:
        euler = tf.euler_from_quaternion(q, axes="syxz")
        set_angles(pitch=euler[0], roll=euler[1], yaw=euler[2])

    Derivation: q = q_Z(yaw) * q_Y(pitch)
        x = -sin(yaw/2)*sin(pitch/2)
        y =  cos(yaw/2)*sin(pitch/2)
        z =  sin(yaw/2)*cos(pitch/2)
        w =  cos(yaw/2)*cos(pitch/2)
    """
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    sp, cp = math.sin(p / 2), math.cos(p / 2)
    sy, cy = math.sin(y / 2), math.cos(y / 2)
    return -sy * sp, cy * sp, sy * cp, cy * cp  # (x, y, z, w)


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
            reliability = ReliabilityPolicy.BEST_EFFORT,
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
        self._pan_pid = PID(Config.PAN_KP, Config.PAN_KI, Config.PAN_KD,
                            output_limit=Config.PAN_LIMIT_DEG)

        # Pitch PID
        self._tilt_pid = PID(Config.TILT_KP, Config.TILT_KI, Config.TILT_KD,
                             output_limit=Config.TILT_LIMIT_DEG)

        # Smoothed errors (EMA)
        self._smooth_yaw   = 0.0
        self._smooth_pitch = 0.0
        self._first_det    = True

        # Control loop timer
        period = 1.0 / Config.PUBLISH_RATE_HZ
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"GimbalTrackerNode (pitch+yaw) started. "
            f"Publishing to {Config.TOPIC_GIMBAL_CMD} | "
            f"Yaw limit: ±{Config.PAN_LIMIT_DEG}° | "
            f"Pitch limit: ±{Config.TILT_LIMIT_DEG}°")

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

        # 1. Pixel coords -> errors (degrees)
        yaw_error   = cx_norm_to_yaw(best.cx_norm)
        pitch_error = cy_norm_to_pitch(best.cy_norm)

        # 2. Low-pass filter (EMA)
        alpha = Config.SMOOTHING_ALPHA
        if self._first_det:
            self._smooth_yaw   = yaw_error
            self._smooth_pitch = pitch_error
            self._first_det    = False
        else:
            self._smooth_yaw   = alpha * yaw_error   + (1 - alpha) * self._smooth_yaw
            self._smooth_pitch = alpha * pitch_error + (1 - alpha) * self._smooth_pitch

        # 3. PID -> commands
        yaw_cmd   = self._pan_pid.update(self._smooth_yaw,   now)
        pitch_cmd = self._tilt_pid.update(self._smooth_pitch, now)

        # 4. Clamp to physical limits
        yaw_cmd   = clamp(yaw_cmd,   Config.PAN_LIMIT_DEG)
        pitch_cmd = clamp(pitch_cmd, Config.TILT_LIMIT_DEG)

        # 5. Pitch + yaw -> quaternion (roll=0)
        qx, qy, qz, qw = angles_to_quaternion(pitch_cmd, yaw_cmd)

        # 6. Publish GimbalOrientation to Storm32 driver
        msg = GimbalOrientation()
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.unlimited     = Config.GIMBAL_UNLIMITED
        self._pub_cmd.publish(msg)

        # 7. Publish debug [yaw_cmd, pitch_cmd, confidence, cx_norm, cy_norm, yaw_error, pitch_error]
        dmsg = Float32MultiArray()
        dmsg.data = [
            float(yaw_cmd),
            float(pitch_cmd),
            float(best.confidence),
            float(best.cx_norm),
            float(best.cy_norm),
            float(yaw_error),
            float(pitch_error),
        ]
        self._pub_debug.publish(dmsg)

        self.get_logger().debug(
            f"[{frame.source}] class={frame.target_class} "
            f"best_conf={best.confidence:.2f} "
            f"cx={best.cx_norm:.3f} cy={best.cy_norm:.3f} "
            f"yaw_err={yaw_error:+.2f} pitch_err={pitch_error:+.2f} "
            f"yaw_cmd={yaw_cmd:+.2f} pitch_cmd={pitch_cmd:+.2f} "
            f"q=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})"
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
