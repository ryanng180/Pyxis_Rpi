#!/usr/bin/env python3
"""
Ladder distance pipeline tester.
Shows each stage of the pipeline in real-time so you can verify
step by step: detection → target lock → ladder distance.

Usage:
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/scripts/ladder_pipeline_test.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import QuaternionStamped
import math
import json
import time
import subprocess

class PipelineTester(Node):
    def __init__(self):
        super().__init__('ladder_pipeline_test')

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Stage 1: Gimbal tracker debug (has YOLO detection info)
        self._yolo_conf = None
        self._yolo_cx = None
        self._yaw_cmd = None
        self._pitch_cmd = None
        self.create_subscription(
            Float32MultiArray, '/storm32/debug', self._debug_cb, be_qos)

        # Stage 2: Target lock
        self._target_locked = None
        self._lock_ts = 0.0
        self.create_subscription(
            Bool, '/gimbal/target_locked', self._lock_cb, be_qos)

        # Stage 3: Gimbal actual angles (IMU1)
        self._gimbal_yaw = None
        self._gimbal_pitch = None
        self.create_subscription(
            Float32MultiArray, '/gimbal/actual_angles', self._angles_cb, be_qos)

        # Stage 4: Scan data
        self._scan_count = 0
        self._scan_min_range = None
        self.create_subscription(
            LaserScan, '/scan/transformed', self._scan_cb, 10)

        # Stage 5: Ladder distance output
        self._ladder_dist = None
        self._ladder_ts = 0.0
        self.create_subscription(
            Float32, '/ladder/distance', self._dist_cb, 10)

        # Stage 5b: Ladder status (full JSON)
        self._ladder_status = None
        self.create_subscription(
            String, '/ladder/status', self._status_cb, 10)

        # Display timer — 2 Hz
        self.create_timer(0.5, self._display)
        self._start_time = time.monotonic()

    def _debug_cb(self, msg):
        # [yaw_cmd, pitch_cmd, confidence, cx_norm, cy_norm, yaw_error, pitch_error]
        if len(msg.data) >= 5:
            self._yaw_cmd = msg.data[0]
            self._pitch_cmd = msg.data[1]
            self._yolo_conf = msg.data[2]
            self._yolo_cx = msg.data[3]

    def _lock_cb(self, msg):
        self._target_locked = msg.data
        self._lock_ts = time.monotonic()

    def _angles_cb(self, msg):
        if len(msg.data) >= 2:
            self._gimbal_yaw = msg.data[0]
            self._gimbal_pitch = msg.data[1]

    def _scan_cb(self, msg):
        valid = [r for r in msg.ranges if 0.3 < r < 16.0]
        self._scan_count = len(valid)
        self._scan_min_range = min(valid) if valid else None

    def _dist_cb(self, msg):
        self._ladder_dist = msg.data
        self._ladder_ts = time.monotonic()

    def _status_cb(self, msg):
        try:
            self._ladder_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _display(self):
        subprocess.run(['clear'])
        now = time.monotonic()
        elapsed = now - self._start_time

        print("=" * 60)
        print("  LADDER DISTANCE PIPELINE TEST")
        print(f"  Elapsed: {elapsed:.0f}s")
        print("=" * 60)

        # Stage 1: YOLO Detection
        print("\n[1] YOLO DETECTION")
        if self._yolo_conf is not None:
            cx_str = f"{self._yolo_cx:.3f}" if self._yolo_cx else "N/A"
            in_center = self._yolo_cx and abs(self._yolo_cx - 0.5) <= 0.15
            center_str = "YES (in lock zone)" if in_center else "NO (outside lock zone)"
            print(f"    Confidence:  {self._yolo_conf:.3f}")
            print(f"    cx_norm:     {cx_str}  <- {center_str}")
            print(f"    Yaw cmd:     {self._yaw_cmd:.1f}")
        else:
            print("    NO DATA -- gimbal tracker not publishing debug")

        # Stage 2: Target Lock
        print("\n[2] TARGET LOCK")
        if self._target_locked is not None:
            lock_age = now - self._lock_ts
            status = "LOCKED" if self._target_locked else "UNLOCKED"
            print(f"    Status:      {status}  (last update {lock_age:.1f}s ago)")
        else:
            print("    NO DATA -- not receiving /gimbal/target_locked")

        # Stage 3: Gimbal Angles
        print("\n[3] GIMBAL IMU1 ANGLES")
        if self._gimbal_yaw is not None:
            print(f"    Yaw:         {self._gimbal_yaw:.1f}")
            print(f"    Pitch:       {self._gimbal_pitch:.1f}")
        else:
            print("    NO DATA -- not receiving /gimbal/actual_angles")

        # Stage 4: LiDAR Scan
        print("\n[4] LIDAR SCAN (/scan/transformed)")
        if self._scan_min_range is not None:
            print(f"    Valid pts:   {self._scan_count}")
            print(f"    Closest:     {self._scan_min_range:.2f} m")
        else:
            print("    NO DATA -- not receiving /scan/transformed")

        # Stage 5: Ladder Distance
        print("\n[5] LADDER DISTANCE OUTPUT")
        dist_age = now - self._ladder_ts
        if self._ladder_dist is not None and dist_age < 3.0:
            print(f"    Gate dist:   {self._ladder_dist:.3f} m  (updated {dist_age:.1f}s ago)")
            if self._ladder_status:
                s = self._ladder_status
                print(f"    LiDAR range: {s.get('ladder_dist_m', 'N/A')} m")
                print(f"    From mount:  {s.get('dist_from_mount_m', 'N/A')} m")
                print(f"    Lateral:     {s.get('lateral_from_hull_m', 'N/A')} m from hull")
                print(f"    Gimbal yaw:  {s.get('gimbal_yaw_deg', 'N/A')}")
                print(f"    LiDAR angle: {s.get('lidar_angle_deg', 'N/A')}")
        elif self._ladder_dist is not None:
            print(f"    STALE -- last value {self._ladder_dist:.3f} m ({dist_age:.1f}s ago)")
        else:
            print("    NOT PUBLISHING")

        # Summary
        print("\n" + "-" * 60)
        yolo_ok = self._yolo_conf is not None and self._yolo_conf > 0
        lock_ok = self._target_locked is True
        scan_ok = self._scan_count > 0
        dist_ok = self._ladder_dist is not None and dist_age < 3.0

        checks = [
            ("YOLO detecting", yolo_ok),
            ("Target locked", lock_ok),
            ("LiDAR has data", scan_ok),
            ("Distance publishing", dist_ok),
        ]
        for label, ok in checks:
            icon = "OK" if ok else "--"
            print(f"  [{icon}] {label}")

        print("\nCtrl+C to exit")


def main():
    rclpy.init()
    node = PipelineTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
