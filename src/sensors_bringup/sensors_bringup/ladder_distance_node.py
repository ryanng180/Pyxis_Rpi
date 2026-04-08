#!/usr/bin/env python3
"""
Ladder Distance Node — Fixed Starboard-Home Offset

Combines the tilt-corrected LiDAR scan with the actual measured gimbal
orientation (IMU1) to produce a parallax-corrected ladder distance.

The gimbal is configured via OlliW to boot-home at STARBOARD: IMU1 yaw = 0°
points dead starboard. With the ±90° mechanical yaw range this gives the
full bow↔stern sweep along the starboard side, which is where the pilot
ladder lives.

Sign convention:
  Gimbal yaw: 0=starboard, + = toward bow, − = toward stern (to verify)
  LiDAR scan: 0=bow, +90=starboard, +180=stern (inverted mount)
  Mapping:    scan_angle_deg = -gimbal_yaw_deg + MECHANICAL_OFFSET_DEG

Subscribes:
  /scan/transformed                    (sensor_msgs/LaserScan)
  /gimbal/controller/camera_orientation (geometry_msgs/QuaternionStamped)
  /gimbal/target_locked                (std_msgs/Bool)

Publishes:
  /ladder/distance   (std_msgs/Float32)  — dist from sensor mount (m)
  /ladder/status     (std_msgs/String)   — full JSON for dashboard
"""

import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import QuaternionStamped

from .parallax import get_ladder_distance, get_lidar_angle, \
    SCAN_ARC_MIN_DEG, SCAN_ARC_MAX_DEG

# Hardcoded mechanical offset: gimbal yaw 0° = starboard = LiDAR scan 90°.
# Tweak slightly (e.g. 88–92) if the OlliW boot-home drifts a few degrees.
MECHANICAL_OFFSET_DEG = 90.0

# Staleness limit — ignore gimbal orientation older than this
GIMBAL_MAX_AGE_S = 0.5


class LadderDistanceNode(Node):

    def __init__(self):
        super().__init__('ladder_distance_node')

        self.declare_parameter(
            'gimbal_orientation_topic', '/gimbal/controller/camera_orientation')
        self.declare_parameter('scan_topic', '/scan/transformed')
        self.declare_parameter('mechanical_offset_deg', MECHANICAL_OFFSET_DEG)

        gimbal_topic = self.get_parameter('gimbal_orientation_topic').value
        scan_topic   = self.get_parameter('scan_topic').value
        self._offset = float(self.get_parameter('mechanical_offset_deg').value)

        self._latest_gimbal_yaw_deg: float | None = None
        self._latest_gimbal_stamp                  = None
        self._target_locked = False

        self.create_subscription(
            QuaternionStamped, gimbal_topic, self._gimbal_cb, 1)
        self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10)
        lock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            Bool, '/gimbal/target_locked', self._lock_cb, lock_qos)

        self._pub_dist   = self.create_publisher(Float32, '/ladder/distance', 10)
        self._pub_status = self.create_publisher(String,  '/ladder/status',   10)

        self.get_logger().info(
            f'Ladder distance node ready (fixed offset = {self._offset:.1f}°).\n'
            f'  scan:   {scan_topic}\n'
            f'  gimbal: {gimbal_topic}')

    def _lock_cb(self, msg: Bool):
        self._target_locked = msg.data

    def _gimbal_cb(self, msg: QuaternionStamped):
        q = msg.quaternion
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest_gimbal_yaw_deg = math.degrees(yaw_rad)
        self._latest_gimbal_stamp   = msg.header.stamp

    def _scan_cb(self, msg: LaserScan):
        if self._latest_gimbal_yaw_deg is None:
            self.get_logger().warn(
                'No gimbal orientation received yet.', throttle_duration_sec=5.0)
            return

        # Reject stale gimbal data
        now_s   = self.get_clock().now().nanoseconds * 1e-9
        stamp_s = (self._latest_gimbal_stamp.sec
                   + self._latest_gimbal_stamp.nanosec * 1e-9)
        if now_s - stamp_s > GIMBAL_MAX_AGE_S:
            self.get_logger().warn(
                f'Gimbal orientation stale ({now_s - stamp_s:.2f}s). Skipping.',
                throttle_duration_sec=2.0)
            return

        if not self._target_locked:
            return

        gimbal_yaw = self._latest_gimbal_yaw_deg

        # ── Step 1: seed range at the fixed-offset scan angle ─────────
        seed_angle = -gimbal_yaw + self._offset
        seed_range = self._range_at_angle(msg, seed_angle)
        if seed_range is None:
            self.get_logger().debug('No valid scan point at seed angle.')
            return

        # ── Step 2: parallax-correct the angle ────────────────────────
        corrected_angle_deg = get_lidar_angle(gimbal_yaw, seed_range)
        scan_angle_deg = -corrected_angle_deg + self._offset

        if not (SCAN_ARC_MIN_DEG <= scan_angle_deg <= SCAN_ARC_MAX_DEG):
            return

        corrected_range = self._range_at_angle(msg, scan_angle_deg)
        if corrected_range is None:
            return

        # ── Step 3: full parallax-corrected distances ─────────────────
        result = get_ladder_distance(gimbal_yaw, corrected_range)

        # ── Publish ───────────────────────────────────────────────────
        mount_dist = result['dist_from_mount_m']

        dist_msg      = Float32()
        dist_msg.data = mount_dist
        self._pub_dist.publish(dist_msg)

        status = {
            'dist_from_mount_m':        result['dist_from_mount_m'],
            'ladder_dist_m':            result['ladder_dist_m'],
            'lateral_from_hull_m':      result['lateral_from_hull_m'],
            'gimbal_yaw_deg':           round(gimbal_yaw, 2),
            'lidar_angle_deg':          result['lidar_angle_deg'],
            'parallax_error_deg':       result['angle_error_deg'],
            'yaw_offset_deg':           round(self._offset, 1),
        }
        status_msg      = String()
        status_msg.data = json.dumps(status)
        self._pub_status.publish(status_msg)

    def _range_at_angle(self, msg: LaserScan, angle_deg: float) -> float | None:
        """Return the tilt-corrected range at the given angle (degrees)."""
        angle_rad = math.radians(angle_deg)
        idx = round((angle_rad - msg.angle_min) / msg.angle_increment)

        if not (0 <= idx < len(msg.ranges)):
            return None

        r = msg.ranges[idx]
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            return None

        return r


def main(args=None):
    rclpy.init(args=args)
    node = LadderDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
