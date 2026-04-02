#!/usr/bin/env python3
"""
Ladder Distance Node

Combines the tilt-corrected LiDAR scan with the actual measured gimbal
orientation (IMU1) to produce a parallax-corrected ladder distance.

Pipeline:
  1. Gimbal yaw  — from /gimbal/control/camera_orientation (IMU1, actual
                   measured angle, not the lagging commanded angle)
  2. LiDAR range — from /scan/transformed (world-frame, tilt-corrected
                   horizontal distances from scan_transform_node)
  3. parallax.get_lidar_angle() — corrects for LiDAR/gimbal mount offset
  4. parallax.get_ladder_distance() — converts to all vessel-relative distances

Subscribes:
  /scan/transformed                    (sensor_msgs/LaserScan)
  /gimbal/control/camera_orientation   (geometry_msgs/QuaternionStamped)

Publishes:
  /ladder/distance   (std_msgs/Float32)  — dist from pilot boarding gate (m)
  /ladder/status     (std_msgs/String)   — full JSON for dashboard
"""

import math
import json
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from geometry_msgs.msg import QuaternionStamped

from .parallax import get_ladder_distance, SCAN_ARC_MIN_DEG, SCAN_ARC_MAX_DEG


# Staleness limit — ignore gimbal orientation older than this
GIMBAL_MAX_AGE_S = 0.5


class LadderDistanceNode(Node):

    def __init__(self):
        super().__init__('ladder_distance_node')

        self.declare_parameter(
            'gimbal_orientation_topic', '/gimbal/control/camera_orientation')
        self.declare_parameter('scan_topic', '/scan/transformed')

        gimbal_topic = self.get_parameter('gimbal_orientation_topic').value
        scan_topic   = self.get_parameter('scan_topic').value

        self._latest_gimbal_yaw_deg: float | None = None
        self._latest_gimbal_stamp                  = None

        self.create_subscription(
            QuaternionStamped, gimbal_topic, self._gimbal_cb, 1)
        self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10)

        self._pub_dist   = self.create_publisher(Float32, '/ladder/distance', 10)
        self._pub_status = self.create_publisher(String,  '/ladder/status',   10)

        self.get_logger().info(
            f'Ladder distance node ready.\n'
            f'  scan:   {scan_topic}\n'
            f'  gimbal: {gimbal_topic}')

    def _gimbal_cb(self, msg: QuaternionStamped):
        # Extract yaw from the quaternion.
        # Storm32 IMU1 uses axes="syxz"; pitch and roll are actively stabilised
        # to near zero, so standard ZYX yaw extraction is accurate here.
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
        now_s    = self.get_clock().now().nanoseconds * 1e-9
        stamp_s  = (self._latest_gimbal_stamp.sec
                    + self._latest_gimbal_stamp.nanosec * 1e-9)
        if now_s - stamp_s > GIMBAL_MAX_AGE_S:
            self.get_logger().warn(
                f'Gimbal orientation stale ({now_s - stamp_s:.2f}s). Skipping.',
                throttle_duration_sec=2.0)
            return

        gimbal_yaw = self._latest_gimbal_yaw_deg

        # ── Step 1: seed distance from the raw scan at the gimbal angle ───
        # Used as the starting estimate for the parallax iteration.
        seed_range = self._range_at_angle(msg, gimbal_yaw)
        if seed_range is None:
            self.get_logger().debug('No valid scan point near gimbal angle.')
            return

        # ── Step 2: parallax-correct the LiDAR angle and fetch range ──────
        # get_ladder_distance calls get_lidar_angle internally, but we need
        # the corrected angle to fetch the right range from the scan.
        from .parallax import get_lidar_angle
        corrected_angle_deg = get_lidar_angle(gimbal_yaw, seed_range)

        # Verify corrected angle is within the valid scan arc
        if not (SCAN_ARC_MIN_DEG <= corrected_angle_deg <= SCAN_ARC_MAX_DEG):
            self.get_logger().debug(
                f'Corrected angle {corrected_angle_deg:.1f}° outside scan arc '
                f'[{SCAN_ARC_MIN_DEG}, {SCAN_ARC_MAX_DEG}]°.')
            return

        # Fetch tilt-corrected horizontal range at the parallax-corrected angle
        corrected_range = self._range_at_angle(msg, corrected_angle_deg)
        if corrected_range is None:
            return

        # ── Step 3: full parallax-corrected distances ──────────────────────
        result = get_ladder_distance(gimbal_yaw, corrected_range)

        # ── Publish ────────────────────────────────────────────────────────
        gate_dist = result['dist_to_boarding_gate_m']

        dist_msg      = Float32()
        dist_msg.data = gate_dist
        self._pub_dist.publish(dist_msg)

        status = {
            'dist_to_boarding_gate_m':  gate_dist,
            'dist_from_mount_m':        result['dist_from_mount_m'],
            'ladder_dist_m':            result['ladder_dist_m'],
            'lateral_from_hull_m':      result['lateral_from_hull_m'],
            'gimbal_yaw_deg':           round(gimbal_yaw, 2),
            'lidar_angle_deg':          result['lidar_angle_deg'],
            'parallax_error_deg':       result['angle_error_deg'],
        }
        status_msg      = String()
        status_msg.data = json.dumps(status)
        self._pub_status.publish(status_msg)

    def _range_at_angle(self, msg: LaserScan, angle_deg: float) -> float | None:
        """Return the tilt-corrected range from the scan at the given angle (degrees).
        Returns None if the point is invalid or out of bounds."""
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
