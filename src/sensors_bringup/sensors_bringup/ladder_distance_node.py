#!/usr/bin/env python3
"""
Ladder Distance Node — Live Offset Calibration

Combines the tilt-corrected LiDAR scan with the actual measured gimbal
orientation (IMU1) to produce a parallax-corrected ladder distance.

The gimbal-to-LiDAR angular offset is computed live from tracking data:
when the gimbal is locked on a target, the closest LiDAR point near the
gimbal direction is the target. The offset between gimbal yaw and that
scan angle is smoothed with an EMA filter. No startup calibration needed.

Subscribes:
  /scan/transformed                    (sensor_msgs/LaserScan)
  /gimbal/controller/camera_orientation (geometry_msgs/QuaternionStamped)
  /gimbal/target_locked                (std_msgs/Bool)

Publishes:
  /ladder/distance   (std_msgs/Float32)  — dist from sensor mount (m)
  /ladder/status     (std_msgs/String)   — full JSON for dashboard

Sign convention:
  Gimbal yaw: 0=home, negative=starboard (ROS convention)
  LiDAR scan: 0=bow, +90=starboard, +180=stern (inverted mount)
  Mapping: scan_angle = -gimbal_yaw + offset
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

# Staleness limit — ignore gimbal orientation older than this
GIMBAL_MAX_AGE_S = 0.5

# Live offset estimation
OFFSET_SEARCH_HALF_WIDTH = 40.0  # search ±40° around -gimbal_yaw for closest point
OFFSET_EMA_ALPHA = 0.15          # smoothing factor (0=frozen, 1=instant)
OFFSET_MIN_SAMPLES = 3           # publish distance only after this many offset samples


class LadderDistanceNode(Node):

    def __init__(self):
        super().__init__('ladder_distance_node')

        self.declare_parameter(
            'gimbal_orientation_topic', '/gimbal/controller/camera_orientation')
        self.declare_parameter('scan_topic', '/scan/transformed')

        gimbal_topic = self.get_parameter('gimbal_orientation_topic').value
        scan_topic   = self.get_parameter('scan_topic').value

        self._latest_gimbal_yaw_deg: float | None = None
        self._latest_gimbal_stamp                  = None
        self._target_locked = False

        # Live offset estimation state
        self._yaw_offset: float | None = None
        self._offset_samples = 0

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
            f'Ladder distance node ready (live offset calibration).\n'
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

    def _find_closest_in_window(self, msg: LaserScan,
                                centre_deg: float,
                                half_width_deg: float):
        """Find the closest valid LiDAR point within centre ± half_width degrees.
        Returns (range_m, angle_deg) or (None, None)."""
        lo = math.radians(max(centre_deg - half_width_deg, SCAN_ARC_MIN_DEG))
        hi = math.radians(min(centre_deg + half_width_deg, SCAN_ARC_MAX_DEG))

        idx_lo = max(0, int((lo - msg.angle_min) / msg.angle_increment))
        idx_hi = min(len(msg.ranges) - 1,
                     int((hi - msg.angle_min) / msg.angle_increment))

        best_r, best_a = float('inf'), None
        for i in range(idx_lo, idx_hi + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max and r < best_r:
                best_r = r
                best_a = math.degrees(msg.angle_min + i * msg.angle_increment)

        return (best_r, best_a) if best_a is not None else (None, None)

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
        negated_yaw = -gimbal_yaw  # in scan convention direction

        # ── Live offset estimation ────────────────────────────────────
        # When locked, the closest LiDAR point near the gimbal direction
        # is the tracked target. Use it to compute the offset.
        if self._yaw_offset is None:
            # First time: search a wide window (full starboard arc)
            search_centre = max(SCAN_ARC_MIN_DEG,
                                min(negated_yaw + 60.0, SCAN_ARC_MAX_DEG))
            closest_r, closest_a = self._find_closest_in_window(
                msg, search_centre, 90.0)
        else:
            # Subsequent: search near the expected angle
            expected_scan = negated_yaw + self._yaw_offset
            closest_r, closest_a = self._find_closest_in_window(
                msg, expected_scan, OFFSET_SEARCH_HALF_WIDTH)

        if closest_a is None:
            self.get_logger().debug('No valid LiDAR point in search window.')
            return

        # offset = scan_angle - (-gimbal_yaw) = scan_angle + gimbal_yaw
        measured_offset = closest_a + gimbal_yaw

        if self._yaw_offset is None:
            self._yaw_offset = measured_offset
            self._offset_samples = 1
            self.get_logger().info(
                f'Offset initial estimate: {measured_offset:.1f}° '
                f'(gimbal={gimbal_yaw:.1f}°, scan={closest_a:.1f}°)')
        else:
            self._yaw_offset = (OFFSET_EMA_ALPHA * measured_offset
                                + (1.0 - OFFSET_EMA_ALPHA) * self._yaw_offset)
            self._offset_samples += 1
            if self._offset_samples == OFFSET_MIN_SAMPLES:
                self.get_logger().info(
                    f'Offset calibrated: {self._yaw_offset:.1f}° '
                    f'(after {OFFSET_MIN_SAMPLES} samples)')

        # Don't publish distance until offset is stable
        if self._offset_samples < OFFSET_MIN_SAMPLES:
            return

        # ── Step 1: seed range at the offset-corrected angle ──────────
        seed_range = self._range_at_angle(msg, negated_yaw + self._yaw_offset)
        if seed_range is None:
            self.get_logger().debug('No valid scan point at corrected angle.')
            return

        # ── Step 2: parallax-correct the angle ────────────────────────
        corrected_angle_deg = get_lidar_angle(gimbal_yaw, seed_range)
        scan_angle_deg = -corrected_angle_deg + self._yaw_offset

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
            'yaw_offset_deg':           round(self._yaw_offset, 1),
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
