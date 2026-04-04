#!/usr/bin/env python3
"""Debug ladder distance pipeline — check each gate."""
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Bool

SCAN_ARC_MIN = 0.0
SCAN_ARC_MAX = 180.0
YAW_OFFSET = 60.0  # gimbal-to-LiDAR mounting offset


class LadderDebug(Node):
    def __init__(self):
        super().__init__('ladder_debug')
        self.gimbal_yaw = None
        self.locked = None
        self.scan = None

        lock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            QuaternionStamped, '/gimbal/controller/camera_orientation',
            self.gimbal_cb, 1)
        self.create_subscription(Bool, '/gimbal/target_locked', self.lock_cb, lock_qos)
        self.create_subscription(LaserScan, '/scan/transformed', self.scan_cb, 10)
        self.create_timer(1.0, self.report)

    def gimbal_cb(self, msg):
        q = msg.quaternion
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.gimbal_yaw = math.degrees(yaw_rad)

    def lock_cb(self, msg):
        self.locked = msg.data

    def scan_cb(self, msg):
        self.scan = msg

    def range_at_angle(self, angle_deg):
        if self.scan is None:
            return None, -1
        angle_rad = math.radians(angle_deg)
        idx = round((angle_rad - self.scan.angle_min) / self.scan.angle_increment)
        if not (0 <= idx < len(self.scan.ranges)):
            return None, idx
        r = self.scan.ranges[idx]
        if not math.isfinite(r) or r < self.scan.range_min or r > self.scan.range_max:
            return None, idx
        return r, idx

    def find_closest_valid(self):
        """Find the closest valid scan point and its angle."""
        if self.scan is None:
            return None, None, None
        best_r = float('inf')
        best_a = None
        best_i = None
        for i, r in enumerate(self.scan.ranges):
            if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                if r < best_r:
                    best_r = r
                    best_a = math.degrees(self.scan.angle_min + i * self.scan.angle_increment)
                    best_i = i
        if best_a is None:
            return None, None, None
        return best_r, best_a, best_i

    def report(self):
        print('=' * 60)
        print(f'  target_locked:  {self.locked}')
        if self.gimbal_yaw is not None:
            print(f'  gimbal_yaw:     {self.gimbal_yaw:.1f} deg (IMU1)')
        else:
            print(f'  gimbal_yaw:     None')
        print(f'  scan received:  {self.scan is not None}')

        if self.gimbal_yaw is None or self.scan is None:
            print('  -> Waiting for data...')
            return

        # Negated + offset for scan lookup
        scan_angle = -self.gimbal_yaw + YAW_OFFSET
        print(f'  scan_angle:     {scan_angle:.1f} deg (-gimbal_yaw + {YAW_OFFSET} offset)')

        r, idx = self.range_at_angle(scan_angle)
        print(f'  scan index:     {idx}')
        print(f'  range at angle: {r}')

        in_arc = SCAN_ARC_MIN <= scan_angle <= SCAN_ARC_MAX
        print(f'  in scan arc [{SCAN_ARC_MIN}, {SCAN_ARC_MAX}]: {in_arc}')

        # Show where the closest valid scan point actually is
        closest_r, closest_a, closest_i = self.find_closest_valid()
        if closest_a is not None:
            offset = closest_a - scan_angle
            print(f'  --- closest valid scan point ---')
            print(f'  closest range:  {closest_r:.3f}m at scan angle {closest_a:.1f} deg (idx {closest_i})')
            print(f'  OFFSET:         {offset:.1f} deg (closest_scan - negated_gimbal)')
            print(f'  -> Gimbal zero is off by ~{offset:.0f} deg from LiDAR zero')
        else:
            print(f'  No valid scan points found!')

        if not in_arc:
            print(f'  -> BLOCKED: angle {scan_angle:.1f} outside arc')
        elif r is None:
            print(f'  -> BLOCKED: no valid range at this angle')
        elif not self.locked:
            print(f'  -> BLOCKED: target not locked')
        else:
            print(f'  -> WOULD PUBLISH: range={r:.3f}m')


rclpy.init()
node = LadderDebug()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
