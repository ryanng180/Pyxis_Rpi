#!/usr/bin/env python3
"""
Scan Filter Node
- Masks out the ~180° arc pointing INTO the ferry cabin
- Keeps only the starboard-facing arc pointing toward the sea/cargo
- Publishes filtered scan on /scan/filtered

ANGLE CONVENTION (after 180 roll correction, looking from above):
  0°   = bow (forward)
  90°  = port (left)
  -90° = starboard (right) ← cargo ship docks here
  180° = stern (back)

CABIN_MASK: the arc pointing inward through the ferry roof.
  Tune CABIN_MIN_DEG and CABIN_MAX_DEG after installation by
  checking raw /scan in RViz and identifying which angles hit the cabin.

OUTWARD_ARC: the 180° facing the sea.
  Default: -180° to 0° = starboard half
  Tune OUTWARD_MIN_DEG and OUTWARD_MAX_DEG to match your install.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter_node')

        # ── Tunable parameters ─────────────────────────────────────────
        # Arc to KEEP (outward facing / starboard side toward cargo)
        # Adjust these after checking raw scan in RViz on the boat
        self.declare_parameter('outward_min_deg', -180.0)  # starboard start
        self.declare_parameter('outward_max_deg',    0.0)  # starboard end

        # Range limits - ignore readings outside these (metres)
        self.declare_parameter('min_range',   0.3)   # ignore reflections < 30cm
        self.declare_parameter('max_range',  16.0)   # RPLIDAR C1 max range

        self.outward_min = math.radians(
            self.get_parameter('outward_min_deg').value)
        self.outward_max = math.radians(
            self.get_parameter('outward_max_deg').value)
        self.min_range   = self.get_parameter('min_range').value
        self.max_range   = self.get_parameter('max_range').value

        self.sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.pub = self.create_publisher(
            LaserScan, '/scan/filtered', 10)

        self.get_logger().info(
            f'Scan filter ready. Keeping arc: '
            f'{self.get_parameter("outward_min_deg").value}° to '
            f'{self.get_parameter("outward_max_deg").value}°')

    def _scan_cb(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header         = msg.header
        filtered.angle_min      = msg.angle_min
        filtered.angle_max      = msg.angle_max
        filtered.angle_increment= msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time      = msg.scan_time
        filtered.range_min      = self.min_range
        filtered.range_max      = self.max_range
        filtered.ranges         = []
        filtered.intensities    = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Mask out angles outside the outward-facing arc
            in_arc = self.outward_min <= angle <= self.outward_max

            # Mask out invalid or out-of-range readings
            valid = self.min_range <= r <= self.max_range

            if in_arc and valid:
                filtered.ranges.append(r)
            else:
                filtered.ranges.append(float('inf'))

            if msg.intensities:
                filtered.intensities.append(
                    msg.intensities[i] if in_arc and valid else 0.0)

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
