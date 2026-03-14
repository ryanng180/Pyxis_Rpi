#!/usr/bin/env python3
"""
Proximity Node
Reads /scan/filtered, finds closest point to cargo ship,
publishes distance + zone for captain dashboard.

Publishes:
  /proximity/distance  (std_msgs/Float32)   - closest distance in metres
  /proximity/zone      (std_msgs/String)     - SAFE / CAUTION / DANGER
  /proximity/status    (std_msgs/String)     - full JSON for dashboard

Zones:
  DANGER  : < 3m   (red)
  CAUTION : 3-10m  (yellow)
  SAFE    : > 10m  (green)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import json
import math


class ProximityNode(Node):

    def __init__(self):
        super().__init__('proximity_node')

        self.declare_parameter('danger_m',  3.0)
        self.declare_parameter('caution_m', 10.0)

        self.danger_m  = self.get_parameter('danger_m').value
        self.caution_m = self.get_parameter('caution_m').value

        self.sub = self.create_subscription(
            LaserScan, '/scan/filtered', self._scan_cb, 10)

        self.pub_dist   = self.create_publisher(Float32, '/proximity/distance', 10)
        self.pub_zone   = self.create_publisher(String,  '/proximity/zone',     10)
        self.pub_status = self.create_publisher(String,  '/proximity/status',   10)

        self._last_zone = None
        self.get_logger().info(
            f'Proximity node ready. '
            f'DANGER<{self.danger_m}m  CAUTION<{self.caution_m}m  SAFE>{self.caution_m}m')

    def _scan_cb(self, msg: LaserScan):
        # Get all valid ranges
        valid_ranges = [r for r in msg.ranges
                        if math.isfinite(r) and msg.range_min <= r <= msg.range_max]

        if not valid_ranges:
            return

        closest = min(valid_ranges)
        avg     = sum(valid_ranges) / len(valid_ranges)

        # Determine zone
        if closest < self.danger_m:
            zone = 'DANGER'
        elif closest < self.caution_m:
            zone = 'CAUTION'
        else:
            zone = 'SAFE'

        # Log zone changes
        if zone != self._last_zone:
            self.get_logger().warn(f'ZONE CHANGE: {self._last_zone} -> {zone}  '
                                   f'closest={closest:.2f}m')
            self._last_zone = zone

        # Publish distance
        dist_msg = Float32()
        dist_msg.data = closest
        self.pub_dist.publish(dist_msg)

        # Publish zone
        zone_msg = String()
        zone_msg.data = zone
        self.pub_zone.publish(zone_msg)

        # Publish full status JSON for dashboard
        status = {
            'closest_m':     round(closest, 2),
            'average_m':     round(avg, 2),
            'zone':          zone,
            'danger_m':      self.danger_m,
            'caution_m':     self.caution_m,
            'point_count':   len(valid_ranges),
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ProximityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
