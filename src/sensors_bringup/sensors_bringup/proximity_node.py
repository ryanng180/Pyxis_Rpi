#!/usr/bin/env python3
"""
Proximity Node
Reads LiDAR scan, finds closest point to cargo ship,
subtracts hull offset to get ferry-hull-to-cargo distance,
publishes distance + zone for captain dashboard.

Publishes:
  /proximity/distance  (std_msgs/Float32)   - hull-to-cargo distance in metres
  /proximity/zone      (std_msgs/String)     - TOO_CLOSE / OPTIMAL / TOO_FAR
  /proximity/status    (std_msgs/String)     - full JSON for dashboard

Zones (target-band — placeholder values, tune during sea trials):
  TOO_CLOSE : hull-to-cargo < 0.3m  (red — collision risk)
  OPTIMAL   : 0.3m to 1.0m          (green — safe boarding range)
  TOO_FAR   : hull-to-cargo > 1.0m  (red — can't board safely)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json
import math


class ProximityNode(Node):

    def __init__(self):
        super().__init__('proximity_node')

        self.declare_parameter('too_close_m',   0.3)
        self.declare_parameter('too_far_m',     1.0)
        self.declare_parameter('hull_offset_m', 1.077)
        # Default to /scan/filtered so the node works standalone (tabletop testing)
        # without requiring scan_transform_node. Set to /scan/transformed in the
        # launch file for ferry deployment to use tilt-corrected horizontal distances.
        self.declare_parameter('scan_topic', '/scan/filtered')
        # Restrict proximity search to a perpendicular arc around starboard (90°).
        # scan_filter_node is opened wide (0-180°) for ladder calibration, so
        # proximity applies its own narrower arc here to ignore bow/stern clutter.
        self.declare_parameter('arc_min_deg',  60.0)
        self.declare_parameter('arc_max_deg', 120.0)

        self.too_close_m   = self.get_parameter('too_close_m').value
        self.too_far_m     = self.get_parameter('too_far_m').value
        self.hull_offset_m = self.get_parameter('hull_offset_m').value
        scan_topic         = self.get_parameter('scan_topic').value
        self.arc_min_rad   = math.radians(self.get_parameter('arc_min_deg').value)
        self.arc_max_rad   = math.radians(self.get_parameter('arc_max_deg').value)

        self.sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10)

        self.get_logger().info(f'Subscribing to {scan_topic}')

        self.pub_dist   = self.create_publisher(Float32, '/proximity/distance', 10)
        self.pub_zone   = self.create_publisher(String,  '/proximity/zone',     10)
        self.pub_status = self.create_publisher(String,  '/proximity/status',   10)
        self.pub_arc_marker = self.create_publisher(
            Marker, '/proximity/arc_bounds', 10)

        self._last_zone = None
        self.get_logger().info(
            f'Proximity node ready. hull_offset={self.hull_offset_m:.3f}m  '
            f'TOO_CLOSE<{self.too_close_m}m  '
            f'OPTIMAL {self.too_close_m}-{self.too_far_m}m  '
            f'TOO_FAR>{self.too_far_m}m')

    def _scan_cb(self, msg: LaserScan):
        # Restrict to the configured arc window (proximity cares about the
        # perpendicular hull region, not bow/stern clutter)
        idx_lo = max(0, int((self.arc_min_rad - msg.angle_min) / msg.angle_increment))
        idx_hi = min(len(msg.ranges) - 1,
                     int((self.arc_max_rad - msg.angle_min) / msg.angle_increment))

        valid_ranges = [r for r in msg.ranges[idx_lo:idx_hi + 1]
                        if math.isfinite(r) and msg.range_min <= r <= msg.range_max]

        if not valid_ranges:
            return

        raw_closest = min(valid_ranges)
        raw_avg     = sum(valid_ranges) / len(valid_ranges)

        # Subtract hull offset: LiDAR-to-cargo → hull-to-cargo
        closest = max(0.0, raw_closest - self.hull_offset_m)

        # Target-band zone classification
        if closest < self.too_close_m:
            zone = 'TOO_CLOSE'
        elif closest > self.too_far_m:
            zone = 'TOO_FAR'
        else:
            zone = 'OPTIMAL'

        # Log zone changes
        if zone != self._last_zone:
            self.get_logger().warn(f'ZONE CHANGE: {self._last_zone} -> {zone}  '
                                   f'hull-to-cargo={closest:.2f}m  '
                                   f'raw={raw_closest:.2f}m')
            self._last_zone = zone

        # Publish hull-to-cargo distance
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
            'raw_lidar_m':   round(raw_closest, 2),
            'zone':          zone,
            'too_close_m':   self.too_close_m,
            'too_far_m':     self.too_far_m,
            'hull_offset_m': self.hull_offset_m,
            'point_count':   len(valid_ranges),
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.pub_status.publish(status_msg)

        # RViz overlay: two rays bounding the proximity arc, in the scan's frame
        self._publish_arc_marker(
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp,
            max_range=float(msg.range_max),
        )

    def _publish_arc_marker(self, frame_id: str, stamp, max_range: float):
        """Publish two line segments at arc_min_deg and arc_max_deg from the
        scan origin, length = max_range, in the scan's own frame."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp    = stamp
        marker.ns    = 'proximity_arc'
        marker.id    = 0
        marker.type  = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02          # 2 cm line width
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9           # translucent yellow
        marker.pose.orientation.w = 1.0

        origin = Point(x=0.0, y=0.0, z=0.0)
        for angle_rad in (self.arc_min_rad, self.arc_max_rad):
            end = Point(
                x=max_range * math.cos(angle_rad),
                y=max_range * math.sin(angle_rad),
                z=0.0,
            )
            marker.points.append(origin)
            marker.points.append(end)

        self.pub_arc_marker.publish(marker)


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
