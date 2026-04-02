#!/usr/bin/env python3
"""
Scan Transform Node

Transforms the filtered LiDAR scan from laser_frame into the world frame,
applying both the static LiDAR mount correction (180° roll for upside-down
mount) and the live boat tilt from the IMU complementary filter.

Each valid range reading is replaced with the true horizontal (XY-plane)
distance in world frame, removing the systematic error introduced by vessel
pitch and roll.

Transform chain used:   laser_frame → base_link → world
  laser_frame → base_link : static 180° roll + z offset (tilt_tf_node)
  base_link   → world     : live boat pitch/roll from IMU (tilt_tf_node)

Subscribes:
  /scan/filtered     (sensor_msgs/LaserScan)  — arc-masked scan

Publishes:
  /scan/transformed  (sensor_msgs/LaserScan)  — tilt-corrected horizontal
                                                distances, same angular
                                                structure as input
"""

import math
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanTransformNode(Node):

    def __init__(self):
        super().__init__('scan_transform_node')

        self.declare_parameter('world_frame',  'world')
        self.declare_parameter('laser_frame',  'laser_frame')

        self._world_frame = self.get_parameter('world_frame').value
        self._laser_frame = self.get_parameter('laser_frame').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            LaserScan, '/scan/filtered', self._scan_cb, 10)
        self._pub = self.create_publisher(
            LaserScan, '/scan/transformed', 10)

        self.get_logger().info(
            f'Scan transform ready: {self._laser_frame} → {self._world_frame}')

    def _scan_cb(self, msg: LaserScan):
        # Use Time(0) = latest available transform.
        # Single-machine setup so clock sync is not a concern; this avoids
        # ExtrapolationException when the scan timestamp is slightly ahead of TF.
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._laser_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        out = LaserScan()
        out.header          = msg.header
        out.header.frame_id = self._world_frame
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = []
        out.intensities     = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                out.ranges.append(float('inf'))
                if msg.intensities:
                    out.intensities.append(0.0)
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # Scan point in laser_frame (2-D scan plane, z = 0)
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            pz = 0.0

            # Active rotation: p' = q * p * q^-1  (Rodrigues formula)
            # t1 = 2 * (q_vec × p)
            t1x = 2.0 * (qy * pz - qz * py)
            t1y = 2.0 * (qz * px - qx * pz)
            t1z = 2.0 * (qx * py - qy * px)

            wx = px + qw * t1x + (qy * t1z - qz * t1y)
            wy = py + qw * t1y + (qz * t1x - qx * t1z)
            # wz not needed for horizontal distance

            # Translate to world origin and take XY horizontal distance.
            # Ignoring Z removes the apparent range change caused by tilt.
            horiz = math.sqrt((wx + tx) ** 2 + (wy + ty) ** 2)

            if horiz < msg.range_min or horiz > msg.range_max:
                out.ranges.append(float('inf'))
            else:
                out.ranges.append(horiz)

            if msg.intensities:
                out.intensities.append(msg.intensities[i])

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
