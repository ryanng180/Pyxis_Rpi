#!/usr/bin/env python3
"""
Tilt TF Node
Publishes:
  - Static TF: base_link -> laser_frame (180 deg roll, upside-down correction)
  - Static TF: base_link -> imu_link
  - Dynamic TF: world -> base_link (orientation from imu_filter_madgwick)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TiltTFNode(Node):

    def __init__(self):
        super().__init__('tilt_tf_node')

        self.declare_parameter('world_frame',     'world')
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('imu_frame',       'imu_link')
        self.declare_parameter('laser_frame',     'laser_frame')
        self.declare_parameter('lidar_x',          0.0)
        self.declare_parameter('lidar_y',          0.0)
        self.declare_parameter('lidar_z',          0.1)
        self.declare_parameter('imu_x',            0.0)
        self.declare_parameter('imu_y',            0.0)
        self.declare_parameter('imu_z',            0.05)

        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value

        self.dyn_br    = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        self._log_count = 0

        # Subscribe to Madgwick-filtered IMU (has orientation quaternion)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.get_logger().info('Tilt TF node ready (Madgwick orientation).')

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        # base_link -> imu_link
        t = TransformStamped()
        t.header.stamp            = now
        t.header.frame_id         = self.base_frame
        t.child_frame_id          = self.get_parameter('imu_frame').value
        t.transform.translation.x = self.get_parameter('imu_x').value
        t.transform.translation.y = self.get_parameter('imu_y').value
        t.transform.translation.z = self.get_parameter('imu_z').value
        t.transform.rotation.x    = 0.0
        t.transform.rotation.y    = 0.0
        t.transform.rotation.z    = 0.0
        t.transform.rotation.w    = 1.0
        tfs.append(t)

        # base_link -> laser_frame (180 deg roll for upside-down mount)
        t2 = TransformStamped()
        t2.header.stamp            = now
        t2.header.frame_id         = self.base_frame
        t2.child_frame_id          = self.laser_frame
        t2.transform.translation.x = self.get_parameter('lidar_x').value
        t2.transform.translation.y = self.get_parameter('lidar_y').value
        t2.transform.translation.z = self.get_parameter('lidar_z').value
        t2.transform.rotation.x    = 0.0
        t2.transform.rotation.y    = 0.0
        t2.transform.rotation.z    = 1.0
        t2.transform.rotation.w    = 0.0
        tfs.append(t2)

        self.static_br.sendTransform(tfs)
        self.get_logger().info(
            'Static TFs published: base_link->imu_link, '
            'base_link->laser_frame (180 roll)')

    def _imu_cb(self, msg: Imu):
        # Strip yaw from Madgwick quaternion — we only need pitch/roll
        # for tilt correction. Without a magnetometer, yaw drifts
        # continuously, causing /filtered to rotate in RViz.
        q = msg.orientation
        # Extract yaw using ZYX Euler convention
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Build a yaw-only quaternion and apply its inverse to remove yaw
        half_yaw = -yaw / 2.0
        qy_inv_z = math.sin(half_yaw)
        qy_inv_w = math.cos(half_yaw)

        # q_stripped = q_yaw_inv * q_original  (quaternion multiply)
        ow = qy_inv_w * q.w - qy_inv_z * q.z
        ox = qy_inv_w * q.x - qy_inv_z * q.y
        oy = qy_inv_w * q.y + qy_inv_z * q.x
        oz = qy_inv_w * q.z + qy_inv_z * q.w

        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id  = self.base_frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = ox
        t.transform.rotation.y = oy
        t.transform.rotation.z = oz
        t.transform.rotation.w = ow

        self.dyn_br.sendTransform(t)

        self._log_count += 1
        if self._log_count % 200 == 0:
            q = msg.orientation
            # Extract roll/pitch for logging only
            roll  = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0,
                               2.0 * (q.w * q.y - q.z * q.x))))
            self.get_logger().info(
                f'Boat tilt  roll={math.degrees(roll):.2f}°  '
                f'pitch={math.degrees(pitch):.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = TiltTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
