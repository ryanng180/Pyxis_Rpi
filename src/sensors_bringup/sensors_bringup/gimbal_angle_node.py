#!/usr/bin/env python3
"""
Gimbal Angle Node
-----------------
Tracks the actual gimbal orientation from STorM32 IMU1 feedback and:
  1. Publishes actual yaw + pitch as a Float32MultiArray for logging and
     LiDAR calibration refinement.
  2. Broadcasts a dynamic TF:  base_link -> gimbal_camera_link
     so the camera pointing direction is visible in RViz and usable
     by any node that queries the TF tree.

The gimbal_camera_link frame represents where the camera is actually
pointing (from IMU1 feedback), not what was commanded. Comparing the
commanded angles from gimbal_tracker against this topic reveals the
self-correction applied by the STorM32 stabiliser.

Subscribes:
  /gimbal/control/camera_orientation  (geometry_msgs/QuaternionStamped)
    — IMU1 actual orientation published by storm32_node at 100 Hz

Publishes:
  /gimbal/actual_angles  (std_msgs/Float32MultiArray)
    — [yaw_deg, pitch_deg] actual gimbal angles from IMU1

TF:
  base_link -> gimbal_camera_link  (dynamic, updated from IMU1 feedback)

Usage:
    ros2 run sensors_bringup gimbal_angle_node
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class GimbalAngleNode(Node):

    def __init__(self):
        super().__init__('gimbal_angle_node')

        self.declare_parameter('camera_orientation_topic',
                               '/gimbal/control/camera_orientation')
        self.declare_parameter('base_frame',   'base_link')
        self.declare_parameter('gimbal_frame', 'gimbal_camera_link')

        self._base_frame   = self.get_parameter('base_frame').value
        self._gimbal_frame = self.get_parameter('gimbal_frame').value
        cam_topic          = self.get_parameter('camera_orientation_topic').value

        self._tf_broadcaster = TransformBroadcaster(self)

        self._pub_angles = self.create_publisher(
            Float32MultiArray, '/gimbal/actual_angles', 1)

        self.create_subscription(
            QuaternionStamped, cam_topic, self._orientation_cb, 1)

        self.get_logger().info(
            f'Gimbal angle node ready.\n'
            f'  Subscribing: {cam_topic}\n'
            f'  Publishing:  /gimbal/actual_angles  [yaw_deg, pitch_deg]\n'
            f'  TF:          {self._base_frame} -> {self._gimbal_frame}')

    def _orientation_cb(self, msg: QuaternionStamped):
        q = msg.quaternion

        # Extract yaw and pitch from the quaternion.
        # STorM32 IMU1 uses axes="syxz"; roll is actively stabilised near zero
        # so standard ZYX extraction is accurate (same approach as ladder_distance_node).
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Clamp argument of asin to [-1, 1] to guard against floating-point noise
        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        pitch_rad = math.asin(sin_pitch)

        yaw_deg   = math.degrees(yaw_rad)
        pitch_deg = math.degrees(pitch_rad)

        # Publish actual angles
        amsg = Float32MultiArray()
        amsg.data = [float(yaw_deg), float(pitch_deg)]
        self._pub_angles.publish(amsg)

        # Broadcast TF: base_link -> gimbal_camera_link
        # Translation is zero (gimbal origin assumed at base_link origin).
        # Tune the mount offset here once physical measurements are confirmed.
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self._base_frame
        t.child_frame_id  = self._gimbal_frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Re-encode the quaternion from the extracted yaw+pitch
        # (roll=0) so the TF is clean regardless of IMU1 roll noise.
        p = pitch_rad
        y = yaw_rad
        sp, cp = math.sin(p / 2), math.cos(p / 2)
        sy, cy = math.sin(y / 2), math.cos(y / 2)

        t.transform.rotation.x = -sy * sp
        t.transform.rotation.y =  cy * sp
        t.transform.rotation.z =  sy * cp
        t.transform.rotation.w =  cy * cp

        self._tf_broadcaster.sendTransform(t)

        self.get_logger().debug(
            f'gimbal actual  yaw={yaw_deg:+.2f}°  pitch={pitch_deg:+.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = GimbalAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
