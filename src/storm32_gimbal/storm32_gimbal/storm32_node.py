import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf_transformations as tf
import numpy as np
import serial

import diagnostic_updater
import diagnostic_msgs.msg

from .storm32 import Storm32

from arcros_interface.msg import GimbalOrientation
from std_srvs.srv import Trigger
from geometry_msgs.msg import QuaternionStamped


class Storm32Node(Node):

    def __init__(self):

        super().__init__("control")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("frame_id", "gimbal_ref")

        self.port = self.get_parameter("port").value
        self.frame_id = self.get_parameter("frame_id").value

        # Connect to gimbal
        self.gimbal = Storm32(port=self.port)
        version = self.gimbal.get_version()

        if not version:
            self.get_logger().fatal(f"Gimbal unresponsive at {self.port}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Gimbal found at {self.port}")

        for k, v in version.items():
            self.get_logger().info(f"{k}: {v}")

        # Diagnostics
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(self.frame_id)
        self.updater.add("Gimbal Diagnostics", self.get_diagnostics_status)

        # Subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            GimbalOrientation,
            "~/target_orientation",
            self.gimbal_quaternion_callback,
            qos,
        )

        # Publishers
        self.camera_pub = self.create_publisher(
            QuaternionStamped,
            "~/camera_orientation",
            1
        )

        self.controller_pub = self.create_publisher(
            QuaternionStamped,
            "~/controller_orientation",
            1
        )

        # Restart service
        self.restart_srv = self.create_service(
            Trigger,
            "~/restart",
            self.restart_controller
        )

        # Timer
        self.pub_timer = self.create_timer(
            0.01,
            self.pub_timer_callback
        )

    def encode_angle(self, x):

        while x > np.pi:
            x -= 2 * np.pi

        while x < -np.pi:
            x += 2 * np.pi

        return x * 180 / np.pi

    def gimbal_quaternion_callback(self, msg):

        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        euler = tf.euler_from_quaternion(quaternion, axes="syxz")
        euler = list(map(self.encode_angle, euler))

        self.get_logger().info(
            f"set_angles : pitch={euler[0]}, roll={euler[1]}, yaw={euler[2]}, unlimited={msg.unlimited}"
        )

        try:

            response = self.gimbal.set_angles(
                euler[0],
                euler[1],
                euler[2],
                unlimited=msg.unlimited
            )

            if response != [1, 150, 0]:
                self.get_logger().error(
                    f"STorM32 response error: {response}"
                )

        except serial.serialutil.SerialException as e:

            self.get_logger().fatal(str(e))
            rclpy.shutdown()

    def get_diagnostics_status(self, stat):

        status = self.gimbal.get_status()

        if status:

            for k, v in status.items():
                stat.add(str(k), str(v))

            state = status.get("State", "Unknown")

            if state == "Normal":
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, state)
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, state)

        return stat

    def restart_controller(self, request, response):

        try:

            success = self.gimbal.restart_controller()

            response.success = success

            if success:
                response.message = "Gimbal restarted successfully"
            else:
                response.message = "Gimbal restart failed"

            return response

        except serial.serialutil.SerialException as e:

            self.get_logger().fatal(str(e))
            rclpy.shutdown()

    def pub_timer_callback(self):

        msg = QuaternionStamped()

        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        try:

            imu1 = (self.gimbal.get_imu1_angles(), self.camera_pub, "imu1")
            imu2 = (self.gimbal.get_imu2_angles(), self.controller_pub, "imu2")

            for euler, pub, name in (imu1, imu2):

                if not euler or len(euler) < 3:
                    continue

                self.get_logger().debug(
                    f"{name} angles: pitch={euler[0]}, roll={euler[1]}, yaw={euler[2]}"
                )

                euler = list(map(np.radians, euler))

                q = tf.quaternion_from_euler(
                    euler[0],
                    euler[1],
                    euler[2],
                    axes="syxz"
                )

                msg.quaternion.x = q[0]
                msg.quaternion.y = q[1]
                msg.quaternion.z = q[2]
                msg.quaternion.w = q[3]

                pub.publish(msg)

            self.updater.update()

        except serial.serialutil.SerialException as e:

            self.get_logger().fatal(str(e))
            rclpy.shutdown()


def main():

    rclpy.init()

    node = Storm32Node()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
