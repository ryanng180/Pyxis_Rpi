#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('port',     '/dev/nano33ble')
        self.declare_parameter('baud',     115200)
        self.declare_parameter('frame_id', 'imu_link')

        self.port     = self.get_parameter('port').value
        self.baud     = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.ser = None
        self._connect()
        self.create_timer(0.001, self._read)

    def _connect(self):
        while True:
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=1)
                self.ser.flushInput()
                time.sleep(2)
                self.get_logger().info(f'Connected to {self.port} - reading data...')
                return
            except serial.SerialException as e:
                self.get_logger().warn(f'Cannot open {self.port}: {e} - retrying in 3s')
                time.sleep(3)

    def _read(self):
        if not self.ser or not self.ser.is_open:
            self._connect()
            return
        try:
            if self.ser.in_waiting == 0:
                return

            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or 'READY' in line or 'ERROR' in line:
                return

            parts = line.split(',')
            if len(parts) != 6:
                return

            ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
            gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])

            msg = Imu()
            msg.header.stamp                       = self.get_clock().now().to_msg()
            msg.header.frame_id                    = self.frame_id
            msg.linear_acceleration.x              = ax
            msg.linear_acceleration.y              = ay
            msg.linear_acceleration.z              = az
            msg.angular_velocity.x                 = gx
            msg.angular_velocity.y                 = gy
            msg.angular_velocity.z                 = gz
            msg.orientation_covariance[0]          = -1.0
            msg.angular_velocity_covariance[0]     = 0.01
            msg.linear_acceleration_covariance[0]  = 0.1
            self.pub.publish(msg)

        except (ValueError, UnicodeDecodeError):
            pass
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}')
            self.ser.close()
            self._connect()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()


if __name__ == '__main__':
    main()
