#!/usr/bin/env python3
"""
Tilt TF Node with gyro bias calibration
Publishes:
  - Static TF: base_link -> laser_frame (180 deg roll, upside-down correction)
  - Static TF: base_link -> imu_link
  - Dynamic TF: world -> base_link (live boat tilt from complementary filter)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import time


class TiltTFNode(Node):

    def __init__(self):
        super().__init__('tilt_tf_node')

        self.declare_parameter('alpha',           0.98)
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
        self.declare_parameter('calibration_samples', 100)

        self.alpha        = self.get_parameter('alpha').value
        self.world_frame  = self.get_parameter('world_frame').value
        self.base_frame   = self.get_parameter('base_frame').value
        self.laser_frame  = self.get_parameter('laser_frame').value
        self.cal_samples  = self.get_parameter('calibration_samples').value

        self.dyn_br    = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # Complementary filter state
        self.roll       = 0.0
        self.pitch      = 0.0
        self.last_time  = None
        self._log_count = 0

        # Gyro bias calibration
        self._calibrated   = False
        self._cal_buf_gx   = []
        self._cal_buf_gy   = []
        self._cal_buf_gz   = []
        self._gyro_bias_x  = 0.0
        self._gyro_bias_y  = 0.0
        self._gyro_bias_z  = 0.0

        self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 10)
        self.get_logger().info(
            f'Tilt TF node ready. Calibrating gyro bias '
            f'({self.cal_samples} samples) - keep unit still...')

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
        now = time.time()

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Data is in G and deg/s — convert to m/s² and rad/s
        if abs(az) < 3.0:
            ax *= 9.80665
            ay *= 9.80665
            az *= 9.80665
            gx = math.radians(gx)
            gy = math.radians(gy)
            gz = math.radians(gz)

        # ── Gyro bias calibration on startup ─────────────────────────
        if not self._calibrated:
            self._cal_buf_gx.append(gx)
            self._cal_buf_gy.append(gy)
            self._cal_buf_gz.append(gz)

            if len(self._cal_buf_gx) >= self.cal_samples:
                self._gyro_bias_x = sum(self._cal_buf_gx) / len(self._cal_buf_gx)
                self._gyro_bias_y = sum(self._cal_buf_gy) / len(self._cal_buf_gy)
                self._gyro_bias_z = sum(self._cal_buf_gz) / len(self._cal_buf_gz)
                self._calibrated  = True
                self.get_logger().info(
                    f'Gyro bias calibrated: '
                    f'gx={math.degrees(self._gyro_bias_x):.3f}°/s  '
                    f'gy={math.degrees(self._gyro_bias_y):.3f}°/s  '
                    f'gz={math.degrees(self._gyro_bias_z):.3f}°/s')
            return

        # ── Apply gyro bias correction ────────────────────────────────
        gx -= self._gyro_bias_x
        gy -= self._gyro_bias_y

        # ── Accelerometer angle estimate ──────────────────────────────
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        if accel_mag < 1.0:
            return

        accel_roll  = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # ── Complementary filter ──────────────────────────────────────
        if self.last_time is None:
            self.roll      = accel_roll
            self.pitch     = accel_pitch
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now
        if dt <= 0 or dt > 0.5:
            return

        self.roll  = self.alpha * (self.roll  + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch

        self._publish_dynamic_tf()

        self._log_count += 1
        if self._log_count % 200 == 0:
            self.get_logger().info(
                f'Boat tilt  roll={math.degrees(self.roll):.2f}°  '
                f'pitch={math.degrees(self.pitch):.2f}°')

    def _publish_dynamic_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id  = self.base_frame

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        cr = math.cos(self.roll  * 0.5)
        sr = math.sin(self.roll  * 0.5)
        cp = math.cos(self.pitch * 0.5)
        sp = math.sin(self.pitch * 0.5)

        t.transform.rotation.w =  cr * cp
        t.transform.rotation.x =  sr * cp
        t.transform.rotation.y =  cr * sp
        t.transform.rotation.z = -sr * sp

        self.dyn_br.sendTransform(t)


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
