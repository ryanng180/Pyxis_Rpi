from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── RPLIDAR C1 ────────────────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port':      '/dev/rplidar',
                'serial_baudrate':  460800,
                'frame_id':         'laser_frame',
                'angle_compensate': True,
                'scan_mode':        'Standard',
            }]
        ),

        # ── IMU Node ──────────────────────────────────────────────────
        Node(
            package='sensors_bringup',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'port':     '/dev/nano33ble',
                'baud':     115200,
                'frame_id': 'imu_link',
            }]
        ),

        # ── Tilt TF Node ──────────────────────────────────────────────
        # Corrects upside-down mount (180 roll) + live boat tilt from IMU
        # UPDATE lidar_x/y/z and imu_x/y/z after measuring on boat
        Node(
            package='sensors_bringup',
            executable='tilt_tf_node',
            name='tilt_tf_node',
            output='screen',
            parameters=[{
                'alpha':       0.98,
                'world_frame': 'world',
                'base_frame':  'base_link',
                'imu_frame':   'imu_link',
                'laser_frame': 'laser_frame',
                'imu_x':   0.0,
                'imu_y':   0.0,
                'imu_z':   0.05,
                'lidar_x': 0.0,
                'lidar_y': 0.0,
                'lidar_z': 0.1,
            }]
        ),

        # ── Scan Filter Node ──────────────────────────────────────────
        # Masks cabin-facing arc, keeps starboard-facing sea arc
        # TUNE outward_min_deg/outward_max_deg after install on boat
        # Check raw /scan in RViz to find which angles hit the cabin
        Node(
            package='sensors_bringup',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'outward_min_deg': -180.0,  # tune after install
                'outward_max_deg':    0.0,  # tune after install
                'min_range':  0.3,
                'max_range': 16.0,
            }]
        ),

        # ── Proximity Node ────────────────────────────────────────────
        # Closest distance + SAFE/CAUTION/DANGER zone for dashboard
        Node(
            package='sensors_bringup',
            executable='proximity_node',
            name='proximity_node',
            output='screen',
            parameters=[{
                'danger_m':  3.0,
                'caution_m': 10.0,
            }]
        ),

    ])
