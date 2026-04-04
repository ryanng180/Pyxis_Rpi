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
                'inverted':         True,    # LiDAR mounted upside-down
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

        # ── Madgwick IMU Filter ───────────────────────────────────────
        # Fuses accel + gyro into orientation quaternion with online
        # gyro bias estimation. Publishes /imu/data with orientation.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag':            False,   # no magnetometer published
                'publish_tf':         False,   # tilt_tf_node owns TF
                'world_frame':        'enu',
                'fixed_frame':        'world',
                'gain':               0.1,     # Madgwick beta, tune later
                'frequency':          0.0,     # use data rate from topic
                'orientation_stddev': 0.01,
            }],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/data',     '/imu/data'),
            ],
        ),

        # ── Tilt TF Node ──────────────────────────────────────────────
        # Static TFs (mount corrections) + dynamic world→base_link from
        # Madgwick orientation. UPDATE lidar_x/y/z and imu_x/y/z on boat.
        Node(
            package='sensors_bringup',
            executable='tilt_tf_node',
            name='tilt_tf_node',
            output='screen',
            parameters=[{
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
        # Z-down mount: starboard is positive. 0°=fwd, 90°=stbd, 180°=aft
        # TUNE outward_min_deg/outward_max_deg after install on boat
        Node(
            package='sensors_bringup',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'outward_min_deg':  80.0,    # tabletop testing
                'outward_max_deg': 100.0,   # tabletop testing
                'min_range':  0.3,
                'max_range': 16.0,
            }]
        ),

        # ── Scan Transform Node ───────────────────────────────────────
        # Transforms /scan/filtered into world frame using TF chain:
        #   laser_frame → base_link (static 180° roll mount correction)
        #               → world    (live boat pitch/roll from IMU)
        # Publishes /scan/transformed with true horizontal distances.
        Node(
            package='sensors_bringup',
            executable='scan_transform_node',
            name='scan_transform_node',
            output='screen',
            parameters=[{
                'world_frame': 'world',
                'laser_frame': 'laser_frame',
            }]
        ),

        # ── Ladder Distance Node ──────────────────────────────────────
        # Parallax-corrected ladder distance using actual gimbal IMU1 angle
        # (not the lagging commanded angle) + tilt-corrected LiDAR range.
        # Publishes /ladder/distance and /ladder/status for dashboard.
        Node(
            package='sensors_bringup',
            executable='ladder_distance_node',
            name='ladder_distance_node',
            output='screen',
            parameters=[{
                'gimbal_orientation_topic': '/gimbal/controller/camera_orientation',
                'scan_topic':               '/scan/transformed',
                'gimbal_yaw_offset_deg':    72.0,   # TEMP: manual offset for this session (drifts each startup without calibration)
            }]
        ),

        # ── Proximity Node ────────────────────────────────────────────
        # Hull-to-cargo distance + TOO_CLOSE/OPTIMAL/TOO_FAR zone
        Node(
            package='sensors_bringup',
            executable='proximity_node',
            name='proximity_node',
            output='screen',
            parameters=[{
                'hull_offset_m': 1.077,   # LiDAR-to-hull perpendicular distance
                'too_close_m':   0.3,     # placeholder — tune during sea trials
                'too_far_m':     1.0,     # placeholder — tune during sea trials
                'scan_topic': '/scan/transformed',  # tilt-corrected; use /scan/filtered for tabletop testing
            }]
        ),

        # ── Cargo Approach Node ───────────────────────────────────────
        # Detects cargo hull line in LiDAR scan, computes heading error
        # and distance profile along the ferry's starboard hull.
        Node(
            package='sensors_bringup',
            executable='cargo_approach_node',
            name='cargo_approach_node',
            output='screen',
            parameters=[{
                'hull_starboard_m': 1.123,     # ferry hull at 1.123m starboard
                'hull_bow_x':       0.987,     # bow transition (ROS x)
                'hull_rear_x':     -12.066,    # rear of hull (ROS x)
                'lidar_x':         -0.01161,   # LiDAR mount position (ROS x)
                'lidar_y':         -0.0457,    # LiDAR mount position (ROS y)
                'profile_step_m':   1.0,       # sample every 1m along hull
                'too_close_m':      0.3,       # placeholder
                'too_far_m':        1.0,       # placeholder
                'min_points':       10,
                'min_r_squared':    0.7,
            }]
        ),

        # ── Phase Manager Node ────────────────────────────────────────
        # Observes proximity data, determines operational phase with
        # debounced transitions. Publishes /system/phase for dashboard
        # and gimbal tracker. Pure observer — safe to add/remove.
        Node(
            package='sensors_bringup',
            executable='phase_manager_node',
            name='phase_manager_node',
            output='screen',
        ),

        # ── Gimbal Angle Node ─────────────────────────────────────────
        # Tracks actual gimbal orientation from STorM32 IMU1 feedback.
        # Publishes /gimbal/actual_angles [yaw_deg, pitch_deg] and
        # dynamic TF base_link -> gimbal_camera_link for RViz and LiDAR
        # calibration refinement.
        Node(
            package='sensors_bringup',
            executable='gimbal_angle_node',
            name='gimbal_angle_node',
            output='screen',
            parameters=[{
                'camera_orientation_topic': '/gimbal/controller/camera_orientation',
                'base_frame':   'base_link',
                'gimbal_frame': 'gimbal_camera_link',
            }]
        ),

    ])
