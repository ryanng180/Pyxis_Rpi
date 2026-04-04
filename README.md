# Pyxis_Rpi

ROS2 workspace for the **Pyxis Maritime Pilot Transfer System**, running on a Raspberry Pi 5 (ROS2 Jazzy).

## Build

```bash
cd ~/ros2_ws

# Build arcros_interface first (other packages depend on its custom messages)
colcon build --packages-select arcros_interface
source install/setup.bash

# Build everything
colcon build
source install/setup.bash
```

> Always run `source install/setup.bash` after building.

## Run

The full system (ROS2 nodes + Jetson inference + dashboard) is launched with the unified startup script:

```bash
~/pyxis_startup.sh              # start everything
~/pyxis_startup.sh --no-rviz    # skip RViz (saves ~18% CPU on RPi5)
```

Ctrl+C gracefully shuts down all services in reverse order.

### Launch subsystems individually

```bash
# Sensor pipeline (LiDAR, IMU, proximity, ladder distance, cargo approach)
ros2 launch sensors_bringup sensors_launch.py

# Gimbal driver
ros2 launch storm32_gimbal gimbal_launch.py

# Gimbal tracker (PID control from Jetson detections)
ros2 run gimbal_tracker gimbal_tracker_pitch_yaw
```

## Packages

| Package | Description |
|---|---|
| `arcros_interface` | Custom ROS2 messages (`GimbalOrientation.msg`) — build first |
| `sensors_bringup` | Sensor pipeline: IMU, LiDAR filtering, tilt correction, proximity, ladder distance, cargo approach |
| `gimbal_tracker` | PID yaw/pitch control from Jetson YOLO detections |
| `storm32_gimbal` | STorM32 gimbal serial driver |
| `sllidar_ros2` | C++ driver for RPLIDAR C1 |

## Network

Pi and Jetson communicate over a direct CAT 5e link with static IPs:

| Device | IP |
|---|---|
| Raspberry Pi 5 | `10.42.0.2` |
| Jetson Orin Nano | `10.42.0.1` |
