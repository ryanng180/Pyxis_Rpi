# Pyxis_Rpi

ROS2 workspace running on the **Raspberry Pi 5** for the **Pyxis Maritime Pilot Transfer System**.

The Raspberry Pi coordinates sensing and actuation components used to detect and stabilise a maritime pilot ladder during transfer operations.

---

# System Overview

System architecture:

Jetson (YOLO ladder detection)
        ↓ UDP
Raspberry Pi 5
        ↓
Gimbal tracker
        ↓
Storm32 gimbal control
        ↓
LiDAR + IMU sensing

The Jetson device performs computer vision inference and sends ladder detections to the Raspberry Pi via UDP.

The Raspberry Pi handles gimbal control and sensor processing.

---

# Repository Structure

## src/gimbal_tracker
Controls the **Storm32 gimbal yaw tracking**.

Receives ladder detections from the Jetson and converts the detected ladder position into gimbal orientation commands.

Main script:
gimbal_tracker_yaw.py

Responsibilities:
- Convert bounding box coordinates to yaw angles
- Publish ROS2 commands to the gimbal
- Stabilise camera tracking of the pilot ladder

---

## src/sllidar_ros2
ROS2 driver for the **SLLiDAR sensor**.

Provides LiDAR scan data used to measure distance to the pilot ladder and surrounding environment.

Used for:
- ladder distance estimation
- obstacle awareness

---

## src/sensors_bringup
Handles **sensor nodes and filtering**.

Includes nodes for:
- IMU
- proximity sensors
- LiDAR scan filtering
- tilt transform publishing

These nodes provide cleaned sensor data to the rest of the system.

---

## src/arcros_interface
Defines **custom ROS2 message interfaces** used by the system.

Example message:
GimbalOrientation.msg

Used for communication between ROS2 nodes.

---

# Utility Script

## udp_recv_eth.py

This script is **not required for normal system integration**.

It is used for **isolated communication testing** to verify that the Jetson device can successfully send UDP detection packets to the Raspberry Pi.

Typical use cases:
- debugging network communication
- verifying inference packet transmission
- validating UDP connectivity between Jetson and Pi

---

# ROS2 Workspace Layout

ros2_ws/
 ├── src/
 │   ├── gimbal_tracker
 │   ├── sllidar_ros2
 │   ├── sensors_bringup
 │   └── arcros_interface
 ├── build/
 ├── install/
 └── log/

Generated folders (build, install, log) are ignored by git.

---

# Build Instructions

cd ~/ros2_ws  
colcon build  
source install/setup.bash

---

# Running the System

## Step 1 — Configure Network

Configure the Ethernet connection between the **Jetson** and **Raspberry Pi**.

### Jetson

```bash
sudo ./scripts/jetson_net_setup.sh
```

### Raspberry Pi

```bash
sudo ./scripts/rpi_net_setup.sh
```

---

## Step 2 — Start Raspberry Pi ROS2 Stack

Launch the ROS2 nodes responsible for:

- gimbal control
- sensor drivers
- LiDAR processing

```bash
./scripts/rpi_launch_all.sh
```

---

## Step 3 — Start Jetson Inference

Launch the computer vision pipeline on the Jetson.

This starts:

- camera capture
- YOLO ladder detection
- UDP packet transmission to the Raspberry Pi

```bash
./scripts/jetson_launch_all.sh
```

---

## System Startup Order

For correct operation, follow this order:

1. Configure network on both devices  
2. Start Raspberry Pi ROS2 stack  
3. Start Jetson inference pipeline

# Project Context

This repository is part of the **Pyxis Maritime Pilot Transfer Project**, which aims to improve the safety of maritime pilot boarding operations through:

- computer vision
- sensor fusion
- active camera stabilisation
