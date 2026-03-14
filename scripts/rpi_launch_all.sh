#!/bin/bash

echo "Sourcing ROS 2 environment..."
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Launching RPi system components..."

# Terminal 1: LiDAR + IMU launch
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Launching sensors_bringup...'
ros2 launch sensors_bringup sensors_launch.py
exec bash
"

# Terminal 2: Proximity status echo
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Monitoring /proximity/status...'
ros2 topic echo /proximity/status
exec bash
"

# Terminal 3: RViz
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Launching RViz2...'
ros2 run rviz2 rviz2
exec bash
"

# Terminal 4: Storm32 gimbal launch
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Launching storm32 gimbal...'
ros2 launch storm32_gimbal gimbal_launch.py
exec bash
"

# Terminal 5: Gimbal tracker
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Launching gimbal tracker...'
ros2 run gimbal_tracker gimbal_tracker
exec bash
"

# Terminal 6: Target orientation echo
gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
echo 'Monitoring /gimbal/controller/target_orientation...'
ros2 topic echo /gimbal/controller/target_orientation
exec bash
"

echo "All RPi terminals launched."
