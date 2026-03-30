from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="storm32_gimbal",
            namespace="gimbal",
            executable="gimbal",
            name="controller",
            output="screen",
            respawn=True,
            parameters=[
                {"frame_id": "gimbal_ref"},
                {"port": "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_48F3544B3835-if00"}
            ]
        )
    ])
