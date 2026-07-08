from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ===================== 1. 里程计发布器 =====================
        Node(
            package='robot_control',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            emulate_tty=True
        ),

        # ===================== 2. 串口驱动 =====================
        Node(
            package='robot_control',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            emulate_tty=True
        )
    ])

    