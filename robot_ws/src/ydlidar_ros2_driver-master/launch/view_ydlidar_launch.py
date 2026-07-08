#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # RViz 配置文件
    rviz_config_dir = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'config',
        'ydlidar.rviz'
    )

    # ====================== 修复 1：正确路径 + 正确文件名 ======================
    lidar_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'  # 这里修复了！
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path)
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        lidar_launch,
        rviz_node
    ])