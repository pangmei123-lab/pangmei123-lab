#!/usr/bin/env python3
"""
Launch file to publish robot_description from a xacro/urdf and start RViz for visualization.

Usage: ros2 launch my_robot_description display.launch.py

This file expects the package to contain:
 - urdf/<your_robot>.urdf.xacro  (default name: robot.urdf.xacro)
 - rviz/display.rviz  (optional, rviz will start with this config if present)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description')

    default_urdf_xacro = PathJoinSubstitution([pkg_share, 'urdf', 'robot_base.urdf.xacro'])
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    urdf_xacro_arg = DeclareLaunchArgument(
        name='xacro_file',
        default_value=default_urdf_xacro,
        description='Absolute path to robot xacro file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config,
        description='Absolute path to rviz config file'
    )

    robot_description_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        LaunchConfiguration('xacro_file')
    ])

    

    # 可选：添加仿真时间参数（通用规范）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description_cmd, value_type=str)},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        urdf_xacro_arg,
        rviz_config_arg,
        use_sim_time_arg,
        joint_state_publisher_node, 
        robot_state_publisher_node,
        rviz_node,
    ])
    