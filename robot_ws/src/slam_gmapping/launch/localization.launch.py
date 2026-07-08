# ============================================================
# AMCL 定位 Launch 文件
# 用法：ros2 launch slam_gmapping localization.launch.py
# ============================================================

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')

    # ========== 地图文件（硬编码绝对路径）==========
    MAP_YAML = '/home/sunrise/robot/robot_ws/maps/gmapping_map.yaml'

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=MAP_YAML,
        description='Full path to map yaml file'
    )

    return LaunchDescription([

        map_file_arg,

        # ====================================================
        # 1. 串口驱动
        # ====================================================
        Node(
            package='robot_control',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            emulate_tty=True,
        ),

        # ====================================================
        # 2. 里程计发布器（odom → base_link）
        # ====================================================
        Node(
            package='robot_control',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            emulate_tty=True,
        ),

        # ====================================================
        # 3. 静态 TF: base_link → lidar_laser_sensor
        # ====================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0.10', '0', '0', '0', '0',
                       'base_link', 'lidar_laser_sensor'],
        ),

        # ====================================================
        # 4. map_server（加载 pgm 地图）
        # ====================================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'use_sim_time': use_sim_time,
            }],
        ),

        # ====================================================
        # 5. AMCL（发布 map → odom）
        # ====================================================
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('slam_gmapping'),
                    'params', 'amcl_params.yaml'
                ),
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ====================================================
        # 6. lifecycle_activator（激活 map_server + amcl）
        # ====================================================
        Node(
            package='robot_control',
            executable='lifecycle_activator',
            name='lifecycle_activator',
            output='screen',
            emulate_tty=True,
        ),

        # ====================================================
        # 7. RViz（复用建图时的配置）
        # ====================================================
        Node(
            package='rviz2',
            namespace='map_rviz',
            executable='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(
                    get_package_share_directory('slam_gmapping'),
                    'rviz', 'view_gmapping.rviz'
                )
            ],
        ),

    ])
