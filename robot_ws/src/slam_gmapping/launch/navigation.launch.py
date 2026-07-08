# ============================================================
# Navigation2 完整导航 —— 一键启动所有节点
# 用法: ros2 launch slam_gmapping navigation.launch.py
# ============================================================

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    MAP_YAML = '/home/sunrise/robot/robot_ws/maps/gmapping_map.yaml'
    NAV2_PARAMS = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'params', 'nav2_params.yaml'
    )

    # YDLIDAR 参数文件
    YDLIDAR_PARAMS = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params', 'ydlidar_x3.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument('map_file', default_value=MAP_YAML),

        # ========== 1. 底盘 + 里程计 ==========
        Node(package='robot_control', executable='serial_driver',
             name='serial_driver', output='screen', emulate_tty=True),
        Node(package='robot_control', executable='odom_publisher',
             name='odom_publisher', output='screen', emulate_tty=True),

        # ========== 2. YDLIDAR 激光雷达（提供 /scan）==========
        Node(package='ydlidar_ros2_driver',
             executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node',
             output='screen', emulate_tty=True,
             parameters=[YDLIDAR_PARAMS]),

        # ========== 3. 静态 TF: base_link → lidar_laser_sensor ==========
        Node(package='tf2_ros', executable='static_transform_publisher',
             output='screen',
             arguments=['0','0.10','0','0','0','0',
                        'base_link','lidar_laser_sensor']),

        # ========== 4. 地图服务 + AMCL 定位 ==========
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen', parameters=[NAV2_PARAMS]),

        # ========== 5. 路径规划 + 运动控制 ==========
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[NAV2_PARAMS]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=[NAV2_PARAMS]),

        # ========== 6. 极简导航器（替代 bt_navigator）==========
        Node(package='robot_control', executable='simple_navigator',
             name='simple_navigator', output='screen', emulate_tty=True),

        # ========== 7. Lifecycle 自动管理器 ==========
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{
                 'autostart': True,
                 'node_names': ['map_server', 'amcl',
                                'planner_server', 'controller_server'],
                 'bond_timeout': 5.0,
             }]),

        # ========== 8. RViz 可视化 ==========
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(
                 get_package_share_directory('slam_gmapping'),
                 'rviz', 'view_gmapping.rviz')]),
    ])
