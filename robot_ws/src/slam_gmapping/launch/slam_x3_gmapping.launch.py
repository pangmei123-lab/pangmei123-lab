from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions
from launch_ros.actions import Node
import os
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([

        # ========== 1. 串口驱动：接收下位机回传的真实速度（必须最先启动） ==========
        Node(
            package='robot_control',
            executable='serial_driver',
            name='serial_driver',
            output='screen',
            emulate_tty=True,
        ),

        # ========== 2. 里程计发布器：发布 odom→base_link TF + /odom 话题 ==========
        Node(
            package='robot_control',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            emulate_tty=True,
        ),

        # ========== 3. 静态 TF: base_link → lidar_laser_sensor ==========
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments = ['0', '0.10', '0', '0', '0', '0', 
                         'base_link', 'lidar_laser_sensor'],
        ),

        # ========== 4. slam_gmapping 建图节点 ==========
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
        ),
        
        # ========== 5. rviz2 可视化 ==========
        Node(
            package='rviz2',
            namespace='map_rviz',
            executable='rviz2',
            output='screen',
            arguments = [ '-d', os.path.join(get_package_share_directory("slam_gmapping"), "rviz", "view_gmapping.rviz")],
        )
       
    ])
