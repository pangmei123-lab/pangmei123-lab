from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取环境变量
    LIDAR_TYPE = os.getenv('LIDAR_TYPE', 'x3')
    print("my_lidar:", LIDAR_TYPE)

    lidar_type_arg = DeclareLaunchArgument(
        name='lidar_type', 
        default_value=LIDAR_TYPE, 
        description='The type of lidar'
    )

    # ====================== 修复路径拼接 ======================
    # X3 雷达启动文件
    x3_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'x3_ydlidar_launch.py'
    )

    # 4ros 雷达启动文件
    ros4_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        '4ros_ydlidar_launch.py'
    )

    lidar_x3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x3_launch_path),
        condition=LaunchConfigurationEquals('lidar_type', 'x3')
    )

    lidar_4ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros4_launch_path),
        condition=LaunchConfigurationEquals('lidar_type', '4ros')
    )

    return LaunchDescription([
        lidar_type_arg,
        lidar_x3_launch,
        lidar_4ros_launch
    ])