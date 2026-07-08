import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定义 lidar_type 参数，默认值为 'x3'
    lidar_type_arg = DeclareLaunchArgument(
        name='lidar_type',
        default_value='x3',  # 如果环境变量未设置，使用默认值
        description='The type of lidar'
    )

    # 包含 ydlidar_ros2_driver 的启动文件，并传递 lidar_type 参数
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch', 'ydlidar_launch.py'
            )
        ),
        launch_arguments={'lidar_type': LaunchConfiguration('lidar_type')}.items()
    )

    # 根据 lidar_type 参数选择不同的 SLAM 启动文件
    slam_4ros_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_gmapping'),
                'launch', 'slam_4ros_gmapping.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('lidar_type', '4ros')
    )

    slam_x3_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_gmapping'),
                'launch', 'slam_x3_gmapping.launch.py'
            )
        ),
        condition=LaunchConfigurationNotEquals('lidar_type', '4ros')
    )

    return LaunchDescription([
        lidar_type_arg,
        lidar_launch,
        slam_4ros_gmapping_launch,
        slam_x3_gmapping_launch
    ])