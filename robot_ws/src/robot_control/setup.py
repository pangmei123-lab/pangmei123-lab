from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键：把 launch 文件安装到 ROS2 目录
        ('share/' + package_name + '/launch',
            ['launch/control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='fish@fishros.com',
    description='Robot control package for mecanum wheel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [

            'odom_publisher = robot_control.odom_publisher:main',
            'serial_driver = robot_control.serial_driver:main',
            'lifecycle_activator = robot_control.lifecycle_activator:main',
            'simple_navigator = robot_control.simple_navigator:main',
        ],
    },
)