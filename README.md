# Four-Wheel Drive Robot Project

A ROS 2 based four-wheel drive robot project with SLAM, navigation, and serial communication capabilities.

## Project Structure

```
robot_ws/
├── src/
│   ├── robot_control/          # Robot control package (serial communication)
│   ├── my_robot_description/   # Robot URDF description
│   ├── slam_gmapping/          # SLAM GMapping package
│   ├── ydlidar_ros2_driver/    # YDLidar ROS 2 driver
│   └── openslam_gmapping/      # GMapping algorithm library
├── build/                      # Build outputs
├── install/                    # Installed packages
└── log/                        # Build logs
```

## Key Features

- **Serial Communication**: RDK X5 <-> STM32 binary protocol with XOR checksum, EMA filtering, and dead-zone handling
- **SLAM Mapping**: Using GMapping algorithm with YDLidar
- **Robot Description**: URDF/Xacro based robot model with wheel, laser sensor, and base components
- **Odometry**: Real velocity feedback from STM32 for accurate pose estimation

## Hardware

- **Main Controller**: RDK X5
- **Motor Controller**: STM32 (four-wheel drive)
- **Lidar**: YDLidar (X3 series)

## Serial Protocol

### TX (Host -> STM32): 16 bytes
| Byte | Content |
|------|---------|
| 0-1  | Header (0xAA 0x55) |
| 2    | Type (0x01 = speed cmd) |
| 3-6  | vx (float32, m/s) |
| 7-10 | vy (float32, m/s) |
| 11-14| wz (float32, rad/s) |
| 15   | XOR checksum |

### RX (STM32 -> Host): 16 bytes
| Byte | Content |
|------|---------|
| 0-1  | Header (0xAA 0x55) |
| 2    | Type (0x02 = real state) |
| 3-6  | vx (float32, m/s) |
| 7-10 | vy (float32, m/s) |
| 11-14| wz (float32, rad/s) |
| 15   | XOR checksum |

## Build & Run

```bash
cd robot_ws
colcon build
source install/setup.bash

# Launch robot control
ros2 launch robot_control control.launch.py

# Launch SLAM
ros2 launch slam_gmapping slam_x3_gmapping.launch.py

# Launch lidar
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py

# Launch robot description
ros2 launch my_robot_description display.launch.py
```

## Packages

| Package | Description | Language |
|---------|-------------|----------|
| robot_control | Serial communication, odometry, navigation | Python |
| my_robot_description | Robot URDF model | XML/Xacro |
| slam_gmapping | ROS 2 GMapping wrapper | C++ |
| ydlidar_ros2_driver | YDLidar driver | C++ |
| openslam_gmapping | GMapping core algorithm | C++ |

## License

Apache-2.0
