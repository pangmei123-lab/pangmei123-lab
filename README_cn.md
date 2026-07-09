# 四轮驱动机器人项目

基于 ROS 2 的四轮驱动机器人项目，具备 SLAM 建图、导航和串口通信功能。

## 项目结构

```
robot_ws/
├── src/
│   ├── robot_control/          # 机器人控制包（串口通信）
│   ├── my_robot_description/   # 机器人 URDF 描述
│   ├── slam_gmapping/          # SLAM GMapping 包
│   ├── ydlidar_ros2_driver/    # YDLidar ROS 2 驱动
│   └── openslam_gmapping/      # GMapping 算法库
├── build/                      # 构建输出
├── install/                    # 安装的包
└── log/                        # 构建日志
```

## 主要功能

- **串口通信**: RDK X5 <-> STM32 二进制协议，支持 XOR 校验、EMA 滤波和死区处理
- **SLAM 建图**: 使用 GMapping 算法配合 YDLidar 激光雷达
- **机器人描述**: 基于 URDF/Xacro 的机器人模型，包含轮子、激光传感器和底盘组件
- **里程计**: 从 STM32 获取真实速度反馈，实现精确位姿估计

## 硬件配置

- **主控制器**: RDK X5
- **电机控制器**: STM32（四轮驱动）
- **激光雷达**: YDLidar（X3 系列）

## 串口协议

### TX（上位机 -> STM32）: 16 字节
| 字节 | 内容 |
|------|------|
| 0-1  | 帧头 (0xAA 0x55) |
| 2    | 类型 (0x01 = 速度指令) |
| 3-6  | vx (float32, m/s) |
| 7-10 | vy (float32, m/s) |
| 11-14| wz (float32, rad/s) |
| 15   | XOR 校验 |

### RX（STM32 -> 上位机）: 16 字节
| 字节 | 内容 |
|------|------|
| 0-1  | 帧头 (0xAA 0x55) |
| 2    | 类型 (0x02 = 真实状态) |
| 3-6  | vx (float32, m/s) |
| 7-10 | vy (float32, m/s) |
| 11-14| wz (float32, rad/s) |
| 15   | XOR 校验 |

## 构建与运行

```bash
cd robot_ws
colcon build
source install/setup.bash

# 启动机器人控制
ros2 launch robot_control control.launch.py

# 启动 SLAM 建图
ros2 launch slam_gmapping slam_x3_gmapping.launch.py

# 启动激光雷达
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py

# 启动机器人模型显示
ros2 launch my_robot_description display.launch.py
```

## 功能包说明

| 功能包 | 描述 | 语言 |
|--------|------|------|
| robot_control | 串口通信、里程计、导航 | Python |
| my_robot_description | 机器人 URDF 模型 | XML/Xacro |
| slam_gmapping | ROS 2 GMapping 封装 | C++ |
| ydlidar_ros2_driver | YDLidar 驱动 | C++ |
| openslam_gmapping | GMapping 核心算法 | C++ |

## 协议改进点（防飘移）

1. 下位机负责麦轮逆解和电机 PID，上位机只发送底盘速度指令
2. 接收真实底盘速度 vx, vy, wz，供里程计积分位姿
3. 帧同步 + XOR 校验，抗干扰、防错位
4. 50Hz 定时发送保证导航指令及时下发
5. EMA 低通滤波：平滑接收到的真实速度，减少噪声累积
6. 速度死区：微小速度清零，防止静止时漂移
7. 零速快速停止：键盘松开(cmd_vel=0)时立即发送停止指令
8. 数据超时保护：超过 0.5s 未收到真实速度则强制清零

## 许可证

Apache-2.0
