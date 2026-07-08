#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
serial_driver.py  (二进制协议版)
功能：RDK X5 <-> STM32 四轮驱动串口通信

【二进制协议】
发送（上位机 → 下位机）：16 字节
  Byte 0-1 : 帧头  0xAA 0x55
  Byte 2   : 类型  0x01（速度指令）
  Byte 3-6 : vx    float32 (小端, m/s)
  Byte 7-10: vy    float32 (小端, m/s)
  Byte 11-14: wz   float32 (小端, rad/s)
  Byte 15  : XOR校验（Byte0 ~ Byte14 逐字节异或）

接收（下位机 → 上位机）：16 字节
  Byte 0-1 : 帧头  0xAA 0x55
  Byte 2   : 类型  0x02（真实状态回传）
  Byte 3-6 : vx    float32 (小端, m/s)
  Byte 7-10: vy    float32 (小端, m/s)
  Byte 11-14: wz   float32 (小端, rad/s)
  Byte 15  : XOR校验（Byte0 ~ Byte14 逐字节异或）

改进点（防飘移）：
  1. 下位机负责麦轮逆解和电机PID，上位机只发底盘速度
  2. 接收真实底盘速度 vx,vy,wz，供odom_publisher积分位姿
  3. 帧同步 + XOR校验，抗干扰、防错位
  4. 50Hz 定时发送保证导航指令及时下发
  5. EMA 低通滤波：平滑接收到的真实速度，减少噪声累积
  6. 速度死区：微小速度清零，防止静止时漂移
  7. 零速快速停止：键盘松开(cmd_vel=0)时立即发送停止指令
  8. 数据超时保护：超过0.5s未收到真实速度则强制清零
"""

import signal
import struct
import serial
import threading
import time
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ===================== 协议常量 =====================
FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55

TYPE_CMD_SPEED  = 0x01
TYPE_REAL_STATE = 0x02

TX_PACKET_SIZE = 16
RX_PACKET_SIZE = 16

DATA_TIMEOUT = 0.5

# ===================== 全局退出标志 =====================
exit_flag = False


def signal_handler(signal_num, frame):
    global exit_flag
    exit_flag = True


signal.signal(signal.SIGINT, signal_handler)


# ===================== 协议工具函数 =====================
def calc_xor(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result & 0xFF


def pack_tx_packet(vx: float, vy: float, wz: float) -> bytes:
    try:
        header_and_type = struct.pack('<BBB', FRAME_HEADER_0, FRAME_HEADER_1, TYPE_CMD_SPEED)
        data = struct.pack('<fff', vx, vy, wz)
        xor_val = calc_xor(header_and_type + data)
        return header_and_type + data + struct.pack('<B', xor_val)
    except Exception:
        return None


def unpack_rx_packet(packet: bytes):
    if len(packet) != RX_PACKET_SIZE:
        return None
    if packet[0] != FRAME_HEADER_0 or packet[1] != FRAME_HEADER_1:
        return None
    if packet[2] != TYPE_REAL_STATE:
        return None
    expected_xor = calc_xor(packet[:RX_PACKET_SIZE - 1])
    if expected_xor != packet[RX_PACKET_SIZE - 1]:
        return None
    try:
        vx, vy, wz = struct.unpack('<fff', packet[3:15])
        return (vx, vy, wz)
    except Exception:
        return None


class SerialDriver:

    def __init__(
        self,
        port='/dev/ttyS1',
        baudrate=115200,
        timeout=0.05,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = None
        self.running = True

        # ---- 接收：真实底盘速度 ----
        self.real_vx = 0.0
        self.real_vy = 0.0
        self.real_wz = 0.0

        # ---- EMA 低通滤波状态 ----
        self.filt_vx = 0.0
        self.filt_vy = 0.0
        self.filt_wz = 0.0
        self.ema_alpha = 0.3

        # ---- 发送：目标底盘速度 ----
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.prev_was_zero = True  # 上一次cmd_vel是否为零

        self.data_lock = threading.Lock()

        # ---- 统计 ----
        self.tx_count = 0
        self.tx_fail_count = 0
        self.rx_count = 0
        self.rx_fail_count = 0

        # ---- 时效检测 ----
        self.last_rx_time = 0.0
        self.last_tx_time = 0.0

        # ---- 调试 ----
        self.last_tx_vx = 0.0
        self.last_tx_vy = 0.0
        self.last_tx_wz = 0.0
        self.last_rx_vx = 0.0
        self.last_rx_vy = 0.0
        self.last_rx_wz = 0.0

        # 接收缓冲区（帧同步）
        self.rx_buffer = bytearray()

        self.connect()
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print("\n===================================")
            print("[Serial] 串口连接成功！（二进制协议 + EMA滤波 + 死区 + 零速快停）")
            print(f"端口: {self.port}  波特率: {self.baudrate}")
            print(f"TX包: {TX_PACKET_SIZE}字节  RX包: {RX_PACKET_SIZE}字节")
            print("===================================\n")
            return True
        except Exception as e:
            print(f"[Serial] 连接失败: {e}")
            return False

    # ===================== 更新目标速度 =====================
    def update_target_speed(self, vx, vy, wz):
        """cmd_vel回调：更新缓存。检测到零速时触发立即发送"""
        is_zero = (vx == 0.0 and vy == 0.0 and wz == 0.0)
        with self.data_lock:
            self.target_vx = vx
            self.target_vy = vy
            self.target_wz = wz

        # 零速快速停止：键盘松开时立即发送一次停止指令
        if is_zero and not self.prev_was_zero:
            self.send_chassis_speed()
        self.prev_was_zero = is_zero

    # ===================== 发送 =====================
    def send_chassis_speed(self):
        if not self.ser or not self.ser.is_open:
            self.tx_fail_count += 1
            return False

        with self.data_lock:
            vx = self.target_vx
            vy = self.target_vy
            wz = self.target_wz

        packet = pack_tx_packet(vx, vy, wz)
        if packet is None:
            self.tx_fail_count += 1
            return False

        try:
            self.ser.write(packet)
            self.tx_count += 1
            self.last_tx_time = time.time()
            self.last_tx_vx = vx
            self.last_tx_vy = vy
            self.last_tx_wz = wz
            return True
        except Exception:
            self.tx_fail_count += 1
            return False

    # ===================== 接收线程 =====================
    def receive_loop(self):
        global exit_flag
        while self.running and not exit_flag:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting)
                    if raw:
                        self.rx_buffer.extend(raw)
                        self._process_rx_buffer()
                else:
                    time.sleep(0.001)
            except Exception:
                time.sleep(0.01)

    def _process_rx_buffer(self):
        while len(self.rx_buffer) >= RX_PACKET_SIZE:
            header0_idx = self.rx_buffer.find(FRAME_HEADER_0)
            if header0_idx < 0:
                self.rx_buffer.clear()
                return
            if header0_idx > 0:
                del self.rx_buffer[:header0_idx]
            if len(self.rx_buffer) < RX_PACKET_SIZE:
                return
            if self.rx_buffer[1] != FRAME_HEADER_1:
                del self.rx_buffer[0]
                self.rx_fail_count += 1
                continue

            candidate = bytes(self.rx_buffer[:RX_PACKET_SIZE])
            result = unpack_rx_packet(candidate)
            if result is not None:
                vx, vy, wz = result

                # ---- 死区过滤 ----
                if abs(vx) < 0.005: vx = 0.0
                if abs(vy) < 0.005: vy = 0.0
                if abs(wz) < 0.002: wz = 0.0

                with self.data_lock:
                    # ---- EMA 低通滤波 ----
                    a = self.ema_alpha
                    self.filt_vx = a * vx + (1 - a) * self.filt_vx
                    self.filt_vy = a * vy + (1 - a) * self.filt_vy
                    self.filt_wz = a * wz + (1 - a) * self.filt_wz
                    self.real_vx = self.filt_vx
                    self.real_vy = self.filt_vy
                    self.real_wz = self.filt_wz
                self.rx_count += 1
                self.last_rx_time = time.time()
                self.last_rx_vx = vx
                self.last_rx_vy = vy
                self.last_rx_wz = wz
                del self.rx_buffer[:RX_PACKET_SIZE]
            else:
                del self.rx_buffer[0]
                self.rx_fail_count += 1

    # ===================== 查询接口 =====================
    def get_real_state(self):
        """获取真实状态。若超时未收到数据，强制清零"""
        with self.data_lock:
            # 数据超时保护：超过0.5s未收到新数据则清零速度
            if self.last_rx_time > 0.0 and (time.time() - self.last_rx_time) > DATA_TIMEOUT:
                self.real_vx = 0.0
                self.real_vy = 0.0
                self.real_wz = 0.0
                # 同时重置滤波器状态，避免残留
                self.filt_vx = 0.0
                self.filt_vy = 0.0
                self.filt_wz = 0.0
            return (self.real_vx, self.real_vy, self.real_wz)

    def is_data_fresh(self):
        if self.last_rx_time == 0.0:
            return False
        return (time.time() - self.last_rx_time) < DATA_TIMEOUT

    def get_stats(self):
        return {
            'tx_count': self.tx_count,
            'tx_fail': self.tx_fail_count,
            'rx_count': self.rx_count,
            'rx_fail': self.rx_fail_count,
            'data_fresh': self.is_data_fresh(),
        }

    def close(self):
        self.running = False
        if hasattr(self, 'recv_thread'):
            self.recv_thread.join(timeout=1)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("\n[Serial] 串口已安全关闭")
        except Exception:
            pass


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = Node('serial_driver')

    driver = SerialDriver(port='/dev/ttyS1', baudrate=115200)

    real_state_pub = node.create_publisher(Odometry, '/robot_real_state', 10)

    def cmd_vel_callback(msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        is_nonzero = abs(vx) > 0.001 or abs(vy) > 0.001 or abs(wz) > 0.001
        if is_nonzero:
            node.get_logger().info(
                f'[cmd_vel] 收到非零速度指令: '
                f'vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}'
            )
        driver.update_target_speed(vx, vy, wz)

    node.create_subscription(Twist, '/cmd_vel', cmd_vel_callback, 10)

    # ---- 50Hz 定时发送 ----
    def send_timer_callback():
        driver.send_chassis_speed()

    node.create_timer(0.02, send_timer_callback)

    # ---- 20Hz 发布真实状态 ----
    def publish_and_debug():
        vx, vy, wz = driver.get_real_state()
        fresh = driver.is_data_fresh()
        stats = driver.get_stats()

        real_msg = Odometry()
        real_msg.header.stamp = node.get_clock().now().to_msg()
        real_msg.header.frame_id = 'odom'
        real_msg.child_frame_id = 'base_link'
        real_msg.twist.twist.linear.x = vx
        real_msg.twist.twist.linear.y = vy
        real_msg.twist.twist.angular.z = wz
        real_msg.pose.pose.orientation.w = 1.0

        real_state_pub.publish(real_msg)

        status = 'OK' if fresh else 'LOST'
        node.get_logger().info(
            f'[串口] 状态:{status} | '
            f'TX→ vx={driver.last_tx_vx:.3f} vy={driver.last_tx_vy:.3f} wz={driver.last_tx_wz:.3f} | '
            f'RX← vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} | '
            f'计数 TX:{stats["tx_count"]}/{stats["tx_fail"]} '
            f'RX:{stats["rx_count"]}/{stats["rx_fail"]}'
        )

    node.create_timer(0.05, publish_and_debug)

    node.get_logger().info(
        'serial_driver 节点启动！（二进制协议+EMA+死区+零速快停+超时保护）'
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
