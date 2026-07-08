# ============================================
# 文件名：odom_publisher.py
# 功能：订阅 /robot_real_state（来自 serial_driver 的真实 vx,vy,wz），发布：
#       1. /odom                  里程计话题
#       2. odom -> base_link      TF坐标变换
# 说明：下位机只返回底盘真实速度 vx,vy,wz，
#       yaw 角由上位机通过 wz 积分得到
#
# 改进点（防飘移）：
#   1. 中点积分：mid_yaw = old_yaw + wz*dt/2
#   2. 速度死区：低于阈值视为零
#   3. EMA低通滤波：平滑原始速度
#   4. 时间戳驱动dt：实际时间差，补偿抖动
#   5. 静止锁定：连续静止>0.5s锁定，杜绝微漂
#   6. 动态协方差：随行进距离增长
# ============================================

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')

        # ===== 1. 订阅真实底盘状态 =====
        self.state_sub = self.create_subscription(
            Odometry, '/robot_real_state', self.state_callback, 10)

        # ===== 2. 发布里程计 =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ===== 3. TF广播器 =====
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== 4. 机器人位姿 =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ===== 5. 速度缓存 =====
        self.latest_vx = 0.0
        self.latest_vy = 0.0
        self.latest_wz = 0.0
        self.has_data = False

        # ===== 6. EMA滤波状态 =====
        self.filt_vx = 0.0
        self.filt_vy = 0.0
        self.filt_wz = 0.0
        self.ema_alpha = 0.3

        # ===== 7. 时间戳驱动 =====
        self.last_t = self.get_clock().now()
        self.dt = 0.05

        # ===== 8. 静止锁定 =====
        self.still_cnt = 0
        self.STILL_LOCK = 10  # 10周期 = 0.5s
        self.locked = False

        # ===== 9. 累积行进距离 =====
        self.travel_dist = 0.0

        # ===== 10. 定时器 =====
        self.timer = self.create_timer(0.05, self.update_odom)

        self.get_logger().info(
            'odom_publisher 启动（中点积分+死区+EMA+时间戳dt+静止锁+动协方差）')

    # ------------------------------------------------------------
    def state_callback(self, msg: Odometry):
        """接收真实底盘速度，死区+EMA滤波"""
        rvx = msg.twist.twist.linear.x
        rvy = msg.twist.twist.linear.y
        rwz = msg.twist.twist.angular.z

        # 死区
        if abs(rvx) < 0.005:
            rvx = 0.0
        if abs(rvy) < 0.005:
            rvy = 0.0
        if abs(rwz) < 0.002:
            rwz = 0.0

        # EMA低通
        a = self.ema_alpha
        self.filt_vx = a * rvx + (1 - a) * self.filt_vx
        self.filt_vy = a * rvy + (1 - a) * self.filt_vy
        self.filt_wz = a * rwz + (1 - a) * self.filt_wz

        self.latest_vx = self.filt_vx
        self.latest_vy = self.filt_vy
        self.latest_wz = self.filt_wz
        self.has_data = True

    # ------------------------------------------------------------
    def update_odom(self):
        """中点积分 + 时间戳dt + 静止锁 + 动态协方差"""
        if not self.has_data:
            return

        # ---- 实际时间差 ----
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        dt = max(0.001, min(dt, 0.2))  # 钳位
        self.dt = dt

        vx = self.latest_vx
        vy = self.latest_vy
        wz = self.latest_wz

        # ---- 静止检测与锁定 ----
        moving = abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6
        if not moving:
            self.still_cnt += 1
            if self.still_cnt >= self.STILL_LOCK:
                self.locked = True
        else:
            self.still_cnt = 0
            self.locked = False

        if self.locked:
            vx = 0.0
            vy = 0.0
            wz = 0.0

        # ---- 中点积分 ----
        old_yaw = self.yaw
        mid_yaw = old_yaw + wz * dt * 0.5

        dx = (vx * math.cos(mid_yaw) - vy * math.sin(mid_yaw)) * dt
        dy = (vx * math.sin(mid_yaw) + vy * math.cos(mid_yaw)) * dt

        self.x += dx
        self.y += dy
        self.travel_dist += math.sqrt(dx * dx + dy * dy)
        self.yaw = old_yaw + wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ---- 动态协方差 ----
        cov = 0.001 + self.travel_dist * 0.05
        if self.locked:
            cov = 0.001

        # ---- 四元数 ----
        hy = self.yaw * 0.5
        qx, qy, qz, qw = 0.0, 0.0, math.sin(hy), math.cos(hy)

        t = self.get_clock().now().to_msg()

        # ---- 发布 TF ----
        tfm = TransformStamped()
        tfm.header.stamp = t
        tfm.header.frame_id = 'odom'
        tfm.child_frame_id = 'base_link'
        tfm.transform.translation.x = self.x
        tfm.transform.translation.y = self.y
        tfm.transform.translation.z = 0.0
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tfm)

        # ---- 发布 Odometry ----
        om = Odometry()
        om.header.stamp = t
        om.header.frame_id = 'odom'
        om.child_frame_id = 'base_link'
        om.pose.pose.position.x = self.x
        om.pose.pose.position.y = self.y
        om.pose.pose.position.z = 0.0
        om.pose.pose.orientation.x = qx
        om.pose.pose.orientation.y = qy
        om.pose.pose.orientation.z = qz
        om.pose.pose.orientation.w = qw
        om.twist.twist.linear.x = vx
        om.twist.twist.linear.y = vy
        om.twist.twist.angular.z = wz

        # 协方差
        om.pose.covariance = [
            cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, cov]
        om.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.odom_pub.publish(om)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
