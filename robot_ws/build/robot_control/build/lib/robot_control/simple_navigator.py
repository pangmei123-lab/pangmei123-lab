"""
极简导航器 —— 替代 bt_navigator
直接调用 planner_server + controller_server，不依赖行为树

v2 改进:
- 检查 controller goal 是否被接受
- 设置 goal_checker_id
- controller 反馈回调（打印当前速度）
- 结果回调（完成/失败诊断）
- 状态机防重复触发
- path header 帧校验
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from nav2_msgs.action import ComputePathToPose, FollowPath
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        cb_group = MutuallyExclusiveCallbackGroup()

        self.planner_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose',
            callback_group=cb_group)
        self.controller_client = ActionClient(
            self, FollowPath, '/follow_path',
            callback_group=cb_group)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10,
            callback_group=cb_group)

        # ---- 监控 /cmd_vel 确认链路通断 ----
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # ---- 状态机 ----
        self._busy = False
        self._current_controller_goal_handle = None

        self.get_logger().info('✅ 极简导航器 v2 就绪，等待 2D Nav Goal...')

    # ===================== cmd_vel 监控 =====================
    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    # ===================== 接收目标 =====================
    def goal_cb(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn('⚠ 上一个导航任务仍在执行，忽略新目标')
            return

        self._busy = True
        self.get_logger().info(
            f'🎯 收到目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            f'frame={msg.header.frame_id}'
        )

        if not self.planner_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ planner_server 未就绪！请检查 lifecycle')
            self._busy = False
            return

        # ---- Step 1: 计算路径 ----
        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = msg
        plan_goal.planner_id = 'GridBased'

        self.get_logger().info('📐 向 planner 请求路径...')
        plan_future = self.planner_client.send_goal_async(plan_goal)
        rclpy.spin_until_future_complete(self, plan_future, timeout_sec=5.0)

        goal_handle = plan_future.result()
        if not goal_handle:
            self.get_logger().error('❌ planner 返回空 handle')
            self._busy = False
            return
        if not goal_handle.accepted:
            self.get_logger().error('❌ planner 拒绝了路径计算请求')
            self._busy = False
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

        plan_result = result_future.result()
        if plan_result is None:
            self.get_logger().error('❌ planner 结果为空')
            self._busy = False
            return

        path: Path = plan_result.result.path

        if not path.poses:
            self.get_logger().error('❌ 路径为空！目标可能不可达或被障碍物阻挡')
            self._busy = False
            return

        self.get_logger().info(
            f'✅ 路径计算成功: {len(path.poses)} 个点, '
            f'frame={path.header.frame_id}, stamp={path.header.stamp.sec}s'
        )

        # ---- 确保 path header 正确 ----
        if not path.header.frame_id:
            path.header.frame_id = 'map'
            self.get_logger().warn('⚠ path.header.frame_id 为空，已设为 map')
        path.header.stamp = self.get_clock().now().to_msg()

        # ---- Step 2: 跟踪路径 ----
        if not self.controller_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ controller_server 未就绪！请检查 lifecycle')
            self._busy = False
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info('🚀 向 controller 发送跟踪指令...')

        send_future = self.controller_client.send_goal_async(
            follow_goal,
            feedback_callback=self._controller_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)

        ctrl_goal_handle = send_future.result()
        if not ctrl_goal_handle:
            self.get_logger().error(
                '❌ controller 返回空 handle — '
                'controller_server 可能未激活或存在 TF/costmap 问题'
            )
            self._busy = False
            return
        if not ctrl_goal_handle.accepted:
            self.get_logger().error(
                '❌ controller 拒绝了跟踪请求！'
                '可能原因: 路径无效/costmap异常/TF缺失/已在目标位置'
            )
            self._busy = False
            return

        self.get_logger().info('✅ controller 接受了跟踪请求，正在执行...')
        self._current_controller_goal_handle = ctrl_goal_handle

        # ---- 异步等待 controller 结果 ----
        result_future = ctrl_goal_handle.get_result_async()
        result_future.add_done_callback(self._controller_result_cb)

    # ===================== Controller 反馈 =====================
    def _controller_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'📡 [Controller反馈] '
            f'dist_to_goal={fb.distance_to_goal:.2f}m, '
            f'speed={fb.speed:.2f}m/s, '
            f'cmd_vel=({self.last_cmd_vel.linear.x:.3f}, '
            f'{self.last_cmd_vel.linear.y:.3f}, '
            f'{self.last_cmd_vel.angular.z:.3f})'
        )

    # ===================== Controller 结果 =====================
    def _controller_result_cb(self, future):
        self._busy = False
        self._current_controller_goal_handle = None

        try:
            result = future.result()
            code = result.result.error_code
            if code == 0:
                self.get_logger().info('🏁 导航成功到达目标！')
            else:
                self.get_logger().error(
                    f'❌ 导航失败，error_code={code}, '
                    f'可能原因: 超时/路径阻塞/定位丢失'
                )
        except Exception as e:
            self.get_logger().error(f'❌ controller 结果异常: {e}')


def main():
    rclpy.init()
    node = SimpleNavigator()
    rclpy.spin(node)
