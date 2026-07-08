"""
极简导航器 —— 替代 bt_navigator
直接调用 planner_server + controller_server，不依赖行为树

v4 改进:
- 同时支持两种 RViz 操作方式:
  1. 工具栏 "2D Goal Pose" (发布 /goal_pose 话题)
  2. Nav2 面板 "Nav2 Goal" (发布 /navigate_to_pose Action)
- 完全非阻塞异步调用
- 支持任务取消
- 完整的错误处理和状态恢复
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # 使用可重入回调组以支持异步流水线
        cb_group = ReentrantCallbackGroup()

        self.planner_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose',
            callback_group=cb_group)
        self.controller_client = ActionClient(
            self, FollowPath, '/follow_path',
            callback_group=cb_group)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10,
            callback_group=cb_group)

        # ---- 监控 /cmd_vel ----
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10,
            callback_group=cb_group)

        # ---- 状态机 ----
        self._busy = False
        self._planner_goal_handle = None
        self._controller_goal_handle = None
        self._cancel_requested = False

        # ---- NavigateToPose Action 服务（兼容 Nav2 RViz 面板）----
        self._nav_action_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self._navigate_to_pose_cb,
            callback_group=cb_group)

        self.get_logger().info(
            '✅ 极简导航器 v4 就绪（全异步 + Action），'
            '支持工具栏 2D Goal Pose 和 Nav2 面板 Nav2 Goal'
        )

    # ===================== cmd_vel 监控 =====================
    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    # ===================== 取消当前任务 =====================
    def _cancel_current_task(self):
        self._cancel_requested = True
        if self._planner_goal_handle is not None:
            self.get_logger().info('🛑 取消 planner 任务...')
            try:
                self._planner_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._planner_goal_handle = None
        if self._controller_goal_handle is not None:
            self.get_logger().info('🛑 取消 controller 任务...')
            try:
                self._controller_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._controller_goal_handle = None
        self._busy = False

    # ===================== 接收目标 =====================
    def goal_cb(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn('⚠ 检测到新目标，取消上一个导航任务...')
            self._cancel_current_task()

        self._busy = True
        self._cancel_requested = False
        self.get_logger().info(
            f'🎯 收到目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            f'frame={msg.header.frame_id}'
        )

        if not self.planner_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ planner_server 未就绪！请等待 lifecycle 激活完成')
            self._busy = False
            return

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = msg
        plan_goal.planner_id = 'GridBased'

        self.get_logger().info('📐 向 planner 请求路径（异步）...')
        send_future = self.planner_client.send_goal_async(plan_goal)
        send_future.add_done_callback(self._on_planner_goal_response)

    # ===================== NavigateToPose Action 回调（Nav2 面板用）=====================
    def _navigate_to_pose_cb(self, goal_handle):
        """ActionServer 回调 — Nav2 RViz 面板点击 Nav2 Goal 时触发"""
        msg = goal_handle.request.pose

        # 先通过 topic 方式发起导航（复用 goal_cb 逻辑）
        if self._busy:
            self.get_logger().warn('⚠ 有任务在进行中，先取消再开始新任务')
            self._cancel_current_task()

        self._busy = True
        self._cancel_requested = False
        self.get_logger().info(
            f'🎯 [Action] 收到导航目标: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}), frame={msg.header.frame_id}'
        )

        if not self.planner_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ planner_server 未就绪！')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        plan_goal = ComputePathToPose.Goal()
        plan_goal.goal = msg
        plan_goal.planner_id = 'GridBased'

        self.get_logger().info('📐 [Action] 向 planner 请求路径...')

        # 用 planner 的异步响应来驱动后续流程
        send_future = self.planner_client.send_goal_async(plan_goal)

        # 由于 ActionServer 的 execute_callback 是同步的，我们用 spin_until_future_complete
        # 这在 ActionServer 回调中是安全的（不会阻塞其他话题订阅）
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        gh = send_future.result()
        if not gh or not gh.accepted:
            self.get_logger().error('❌ [Action] planner 拒绝或超时')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)

        plan_result = result_future.result()
        if plan_result is None:
            self.get_logger().error('❌ [Action] planner 结果为空')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        try:
            path: Path = plan_result.result.path
        except Exception as e:
            self.get_logger().error(f'❌ [Action] 获取路径失败: {e}')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        if not path.poses:
            self.get_logger().error('❌ [Action] 路径为空！')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info(f'✅ [Action] 路径计算成功: {len(path.poses)} 个点')

        if not path.header.frame_id:
            path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # ---- 跟踪路径 ----
        if not self.controller_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ [Action] controller_server 未就绪！')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info('🚀 [Action] 向 controller 发送跟踪指令...')

        send_future = self.controller_client.send_goal_async(
            follow_goal,
            feedback_callback=self._controller_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)

        ctrl_gh = send_future.result()
        if not ctrl_gh or not ctrl_gh.accepted:
            self.get_logger().error('❌ [Action] controller 拒绝')
            self._busy = False
            goal_handle.abort()
            return NavigateToPose.Result()

        self.get_logger().info('✅ [Action] controller 已接受，机器人正在移动...')
        self._controller_goal_handle = ctrl_gh

        # 等待 controller 完成
        result_future = ctrl_gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self._controller_goal_handle = None
        self._busy = False

        # 返回结果给 Action 客户端（RViz）
        result = NavigateToPose.Result()
        try:
            ctrl_result = result_future.result()
            if ctrl_result is not None and ctrl_result.result is not None:
                result.result = ctrl_result.result
        except Exception:
            pass

        if result.result.error_code == 0:
            self.get_logger().info('🏁 [Action] 导航成功！')
            goal_handle.succeed()
        else:
            self.get_logger().error(f'❌ [Action] 导航失败: code={result.result.error_code}')
            goal_handle.abort()

        return result

    # ===================== Planner 目标响应 =====================
    def _on_planner_goal_response(self, future):
        if self._cancel_requested:
            self._busy = False
            return

        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('❌ planner 返回空 handle')
            self._busy = False
            return
        if not goal_handle.accepted:
            self.get_logger().error('❌ planner 拒绝了路径计算请求')
            self._busy = False
            return

        self._planner_goal_handle = goal_handle
        self.get_logger().info('✅ planner 已接受，正在计算路径...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_planner_result)

    # ===================== Planner 结果 =====================
    def _on_planner_result(self, future):
        self._planner_goal_handle = None

        if self._cancel_requested:
            self._busy = False
            return

        plan_result = future.result()
        if plan_result is None:
            self.get_logger().error('❌ planner 结果为空')
            self._busy = False
            return

        try:
            path: Path = plan_result.result.path
        except Exception as e:
            self.get_logger().error(f'❌ 获取路径失败: {e}')
            self._busy = False
            return

        if not path.poses:
            self.get_logger().error(
                '❌ 路径为空！目标可能不可达。请选择地图空闲区域。'
            )
            self._busy = False
            return

        self.get_logger().info(
            f'✅ 路径计算成功: {len(path.poses)} 个点, '
            f'frame={path.header.frame_id}'
        )

        # 确保 path header 正确
        if not path.header.frame_id:
            path.header.frame_id = 'map'
            self.get_logger().warn('⚠ path.header.frame_id 为空，已设为 map')
        path.header.stamp = self.get_clock().now().to_msg()

        # ---- Step 2: 跟踪路径 ----
        if not self.controller_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('❌ controller_server 未就绪！请等待 lifecycle 激活完成')
            self._busy = False
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        follow_goal.controller_id = 'FollowPath'
        follow_goal.goal_checker_id = 'general_goal_checker'

        self.get_logger().info('🚀 向 controller 发送跟踪指令（异步）...')
        send_future = self.controller_client.send_goal_async(
            follow_goal,
            feedback_callback=self._controller_feedback_cb
        )
        send_future.add_done_callback(self._on_controller_goal_response)

    # ===================== Controller 目标响应 =====================
    def _on_controller_goal_response(self, future):
        if self._cancel_requested:
            self._busy = False
            return

        ctrl_goal_handle = future.result()
        if not ctrl_goal_handle:
            self.get_logger().error(
                '❌ controller 返回空 handle — '
                '可能未激活或存在 TF/costmap 问题'
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

        self.get_logger().info('✅ controller 已接受，机器人正在移动...')
        self._controller_goal_handle = ctrl_goal_handle
        result_future = ctrl_goal_handle.get_result_async()
        result_future.add_done_callback(self._controller_result_cb)

    # ===================== Controller 反馈 =====================
    def _controller_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'📡 [Controller] '
            f'剩余距离={fb.distance_to_goal:.2f}m, '
            f'当前速度={fb.speed:.2f}m/s'
        )

    # ===================== Controller 结果 =====================
    def _controller_result_cb(self, future):
        self._controller_goal_handle = None
        self._busy = False

        if self._cancel_requested:
            self.get_logger().info('🛑 导航任务已取消')
            return

        try:
            result = future.result()
            if result is None or result.result is None:
                self.get_logger().error('❌ controller 返回空结果')
                return

            code = result.result.error_code
            if code == 0:
                self.get_logger().info('=' * 50)
                self.get_logger().info('🏁 导航成功！机器人已到达目标位置！')
                self.get_logger().info('=' * 50)
            else:
                error_msgs = {
                    1: '失败(通用)',
                    100: '规划超时',
                    200: '目标检查失败',
                    201: '路径阻塞',
                    202: '旋转到目标失败',
                    203: '定位丢失',
                    204: '目标不可达',
                    400: '内部错误',
                }
                desc = error_msgs.get(code, f'未知错误')
                self.get_logger().error(f'❌ 导航失败: code={code} ({desc})')
        except Exception as e:
            self.get_logger().error(f'❌ controller 结果异常: {e}')


def main():
    rclpy.init()
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
