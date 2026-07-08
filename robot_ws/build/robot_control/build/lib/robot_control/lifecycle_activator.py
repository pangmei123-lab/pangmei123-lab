# ============================================================
# Lifecycle 节点激活器 v2
# - 参数化节点列表
# - bt_navigator 激活前主动探测 planner/controller 的 action 服务
# - 超时 30 秒，每 0.5 秒检查一次
# ============================================================

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time
import sys


class LifecycleActivator(Node):

    # bt_navigator 依赖的 action 服务
    REQUIRED_ACTIONS = [
        '/compute_path_to_pose',
        '/follow_path',
    ]

    # 也必须有 map 帧（AMCL 发布）
    REQUIRED_FRAMES = ['map']

    def __init__(self):
        super().__init__('lifecycle_activator')

        self.declare_parameter('node_names', ['map_server', 'amcl'])
        node_names = self.get_parameter(
            'node_names').get_parameter_value().string_array_value

        max_retries = 60
        retry_delay = 0.5

        success = True
        for name in node_names:
            # bt_navigator 最后一个激活，之前确保依赖的 action 服务就绪
            if name == 'bt_navigator':
                if not self._wait_for_actions(timeout=30.0):
                    self.get_logger().error(
                        '❌ 依赖的 action 服务未就绪，跳过 bt_navigator')
                    success = False
                    break

            if not self._activate_node(name, max_retries, retry_delay):
                success = False

        if success:
            self.get_logger().info('=' * 50)
            self.get_logger().info('✅ 所有 lifecycle 节点已激活，导航就绪！')
            self.get_logger().info('=' * 50)
        else:
            self.get_logger().error('❌ 部分节点激活失败')

        self.destroy_node()
        rclpy.shutdown()

    # ---------- 探测 action 服务 + TF 帧 ----------
    def _wait_for_actions(self, timeout):
        """等待 planner/controller 的 action 服务 + map 帧全部就绪"""
        from rclpy.executors import SingleThreadedExecutor
        import tf2_ros

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)

        start = time.time()
        while time.time() - start < timeout:
            executor.spin_once(timeout_sec=0.1)

            all_ready = True
            missing = []

            # 检查 action 服务
            topics = self.get_topic_names_and_types()
            for action_name in self.REQUIRED_ACTIONS:
                found = any(action_name in t for t, _ in topics)
                if not found:
                    all_ready = False
                    missing.append(f'action:{action_name}')

            # 检查 TF 帧
            for frame in self.REQUIRED_FRAMES:
                if not tf_buffer.can_transform(
                        'base_link', frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)):
                    all_ready = False
                    missing.append(f'frame:{frame}')

            if all_ready:
                self.get_logger().info(
                    f'✅ action + frame 全部就绪 ({time.time() - start:.1f}s)')
                executor.remove_node(self)
                return True

            elapsed = time.time() - start
            if int(elapsed) % 5 == 0 and elapsed >= 4.5:
                self.get_logger().info(
                    f'  等待: {missing} ({elapsed:.0f}s/{timeout:.0f}s)')

            time.sleep(0.3)

        executor.remove_node(self)
        self.get_logger().error('❌ action/frame 等待超时')
        return False

    # ---------- 激活单个节点 ----------
    def _activate_node(self, node_name, max_retries, retry_delay):
        change_state_client = self.create_client(
            ChangeState, f'/{node_name}/change_state')

        self.get_logger().info(f'⏳ 等待 {node_name} 上线...')
        for i in range(max_retries):
            if change_state_client.wait_for_service(timeout_sec=retry_delay):
                break
            if i % 10 == 0:
                self.get_logger().info(
                    f'  等待 {node_name} ({i}/{max_retries})...')
        else:
            self.get_logger().error(f'❌ {node_name} 超时未上线')
            return False

        time.sleep(1.0)

        self.get_logger().info(f'🔄 {node_name}: configuring...')
        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        future = change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not (future.result() and future.result().success):
            self.get_logger().warn(f'⚠ {node_name}: configure 失败（可能已配置）')

        time.sleep(0.5)

        self.get_logger().info(f'🔄 {node_name}: activating...')
        req.transition.id = Transition.TRANSITION_ACTIVATE
        future = change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info(f'✅ {node_name}: activated')
            return True
        else:
            self.get_logger().error(f'❌ {node_name}: activate 失败')
            return False


def main():
    rclpy.init(args=sys.argv)
    LifecycleActivator()
