#!/usr/bin/env python3
"""语义导航：中文指令 -> 地图坐标 -> nav2 自动规划+避障。

两种输入方式：
1. 终端直跑（ros2 run，stdin 是终端）时直接打字：去前门 / 去后门 / 去电视机前 / 停止
2. 话题 /goto_cmd (std_msgs/String)，launch 方式下 stdin 不通时用：
   ros2 topic pub --once /goto_cmd std_msgs/msg/String "{data: 去前门}"
   前门 -> (-3.61, -2.52)，后门 -> (-6.81, 3.69)，电视机前 -> (-4.88, 4.25)
   （RViz Publish Point 实点标定）
   停止/停/stop -> 取消当前导航任务（cmd_vel_sport_bridge 看门狗自动停车）

链路：ActionClient(NavigateToPose) -> bt_navigator -> planner/behavior -> /cmd_vel
      -> cmd_vel_sport_bridge -> 1008 MOVE（与 RViz 2D Goal Pose 同一条链）。
不用 BasicNavigator：它内部 spin_until_future_complete 会与本节点的主 spin
并发执行同一节点，导致 "generator already executing" / 段错误（2026-11-07 实测）。
前置：navigation.launch.py 已启动且已完成 2D Pose Estimate 初始化。
"""
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class GotoNode(Node):

    def __init__(self):
        super().__init__('goto_node')
        self.declare_parameter('front_door', [-3.61, -2.52])  # RViz 实点标定
        self.declare_parameter('back_door', [-6.81, 3.69])    # RViz 实点标定
        self.declare_parameter('tv', [-4.88, 4.25])           # RViz 实点标定

        self.goals = {
            'front': tuple(self.get_parameter('front_door').value),
            'back': tuple(self.get_parameter('back_door').value),
            'tv': tuple(self.get_parameter('tv').value),
        }
        self.goal_handle = None
        self.goal_label = None
        # 目标代号：每次下发自增。旧目标的迟到回调（被抢占/取消后再到达）
        # 凭代号丢弃，避免清掉新目标的 label/handle（2026-11-16 修串扰）
        self.goal_seq = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(String, 'goto_cmd', self.cmd_cb, 10)
        # stdin 是终端（ros2 run 直跑）时开打字输入；launch 下跳过，走话题
        self.interactive = sys.stdin is not None and sys.stdin.isatty()
        if self.interactive:
            threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info(
            ('就绪，直接输入：去前门/去后门/去电视机前/停止'
             if self.interactive else
             '就绪。发指令：ros2 topic pub --once /goto_cmd '
             'std_msgs/msg/String "{data: 去前门}"')
            + '  [前门(%.1f,%.1f) 后门(%.1f,%.1f) 电视(%.1f,%.1f)]'
            % (self.goals['front'] + self.goals['back'] + self.goals['tv']))

    # ---------- 指令解析 -> nav2 目标 ----------
    def handle_cmd(self, cmd):
        cmd = cmd.strip().replace(' ', '')
        if not cmd:
            return
        if '停' in cmd or 'stop' in cmd.lower():
            self.cancel()
        elif '前门' in cmd:
            self.set_goal('front', '前门')
        elif '后门' in cmd:
            self.set_goal('back', '后门')
        elif '电视' in cmd:
            self.set_goal('tv', '电视机前')
        else:
            self.get_logger().warn('无法理解：%s（支持：去前门/去后门/去电视机前/停止）' % cmd)

    def cmd_cb(self, msg: String):
        self.handle_cmd(msg.data)

    def input_loop(self):
        while rclpy.ok():
            try:
                line = input()
            except EOFError:
                return
            self.handle_cmd(line)

    def set_goal(self, key, label):
        if not self.client.server_is_ready():
            self.get_logger().warn(
                'navigate_to_pose action 不可用（导航栈未启动或未激活？）')
            return
        x, y = self.goals[key]
        yaw = self.goal_yaw(x, y)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_label = label
        self.goal_handle = None
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda fb: self.fb_cb(fb, seq, label))
        future.add_done_callback(lambda f: self.goal_resp_cb(f, seq, label))
        self.get_logger().info(
            '导航至 %s：目标 (%.2f, %.2f) 朝向 %.0f°'
            % (label, x, y, math.degrees(yaw)))

    def goal_yaw(self, x, y):
        """目标朝向取"机器人->目标"方位角，到达后无需无谓地原地转身；
        已在目标 0.2m 内则保持当前朝向；TF 不可用时退回 0°。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link_level', Time())
        except Exception:
            return 0.0
        q = tf.transform.rotation
        cur_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = x - tf.transform.translation.x
        dy = y - tf.transform.translation.y
        if math.hypot(dx, dy) < 0.2:
            return cur_yaw
        return math.atan2(dy, dx)

    def goal_resp_cb(self, future, seq, label):
        if seq != self.goal_seq:      # 已被更新的目标取代，丢弃旧响应
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('目标被拒绝：%s' % label)
            self.goal_label = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.result_cb(f, seq, label))

    def fb_cb(self, feedback, seq, label):
        if seq != self.goal_seq:
            return
        self.get_logger().info(
            '%s：剩余 %.2f m'
            % (label, feedback.feedback.distance_remaining),
            throttle_duration_sec=1.0)

    def result_cb(self, future, seq, label):
        if seq != self.goal_seq:      # 取消/被抢占后的迟到结果
            return
        status = future.result().status
        text = {GoalStatus.STATUS_SUCCEEDED: '成功',
                GoalStatus.STATUS_CANCELED: '取消',
                GoalStatus.STATUS_ABORTED: '中止'}.get(status, status)
        self.get_logger().info('%s：导航结果 %s' % (label, text))
        self.goal_label = None
        self.goal_handle = None

    def cancel(self):
        if self.goal_label is not None:
            self.get_logger().info('取消导航：%s' % self.goal_label)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_label = None
        self.goal_handle = None
        self.goal_seq += 1            # 让在途回调失效


def main(args=None):
    rclpy.init(args=args)
    node = GotoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
