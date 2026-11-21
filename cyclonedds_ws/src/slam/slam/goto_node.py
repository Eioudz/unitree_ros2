#!/usr/bin/env python3
"""语义导航：中文指令 -> 地图坐标 -> 闭环控制直线移动（无避障）。

两种输入方式：
1. 终端直跑（ros2 run，stdin 是终端）时直接打字：去前门 / 去后门 / 停止
2. 话题 /goto_cmd (std_msgs/String)，launch 方式下 stdin 不通时用：
   ros2 topic pub --once /goto_cmd std_msgs/msg/String "{data: 去前门}"
   前门 -> (-3.61, -2.52)，后门 -> (-6.81, 3.69)（RViz Publish Point 实点标定）
   停止/停/stop -> 立即停

控制：20Hz 查 TF map->base_link_level（AMCL 提供），P 控制先转向再前进，
1008 MOVE 发布；req_id 自增（固定 0 会被 sport 服务丢弃）。
直线路径上如有障碍物不会绕行，请人工保证通道畅通。
前置：localization.launch.py 已启动且已完成 2D Pose Estimate 初始化。
"""
import json
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from unitree_api.msg import Request

RATE_HZ = 20.0
ARRIVE_M = 0.3        # 到位判定半径
MAX_VX = 0.4          # m/s
MAX_WZ = 0.6          # rad/s
ALIGN_RAD = 0.35      # 朝向误差小于此值才前进
TIMEOUT_S = 90.0      # 单次导航超时


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class GotoNode(Node):

    def __init__(self):
        super().__init__('goto_node')
        self.declare_parameter('front_door', [-3.61, -2.52])  # RViz 实点标定
        self.declare_parameter('back_door', [-6.81, 3.69])    # RViz 实点标定

        self.goals = {
            'front': tuple(self.get_parameter('front_door').value),
            'back': tuple(self.get_parameter('back_door').value),
        }
        self.target = None          # (x, y) 或 None
        self.start_time = None
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.req_id = 0
        self.pub = self.create_publisher(
            Request, '/api/sport/request',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0 / RATE_HZ, self.control_tick)
        self.create_subscription(String, 'goto_cmd', self.cmd_cb, 10)
        # stdin 是终端（ros2 run 直跑）时开打字输入；launch 下跳过，走话题
        self.interactive = sys.stdin is not None and sys.stdin.isatty()
        if self.interactive:
            threading.Thread(target=self.input_loop, daemon=True).start()
        self.get_logger().info(
            ('就绪，直接输入：去前门/去后门/停止'
             if self.interactive else
             '就绪。发指令：ros2 topic pub --once /goto_cmd '
             'std_msgs/msg/String "{data: 去前门}"')
            + '  [前门(%.1f,%.1f) 后门(%.1f,%.1f)]'
            % (self.goals['front'] + self.goals['back']))

    # ---------- 指令解析 -> 坐标 ----------
    def handle_cmd(self, cmd):
        cmd = cmd.strip().replace(' ', '')
        if not cmd:
            return
        if '停' in cmd or 'stop' in cmd.lower():
            self.abort('收到停止指令')
        elif '前门' in cmd:
            self.set_goal('front', '前门')
        elif '后门' in cmd:
            self.set_goal('back', '后门')
        else:
            self.get_logger().warn('无法理解：%s（支持：去前门/去后门/停止）' % cmd)

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
        self.target = self.goals[key]
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            '导航至 %s：目标 (%.2f, %.2f)' % (label, *self.target))

    def abort(self, reason):
        if self.target is not None:
            self.get_logger().info('中止：%s' % reason)
        self.target = None
        self.publish_move(0.0, 0.0)

    # ---------- 20Hz 控制循环 ----------
    def control_tick(self):
        if self.target is None:
            return
        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 > TIMEOUT_S:
            self.abort('超时 %.0fs' % TIMEOUT_S)
            return
        try:
            tf = self.tf_buf.lookup_transform(
                'map', 'base_link_level', rclpy.time.Time())
        except Exception:
            self.get_logger().warn('无 TF map->base_link_level（定位未启动或未初始化？）',
                                    throttle_duration_sec=3.0)
            return
        p = tf.transform.translation
        yaw = yaw_from_quat(*(getattr(tf.transform.rotation, a) for a in 'xyzw'))
        dx, dy = self.target[0] - p.x, self.target[1] - p.y
        dist = math.hypot(dx, dy)
        if dist < ARRIVE_M:
            self.abort('到达目标 (%.2f, %.2f)' % self.target)
            return
        err = wrap(math.atan2(dy, dx) - yaw)
        wz = max(-MAX_WZ, min(MAX_WZ, 2.0 * err))
        vx = 0.0 if abs(err) > ALIGN_RAD else min(MAX_VX, max(0.15, dist))
        self.publish_move(vx, wz)

    def publish_move(self, vx, wz):
        req = Request()
        req.header.identity.api_id = 1008
        self.req_id += 1
        req.header.identity.id = self.req_id
        req.header.lease.id = 0
        req.header.policy.noreply = True
        req.parameter = json.dumps({'x': float(vx), 'y': 0.0, 'z': float(wz)})
        self.pub.publish(req)


def main(args=None):
    rclpy.init(args=args)
    node = GotoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_move(0.0, 0.0)
        except Exception:
            pass    # 退出时 context 可能已关闭
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
