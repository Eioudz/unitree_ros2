#!/usr/bin/env python3
"""键盘遥控 Go2(慢速版)。

键位(按下后保持运动,直至按下一个键):
  w / ↑   前进        a / ←   左转
  d / →   右转        s / 空格 停止
  q       退出(退出前自动停车)
安全: 超过 auto_stop_sec 秒未按键自动停车(防失控); 无后退键(vx≥0 硬约束)。
"""
import json
import queue
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_api.msg import Request


class KeyboardNode(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('speed', 0.2)          # 前进线速度 m/s
        self.declare_parameter('turn_rate', 0.4)       # 转向角速度 rad/s
        self.declare_parameter('auto_stop_sec', 5.0)    # 无按键自动停车
        self.declare_parameter('publish_hz', 20.0)      # 指令发布频率

        request_topic = self.get_parameter('request_topic').value
        self.speed = self.get_parameter('speed').value
        self.turn_rate = self.get_parameter('turn_rate').value
        self.auto_stop_sec = self.get_parameter('auto_stop_sec').value
        hz = self.get_parameter('publish_hz').value

        request_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(Request, request_topic, request_qos)
        self.create_subscription(
            Request, '/api/sport/response', self._response_cb, 10)

        self.vx = 0.0
        self.vyaw = 0.0
        self.req_id = 0
        self.last_key_time = time.time()
        self.running = True

        self.key_q = queue.Queue()
        self.term_attrs = None
        if sys.stdin.isatty():
            self.term_attrs = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)  # 保留 Ctrl+C(ISIG),按字节读
        self.key_thread = threading.Thread(target=self._read_keys, daemon=True)
        self.key_thread.start()

        self.create_timer(1.0 / hz, self.timer_callback)
        self.get_logger().info(
            f'键盘遥控就绪: w/↑前进 a/←左转 d/→右转 s/空格停止 q退出 | '
            f'速度={self.speed}m/s 转速={self.turn_rate}rad/s '
            f'自动停车={self.auto_stop_sec}s')

    def _read_keys(self):
        """后台线程: 阻塞读终端,解析普通键与方向键序列。"""
        while self.running:
            ch = sys.stdin.read(1)
            if not ch:
                break
            if ch == '\x1b' and select.select([sys.stdin], [], [], 0.1)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = '\x1b[' + sys.stdin.read(1)  # 方向键
                else:
                    ch = '\x1b'
            self.key_q.put(ch)

    def _apply_key(self, key):
        self.last_key_time = time.time()
        if key in ('w', '\x1b[A'):
            self.vx, self.vyaw = self.speed, 0.0
        elif key in ('a', '\x1b[D'):
            self.vx, self.vyaw = 0.0, self.turn_rate
        elif key in ('d', '\x1b[C'):
            self.vx, self.vyaw = 0.0, -self.turn_rate
        elif key in ('s', ' ', '\x1b[B'):
            self.vx, self.vyaw = 0.0, 0.0
        elif key == 'q':
            self.running = False
        # 其他键忽略

    def _response_cb(self, resp):
        self.get_logger().info(
            f'sport应答 api={resp.header.identity.api_id} '
            f'参数={resp.parameter[:200] if resp.parameter else "无"}')

    def _publish_move(self):
        req = Request()
        req.header.identity.api_id = 1008
        self.req_id += 1
        req.header.identity.id = self.req_id
        req.header.lease.id = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps({
            'x': float(self.vx),
            'y': 0.0,
            'z': float(self.vyaw),
        })
        self.pub.publish(req)

    def timer_callback(self):
        while not self.key_q.empty():
            self._apply_key(self.key_q.get_nowait())
        if not self.running:
            self.vx, self.vyaw = 0.0, 0.0
        # 无按键超时保护(仅运动时触发)
        if (self.auto_stop_sec > 0.0 and (self.vx != 0.0 or self.vyaw != 0.0)
                and time.time() - self.last_key_time > self.auto_stop_sec):
            self.vx, self.vyaw = 0.0, 0.0
            self.get_logger().info('超时未按键,自动停止')
        self._publish_move()
        self.get_logger().info(
            f'发布 id={self.req_id} vx={self.vx} vyaw={self.vyaw}',
            throttle_duration_sec=2.0)
        if not self.running:
            raise SystemExit

    def shutdown(self):
        self.running = False
        self.vx, self.vyaw = 0.0, 0.0
        try:
            self._publish_move()  # 退出前停车
        except Exception:
            pass
        if self.term_attrs is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.term_attrs)


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = KeyboardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
