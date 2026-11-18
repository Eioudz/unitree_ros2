#!/usr/bin/env python3
"""Nav2 /cmd_vel -> Go2 运动指令桥接（bt_navigator 链路的最后一环）。

DWB 控制器输出标准 /cmd_vel (geometry_msgs/Twist)，Go2 只认
/api/sport/request api_id=1008 MOVE，本节点做转换 + 安全兜底：
    1. 限幅：vx∈[min_vx, max_vx]、|wz|<=max_wz（与 costmap.yaml 的 DWB 限值一致；
       2026-11-17 放开负 vx：贴脸障碍时倒车逃逸/BT BackUp 是唯一出路）
    2. 看门狗：超过 watchdog 秒收不到 /cmd_vel（控制器断流/崩溃）立即发停车，
       避免"最后一条指令一直生效"导致狗刹不住
    3. 1008 MOVE 需 req_id 自增（固定 0 会被 sport 服务丢弃）

测试（无机器人）：
    ros2 topic pub -r 5 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.3}}"
    # 另一终端 echo /api/sport/request 应看到 x=-0.3 原样透传
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from unitree_api.msg import Request


class CmdVelSportBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_sport_bridge')
        self.declare_parameter('min_vx', 0.0)    # 禁倒车：vx>=0，需要后退靠原地转向
        self.declare_parameter('max_vx', 0.5)    # 与 costmap.yaml 的 DWB 限值一致（2026-11-17 提速）
        self.declare_parameter('max_wz', 1.2)    # 与 DWB 角速度上限一致（2026-11-17 提速）
        self.declare_parameter('watchdog', 0.5)   # s，超时停车

        self.min_vx = self.get_parameter('min_vx').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_wz = self.get_parameter('max_wz').value
        self.watchdog = self.get_parameter('watchdog').value

        self.req_id = 0
        self.last_cmd_time = None
        self.stopped_sent = True   # 初始视为已停，避免启动即发多余停车
        self.pub = self.create_publisher(
            Request, '/api/sport/request',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        # 20Hz 看门狗
        self.create_timer(0.05, self.watchdog_tick)
        self.get_logger().info(
            '就绪：/cmd_vel -> 1008 MOVE（vx∈[%.2f, %.2f]，|wz|<=%.2f，'
            '看门狗 %.2fs）' % (self.min_vx, self.max_vx, self.max_wz, self.watchdog))

    def cmd_cb(self, msg: Twist):
        # 限幅：允许倒车（负 vx 透传），正负各自封顶
        vx = max(self.min_vx, min(float(msg.linear.x), self.max_vx))
        wz = max(-self.max_wz, min(float(msg.angular.z), self.max_wz))
        self.last_cmd_time = self.get_clock().now()
        self.stopped_sent = False
        self.publish_move(vx, wz)

    def watchdog_tick(self):
        if self.stopped_sent or self.last_cmd_time is None:
            return
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.watchdog:
            self.publish_move(0.0, 0.0)
            self.stopped_sent = True
            self.get_logger().warn('看门狗触发：%.2fs 无 /cmd_vel，已停车' % self.watchdog)

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
    node = CmdVelSportBridge()
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
