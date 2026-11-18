#!/usr/bin/env python3
"""
Go2 真机里程计 TF 桥接节点。

订阅雷达模块 LIO 里程计 /utlidar/robot_odom (nav_msgs/Odometry, odom->base_link)，
广播 TF：

  1. odom -> base_link        原始 6DOF 位姿（含 roll/pitch），供 RViz 机器人模型显示
  2. odom -> base_link_level  水平化虚拟坐标系（保留 x/y/z/yaw，roll/pitch 置零），
                              作为 pointcloud_to_laserscan 的 target_frame，
                              使 2D 切片平面始终与地面平行：
                              机器狗行走时机身俯仰不会让地面点混入 /scan。

TF 时间戳统一重打为本机时钟：robot_odom 的 stamp 属狗主控时钟域（慢约 163 天），
而 AMCL/Nav2/RViz 用本机 now() 查 TF，跨域查询会报 "Transform data too old"。
位姿值不变、仅重打 stamp（150Hz 高频下与采样时刻偏差仅数 ms）；下游 livox_scan
的 /scan stamp 也统一到本机时钟，全链路（TF/scan/AMCL/Nav2）同源自洽。
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):

    def __init__(self):
        super().__init__('odom_tf_bridge')

        self.declare_parameter('odom_topic', '/utlidar/robot_odom')
        # 去俯仰虚拟坐标系名（pointcloud_to_laserscan 的 target_frame 应与其一致）
        self.declare_parameter('level_frame', 'base_link_level')
        self.declare_parameter('publish_level_frame', True)
        # 发布本机时钟重打版 Odometry 到 /odom_local，供 Nav2 controller_server/
        # bt_navigator 的 odom_topic 订阅——狗端 /utlidar/robot_odom 的 stamp 属
        # 狗主控时钟域（启动时 epoch ~33天），与本机 TF/scan 时间不同步，
        # Nav2 MessageFilter 按本机时钟过滤会全部丢弃 -> DWB 拿不到当前速度 -> 不走。
        # 即使时钟已 SSH 同步，重启后狗可能再次漂移，此桥保证始终同源。
        self.declare_parameter('publish_local_odom', True)

        odom_topic = self.get_parameter('odom_topic').value
        self.level_frame = self.get_parameter('level_frame').value
        self.publish_level = self.get_parameter('publish_level_frame').value
        self.publish_local = self.get_parameter('publish_local_odom').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, odom_topic, self.cb, 20)

        if self.publish_local:
            self.odom_pub = self.create_publisher(Odometry, '/odom_local', 20)

        self.get_logger().info(
            f"odom_tf_bridge: '{odom_topic}' -> TF "
            f"[odom->base_link{' , odom->' + self.level_frame if self.publish_level else ''}]"
            f"{' + /odom_local (stamp 重打本机时钟)' if self.publish_local else ''}")

    def cb(self, msg: Odometry):
        parent = msg.header.frame_id or 'odom'
        child = msg.child_frame_id or 'base_link'
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # TF 1: odom -> base_link（原样转发位姿，stamp 重打为本机时钟）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = pos.z
        t.transform.rotation = ori

        transforms = [t]

        # TF 2: odom -> base_link_level（仅保留 yaw 的水平化坐标系）
        if self.publish_level:
            yaw = math.atan2(
                2.0 * (ori.w * ori.z + ori.x * ori.y),
                1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
            tl = TransformStamped()
            tl.header.stamp = t.header.stamp
            tl.header.frame_id = parent
            tl.child_frame_id = self.level_frame
            tl.transform.translation.x = pos.x
            tl.transform.translation.y = pos.y
            tl.transform.translation.z = pos.z
            tl.transform.rotation.z = math.sin(yaw / 2.0)
            tl.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(tl)

        self.br.sendTransform(transforms)

        # 发布本机时钟重打版 Odometry（stamp 与上方 TF 同源），供 Nav2
        # controller_server/bt_navigator 的 odom_topic 订阅
        if self.publish_local:
            local_msg = Odometry()
            local_msg.header.stamp = t.header.stamp   # 本机时钟（与 TF 同源）
            local_msg.header.frame_id = parent
            local_msg.child_frame_id = child
            local_msg.pose = msg.pose
            local_msg.twist = msg.twist
            self.odom_pub.publish(local_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
