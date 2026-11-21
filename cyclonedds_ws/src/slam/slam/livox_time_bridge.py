#!/usr/bin/env python3
"""
Mid360 点云时间戳桥接节点。

问题：/livox/lidar 的时间戳由 livox_ros_driver2 用节点所在主机（Jetson/笔记本，
真实时间域）打戳，而 /utlidar/robot_odom（TF 源，odom_tf_bridge 的输入）用的是
狗主控时钟域（慢约 155 天）。slam_toolbox 拿 scan 时间戳查 TF，两个时钟不同源
则永远查不到，slam_toolbox 不出图。

方案：订阅 robot_odom 实时标定偏移 offset = odom时间戳 - 本机收到时刻，
转发 /livox/lidar -> /livox/lidar_sync 时把 stamp 重写为 本机now() + offset
（即狗主控时钟域的"现在"），frame_id 保持 livox_frame，点数据原样透传。

QoS：订阅用 SensorData（BEST_EFFORT，兼容驱动的 RELIABLE 发布）；
发布用默认 RELIABLE（对下游 pointcloud_to_laserscan 任意 QoS 均兼容）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class LivoxTimeBridge(Node):

    def __init__(self):
        super().__init__('livox_time_bridge')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('sync_topic', '/livox/lidar_sync')
        self.declare_parameter('odom_topic', '/utlidar/robot_odom')

        lidar_topic = self.get_parameter('lidar_topic').value
        sync_topic = self.get_parameter('sync_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.offset = None
        self.pub = self.create_publisher(PointCloud2, sync_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 20)
        self.create_subscription(
            PointCloud2, lidar_topic, self.cloud_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'livox_time_bridge: {lidar_topic} -> {sync_topic} '
            f'(stamp 对齐到 {odom_topic} 的狗主控时钟域)')

    def odom_cb(self, msg: Odometry):
        off = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 \
            - self.get_clock().now().nanoseconds * 1e-9
        if self.offset is None:
            self.offset = off
        else:
            # EMA 平滑网络/调度抖动；有线链路抖动 <1ms，10Hz 点云足够
            self.offset = 0.9 * self.offset + 0.1 * off

    def cloud_cb(self, msg: PointCloud2):
        if self.offset is None:
            return  # 尚未标定偏移，丢弃（冷启动 <1s）
        stamp = self.get_clock().now().nanoseconds * 1e-9 + self.offset
        sec = int(stamp)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((stamp - sec) * 1e9)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxTimeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
