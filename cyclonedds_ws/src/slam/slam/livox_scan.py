#!/usr/bin/env python3
"""
Mid360 点云切片节点（时间戳对齐 + 水平切片二合一，替代 livox_time_bridge +
pointcloud_to_laserscan 组合）。

为什么自己写：
1. 时钟域问题：/utlidar/robot_odom（TF 源）的 stamp 是狗主控时钟域（慢约
   163 天），AMCL/Nav2/RViz 用本机时钟查 TF 跨域必挂 -> odom_tf_bridge 与本
   节点统一把 stamp 重打为本机时钟，全链路（TF/scan/AMCL/Nav2）同源自洽。
2. pointcloud_to_laserscan 包在 launch 方式下实测时好时坏：参数生效、订阅
   存在、QoS 兼容，却静默无输出（Node 与 ExecuteProcess 两种方式均复现，
   手动 ros2 run 则 100% 正常），不可依赖 -> 自研切片逻辑完全可控。

处理流程（每帧，numpy 向量化）：
    /livox/lidar (PointCloud2, frame: livox_frame, ~10Hz)
    -> stamp 重写为 本机时钟 now - 40ms（与 odom_tf_bridge 的 TF 同源）
    -> TF lookup livox_frame->base_link_level @stamp（去俯仰水平坐标系）
    -> 取 [min_height, max_height] 高度带，算极角/半径
    -> 同 bin 取最近点，生成 sensor_msgs/LaserScan -> /scan

QoS：订阅点云 SensorData（BEST_EFFORT，兼容驱动 RELIABLE 发布）；
发布 SensorData（BEST_EFFORT，兼容 slam_toolbox 的 BEST_EFFORT 订阅）。
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def quat_to_rotmat(x, y, z, w):
    """四元数 -> 3x3 旋转矩阵（numpy）。"""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class LivoxScanNode(Node):

    def __init__(self):
        super().__init__('livox_scan_node')
        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('target_frame', 'base_link_level')
        self.declare_parameter('min_height', -0.15)
        self.declare_parameter('max_height', 0.35)
        self.declare_parameter('range_min', 0.55)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scan_bins', 720)  # 0.5°/bin
        self.declare_parameter('publish_inf', True)

        gp = self.get_parameter
        self.target = gp('target_frame').value
        self.zmin = gp('min_height').value
        self.zmax = gp('max_height').value
        self.rmin = gp('range_min').value
        self.rmax = gp('range_max').value
        self.bins = int(gp('scan_bins').value)
        self.use_inf = gp('publish_inf').value

        self.tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        TransformListener(self.tf_buf, self)

        # 发布 RELIABLE：/scan 仅 3KB/帧，走无线中继时其中继订阅端固定
        # RELIABLE（BEST_EFFORT pub 与之不匹配会静默收不到）；笔记本端
        # slam_toolbox 的 BEST_EFFORT 订阅与 RELIABLE pub 兼容
        self.pub = self.create_publisher(
            LaserScan, gp('scan_topic').value,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            PointCloud2, gp('lidar_topic').value, self.cloud_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'livox_scan: {gp("lidar_topic").value} -> {gp("scan_topic").value} '
            f'(stamp 用本机时钟, target={self.target}, '
            f'z[{self.zmin},{self.zmax}] r[{self.rmin},{self.rmax}] bins={self.bins})')

    def cloud_cb(self, msg: PointCloud2):
        # stamp 直接用本机时钟（与 odom_tf_bridge 重打后的 TF 同源）。
        # 回退 40ms 安全余量：TF 帧经无线中继到达有传输/调度延迟（实测抖动
        # 5~15ms，余量小于抖动时查询 stamp 超前最新 TF 帧，报
        # extrapolation-into-future 丢帧）；0.5m/s 车速下 40ms 位置偏差仅
        # 2cm，AMCL/建图可忽略。
        t = (self.get_clock().now() - Duration(seconds=0.04)).to_msg()

        # TF：livox_frame -> target（去俯仰水平坐标系），按 scan 时间戳精确查询
        try:
            tf = self.tf_buf.lookup_transform(self.target, msg.header.frame_id, t)
        except Exception as e:
            self.get_logger().warn(f'TF lookup {msg.header.frame_id}->{self.target} fail: {e}',
                                   throttle_duration_sec=2.0)
            return
        tr = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        tvec = np.array([tr.x, tr.y, tr.z])

        # 解析点云（只取 x/y/z，避免逐点迭代）
        offs = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        buf = bytes(msg.data)
        n_pts = len(buf) // step
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [np.float32] * 3,
                       'offsets': [offs['x'], offs['y'], offs['z']],
                       'itemsize': step})
        pts = np.frombuffer(buf, dtype=dt, count=n_pts)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)

        # 变换到 target 坐标系：p' = R @ p + t
        tf_pts = xyz @ R.T + tvec

        # 高度带过滤 -> 极坐标
        mask = (tf_pts[:, 2] >= self.zmin) & (tf_pts[:, 2] <= self.zmax)
        hx, hy = tf_pts[mask, 0], tf_pts[mask, 1]
        rng = np.hypot(hx, hy)
        valid = (rng > self.rmin) & (rng < self.rmax)
        rng, ang = rng[valid], np.arctan2(hy[valid], hx[valid])

        # 投影到 bins，同 bin 取最近点
        inc = 2.0 * math.pi / self.bins
        idx = ((ang + math.pi) / inc).astype(np.int64) % self.bins
        ranges = np.full(self.bins, np.inf if self.use_inf else self.rmax, dtype=np.float32)
        np.minimum.at(ranges, idx, rng.astype(np.float32))

        out = LaserScan()
        out.header.stamp = t
        out.header.frame_id = self.target
        out.angle_min = -math.pi
        out.angle_max = math.pi
        out.angle_increment = inc
        out.time_increment = 0.0
        out.scan_time = 0.1
        out.range_min = self.rmin
        out.range_max = self.rmax
        out.ranges = ranges.tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
