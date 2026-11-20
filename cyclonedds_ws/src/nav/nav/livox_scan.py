#!/usr/bin/env python3
"""
Mid360 点云切片节点（时间戳对齐 + 水平切片二合一，替代 livox_time_bridge +
pointcloud_to_laserscan 组合）。

为什么自己写：
1. 时钟域问题：/utlidar/robot_odom（TF 源）的 stamp 是狗主控时钟域，与笔记本
   不同步时 AMCL/Nav2/RViz 用本机时钟查 TF 必挂 -> odom_tf_bridge 与本节点
   统一把 stamp 重打为本机时钟，全链路（TF/scan/AMCL/Nav2）同源自洽。
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
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
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
        # ---- 凑帧策略（两种，按链路密度自动选，launch 里配）----
        # aggregate_mode:
        #   'coverage'  组内累积，带内回波 bin 比例达 aggregate_min_coverage
        #               才发布（高频点云用：有线 2058Hz，38ms 就凑满）
        #   'sliding'   滑动窗口，每条 scan 都包含最近 aggregate_window_sec
        #               的数据（低频点云用：中继 ~7000 点/秒，凑帧永远凑不满，
        #               固定 0.5s 兜底会退化成 2Hz 且重叠只有 44%）
        # 两种模式的共同目标：让相邻两条 /scan 高度重叠。重叠率是闪烁的唯一
        # 判据（RViz 每帧擦掉重画，重叠 <70% 就肉眼可见地闪）。
        self.declare_parameter('aggregate_mode', 'coverage')
        # coverage 模式：带内回波 bin 比例阈值。计数口径必须是"高度带内、
        # 量程内"的回波，不能数所有点的方位（地面/远点铺满 360° 会让计数
        # 虚高，几十毫秒就发布、实际输出只有小半圈 -> 闪烁）。
        self.declare_parameter('aggregate_min_coverage', 0.70)
        # coverage 模式：组龄兜底，超时未凑满也发布并重开。
        # sliding 模式：滑动窗口时长（每条 scan 覆盖最近这么多秒的数据）。
        self.declare_parameter('aggregate_window_sec', 0.5)
        # sliding 模式：固定发布频率。低频链路下必须固定频率发布，
        # 否则频率会随点云到达抖动。
        self.declare_parameter('sliding_publish_hz', 5.0)

        gp = self.get_parameter
        self.target = gp('target_frame').value
        self.zmin = gp('min_height').value
        self.zmax = gp('max_height').value
        self.rmin = gp('range_min').value
        self.rmax = gp('range_max').value
        self.bins = int(gp('scan_bins').value)
        self.use_inf = gp('publish_inf').value

        # 凑帧状态。诊断手册：docs/scan-rate-spike-diagnosis.md
        self.mode = str(gp('aggregate_mode').value)
        self.min_cover = float(gp('aggregate_min_coverage').value)
        self.agg_window = Duration(
            seconds=float(gp('aggregate_window_sec').value))
        self.pub_hz = float(gp('sliding_publish_hz').value)
        # coverage 模式：累积容器
        self.group_ranges = np.full(
            self.bins, np.inf if self.use_inf else self.rmax, dtype=np.float32)
        self.group_t0 = None  # 组龄计时起点（首片到达时刻）
        # sliding 模式：滑动窗口内的切片 [(到达时刻, 变换后的点)] + 下次发布时刻
        self.win = []
        self.next_pub = None

        # 点云回调放独立 callback group：本节点用 MultiThreadedExecutor，
        # 点云回调（2 万点 numpy 运算）与 /tf 回调若共用默认的互斥组，会把
        # TF 堵在队列后面，buffer 相对墙钟滞后可达数百 ms -> 按 now-40ms 查
        # TF 报 extrapolation-into-the-future，整帧点云被丢弃。
        # 实测症状：/livox/lidar 5Hz 而 /scan 只有 1.4Hz（warn 被 throttle 2s
        # 掩盖，看着像偶发，实际是每帧都失败）；下游 AMCL 收不到扫描 ->
        # map->odom 不更新 -> 代价地图冻结 -> DWB 每帧同一条轨迹 -> 原地绕圈。
        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        TransformListener(self.tf_buf, self)

        # 发布 BEST_EFFORT（2026-11-19 修）：此前是 RELIABLE/depth=10，只要
        # 出现一个慢订阅者（ros2 topic hz、RViz 的 LaserScan 显示、被反压的
        # 代价地图），CycloneDDS 就会对 RELIABLE writer 施加反压：publish()
        # 阻塞、消息在 writer 队列里排队，等真正送达时 header.stamp 已经旧了
        # 好几秒。AMCL 的 MessageFilter 拿到这种陈旧 scan 直接丢弃
        # （"the timestamp on the message is earlier than all the data in the
        # transform cache"）-> map->odom 停止更新 -> controller_server 刷
        # "Transform data too old when converting from map to odom" -> DWB 拿
        # 不到机器人位姿、一帧 /cmd_vel 都不发 -> 狗不动。RViz 侧同源症状是
        # "Message Filter dropping message: frame 'odom' ... queue is full"。
        # 实测对照：RELIABLE 订阅 /scan 只有 0.45Hz，BEST_EFFORT 订阅 4.8Hz
        # （= 点云输入频率），且 BEST_EFFORT 下 scan.stamp 不再冻结。
        # 传感器话题本就该"宁可丢旧帧，也不排队"，故改为 BEST_EFFORT/depth=5。
        # 现网所有订阅端（amcl、local_costmap、RViz、slam_toolbox）均为
        # BEST_EFFORT，与之匹配；若将来有 RELIABLE 订阅端需另行调整。
        self.pub = self.create_publisher(
            LaserScan, gp('scan_topic').value,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            PointCloud2, gp('lidar_topic').value, self.cloud_cb,
            qos_profile_sensor_data, callback_group=self.cloud_group)

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
        now = self.get_clock().now()
        t = (now - Duration(seconds=0.04)).to_msg()

        # TF：livox_frame -> target（去俯仰水平坐标系），按 scan 时间戳精确查询。
        # 精确时刻查不到时（TF 尚未到达 / 已滑出 buffer）退到"最新可用变换"，
        # 而不是丢弃整帧：0.5m/s 下车速下几帧的位姿偏差 < 数 cm，远小于
        # range_min 0.55 的自点屏蔽余量，但丢帧会让 /scan 掉到 1/5 频率，
        # 进而拖垮 AMCL->map->odom->costmap 整条链。
        try:
            tf = self.tf_buf.lookup_transform(self.target, msg.header.frame_id, t)
        except Exception as e:
            try:
                tf = self.tf_buf.lookup_transform(
                    self.target, msg.header.frame_id, rclpy.time.Time())
                self.get_logger().warn(
                    f'TF exact lookup failed ({type(e).__name__}), '
                    f'used latest transform instead', throttle_duration_sec=5.0)
            except Exception as e2:
                self.get_logger().warn(
                    f'TF lookup {msg.header.frame_id}->{self.target} fail: {e2}',
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

        # 变换到 target 坐标系：p' = R @ p + t（每片按各自到达时刻变换）
        tf_pts = xyz @ R.T + tvec

        # 高度带过滤 -> 极坐标 -> 投影到 bins（同 bin 取最近点）。
        # 未过滤方向保持 inf：那是"看过但带内无障碍"= 真实自由空间，
        # costmap 据此清理是正确行为。
        inc = 2.0 * math.pi / self.bins
        mask = (tf_pts[:, 2] >= self.zmin) & (tf_pts[:, 2] <= self.zmax)
        hx, hy = tf_pts[mask, 0], tf_pts[mask, 1]
        rng = np.hypot(hx, hy)
        valid = (rng > self.rmin) & (rng < self.rmax)
        rng, ang = rng[valid], np.arctan2(hy[valid], hx[valid])
        idx = ((ang + math.pi) / inc).astype(np.int64) % self.bins

        if self.mode == 'sliding':
            self._sliding(now, idx, rng, inc)
            return

        # ---- coverage 模式：累积到达标即发 ----
        np.minimum.at(self.group_ranges, idx, rng.astype(np.float32))
        if self.group_t0 is None:
            self.group_t0 = now

        # 覆盖率 = 带内真有回波的 bin 数（= group_ranges 里的有限值个数）。
        # 必须按"输出的内容"计数：早期版本数的是所有点的方位（地面+远点铺满
        # 360°，几十毫秒就凑满），于是每条 scan 只有小半圈有回波，相邻两条
        # 重叠仅 31%，RViz 逐帧重绘 -> 一闪一闪。
        covered = int(np.isfinite(self.group_ranges).sum()) if self.use_inf \
            else int((self.group_ranges < self.rmax).sum())

        # 发布：带内覆盖率达 min_coverage（丢片/空窗时自动跨帧凑满），
        # 或组龄超 agg_window 兜底（防持续遮挡导致永远凑不满）。
        if (covered >= self.min_cover * self.bins
                or (now - self.group_t0) >= self.agg_window):
            self._publish(now, self.group_ranges, inc)
            self.group_ranges = np.full(
                self.bins, np.inf if self.use_inf else self.rmax,
                dtype=np.float32)
            self.group_t0 = None

    def _sliding(self, now, idx, rng, inc):
        """滑动窗口模式：每条 scan 都覆盖最近 agg_window 秒的全部数据。

        低频链路（WiFi 中继 ~7000 点/秒）下，coverage 模式永远凑不满阈值，
        只能等满超时兜底 -> 2Hz 且相邻重叠仅 44%（仍然闪）。滑动窗口让相邻
        两条共享窗口内绝大部分数据，重叠率天然很高（实测 T=0.6s/5Hz -> 73%），
        且频率固定不随点云到达抖动。
        代价：移动时窗口内包含数百毫秒前的点，会有拖影；由 costmap 的
        inflation_radius 和 AMCL 的 update_min_d 兜底，中继链路本就做不到高频。
        """
        self.win.append((now, idx, rng.astype(np.float32)))
        # 丢弃滑出窗口的切片
        cutoff = now - self.agg_window
        while self.win and self.win[0][0] < cutoff:
            self.win.pop(0)

        if self.next_pub is None:
            self.next_pub = now
        if now < self.next_pub:
            return
        self.next_pub = now + Duration(seconds=1.0 / self.pub_hz)

        ranges = np.full(
            self.bins, np.inf if self.use_inf else self.rmax, dtype=np.float32)
        for _, i, r in self.win:
            np.minimum.at(ranges, i, r)
        self._publish(now, ranges, inc)

    def _publish(self, now, ranges, inc):
        # scan.stamp 贴近发布时刻而非组首片时刻：obstacle_layer 按
        # scan.stamp 查 odom->base_link_level，stamp 旧了会超出 TF
        # buffer 回溯范围丢帧（extrapolate-into-past）；贴发布时刻与
        # odom TF 对齐。移动时点云几何与 stamp 的错位 <0.4s*v
        # （0.5m/s 时 <0.2m），由 costmap inflation_radius 兜底。
        out = LaserScan()
        out.header.stamp = (now - Duration(seconds=0.04)).to_msg()
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
    # 多线程执行器：点云回调在 cloud_group 里跑，/tf 回调在默认组里跑，
    # 两者不再互相排队（见 __init__ 中 cloud_group 的说明）
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
