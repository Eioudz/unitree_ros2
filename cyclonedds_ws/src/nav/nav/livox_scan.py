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
        # 多帧聚合窗口：0 = 不聚合（整帧驱动）；>0 把"小切片突发"凑回整帧。
        # launch 固化为 0.2（狗端驱动随时可能回退切片模式，见文件头说明）
        self.declare_parameter('aggregate_window_sec', 0.0)

        gp = self.get_parameter
        self.target = gp('target_frame').value
        self.zmin = gp('min_height').value
        self.zmax = gp('max_height').value
        self.rmin = gp('range_min').value
        self.rmax = gp('range_max').value
        self.bins = int(gp('scan_bins').value)
        self.use_inf = gp('publish_inf').value

        # 切片聚合状态：狗端 Mid360 驱动有两种发送模式，且会来回回退：
        #   - 整帧直发：~5.3Hz，227KB/帧
        #   - 小切片突发：~600Hz，60 片/帧（每片 ~6°），按 ~130ms 簇到达
        # 切片模式下逐片成 scan 会让 RViz 逐扇区闪烁 + 下游 60 倍负载，
        # 故按窗口聚合后再发。窗口取值必须 >= 簇周期+簇时长（0.2s），
        # 否则窗口只撞上簇的零头 -> scan 只盖一小角扇区、其余 inf ->
        # costmap clearing 把盲区障碍抹掉。整帧模式下只是把相邻帧并一帧，无害。
        # 诊断手册：docs/scan-rate-spike-diagnosis.md
        self.agg_window = Duration(
            seconds=float(gp('aggregate_window_sec').value))
        self.agg_pts = []
        self.agg_n = 0        # 窗口内已累积点数
        self.agg_wall = None  # 窗口起始墙钟
        self.agg_cap = 60000  # 单窗点数上限（整帧 ~2 万点，防异常流量撑内存）

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

        # 聚合窗口（aggregate_window_sec，0 = 不聚合）：窗口内累积，
        # 到期拼整帧走后续流程。整帧驱动下窗口为 0，每帧立即处理。
        if self.agg_window.nanoseconds > 0:
            if self.agg_wall is None:
                self.agg_wall = now
            if self.agg_n < self.agg_cap:
                self.agg_pts.append(tf_pts)
                self.agg_n += tf_pts.shape[0]
            if (now - self.agg_wall) < self.agg_window:
                return
            tf_pts = np.concatenate(self.agg_pts)
        # scan 时间戳必须贴近"发布时刻"，而非聚合窗口首片。首片时刻是主机收到
        # 点云的时刻（经无线中继已滞后 ~0.3s），若用它作 stamp，obstacle_layer
        # 按 scan.stamp 查 odom->base_link_level 时会超出 TF buffer 回溯范围，
        # 报 extrapolate-into-past 丢帧 -> costmap 吃不到 /scan。
        # 贴发布时刻后 scan.stamp 与 odom TF 对齐，obstacle_layer 能查到 TF。
        # 移动场景下点云几何用"到达时 TF"，与最新 stamp 的错位 <0.4s*v
        # （0.5m/s 时 <0.2m），由 costmap inflation_radius 0.55 兜底。
        now = self.get_clock().now()
        stamp = (now - Duration(seconds=0.04)).to_msg()
        self.agg_pts, self.agg_n, self.agg_wall = [], 0, None

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
        out.header.stamp = stamp
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
