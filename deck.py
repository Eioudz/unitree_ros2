import math
import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from unitree_api.msg import Request


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # ---------------- 参数 ----------------
        self.declare_parameter('cloud_topic', '/utlidar/cloud_base')
        self.declare_parameter('max_distance', 1.5)
        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('half_angle_deg', 45.0)
        self.declare_parameter('point_threshold', 7)
        self.declare_parameter('sector_threshold', 3)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('sector_deg', 15.0)
        self.declare_parameter('cluster_gap_deg', 5.0)
        self.declare_parameter('near_band', 0.3)

        # ---------------- 控制参数 ----------------
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('angle_deadband_deg', 3.0)

        topic = self.get_parameter('cloud_topic').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.half_angle = math.radians(
            self.get_parameter('half_angle_deg').value)
        self.point_threshold = self.get_parameter('point_threshold').value
        self.sector_threshold = self.get_parameter('sector_threshold').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.sector_deg = self.get_parameter('sector_deg').value
        self.cluster_gap_deg = self.get_parameter('cluster_gap_deg').value
        self.near_band = self.get_parameter('near_band').value

        request_topic = self.get_parameter('request_topic').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.angle_deadband = math.radians(
            self.get_parameter('angle_deadband_deg').value)

        half_deg = math.degrees(self.half_angle)
        self.sector_edges = np.arange(
            -half_deg, half_deg + self.sector_deg, self.sector_deg)

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(
            PointCloud2, topic, self.cloud_callback, qos)

        request_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_request = self.create_publisher(
            Request, request_topic, request_qos)

        self.get_logger().info(
            f'订阅: {topic}, 发布: {request_topic}, '
            f'Kp={self.kp_angular}, max_wz={self.max_angular_z}')

    # ============================================================
    def cloud_callback(self, msg: PointCloud2):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if buf.size == 0:
            return
        pts_raw = buf.reshape(-1, msg.point_step)
        x = pts_raw[:, 0:4].copy().view(np.float32).flatten()
        y = pts_raw[:, 4:8].copy().view(np.float32).flatten()
        z = pts_raw[:, 8:12].copy().view(np.float32).flatten()
        total = len(x)
        if total == 0:
            return

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]
        if x.size == 0:
            return

        dist = np.sqrt(x**2 + y**2 + z**2)
        angle = np.arctan2(y, x)

        mask = (
            (dist > self.min_distance) &
            (dist < self.max_distance) &
            (x > 0) &
            (np.abs(angle) < self.half_angle) &
            (z > self.min_height) &
            (z < self.max_height)
        )
        count = int(np.count_nonzero(mask))

        # ---- 没检测到：发送停止指令 ----
        if count == 0:
            self._publish_move(0.0)
            self.get_logger().info(
                f'总:{total} ROI:0 前方安全',
                throttle_duration_sec=2.0)
            return

        roi_angle = angle[mask]
        roi_dist = dist[mask]

        # ---- 扇区统计（日志） ----
        near_limit = roi_dist.min() + self.near_band
        near_sel = roi_dist <= near_limit
        near_angle = roi_angle[near_sel]

        hist, edges = np.histogram(near_angle, bins=self.sector_edges)
        centers = (edges[:-1] + edges[1:]) / 2.0
        bar = ''.join('#' if c > self.sector_threshold else
                      ('+' if c > 0 else '.') for c in hist)
        self.get_logger().info(
            f'总:{total} ROI:{count} 最近:{roi_dist.min():.2f}m '
            f'扇区[{bar}]',
            throttle_duration_sec=1.0)

        # ---- 计算目标角度（最近一批点的角度中位数） ----
        target_angle = float(np.median(near_angle))
        if abs(target_angle) < self.angle_deadband:
            target_angle = 0.0

        # ---- 计算并发布角速度 ----
        wz = -self.kp_angular * target_angle
        wz = max(-self.max_angular_z, min(self.max_angular_z, wz))
        self._publish_move(wz)

        angle_deg = math.degrees(target_angle)
        side = '左' if angle_deg > 5 else ('右' if angle_deg < -5 else '正')
        self.get_logger().info(
            f'ROI:{count} 最近:{roi_dist.min():.2f}m '
            f'目标角度:{angle_deg:+.1f}°({side}) '
            f'wz:{wz:+.2f}rad/s',
            throttle_duration_sec=0.5)

    # ============================================================
    def _publish_move(self, vyaw: float):
        """
        发布 Move 指令到 /api/sport/request。
        api_id = 1008，参数格式：{"x": vx, "y": vy, "z": vyaw}
        """
        req = Request()
        req.header.identity.api_id = 1008
        req.header.identity.id = 0
        req.parameter = json.dumps({
            "x": 0.0,
            "y": 0.0,
            "z": float(vyaw)
        })
        self.pub_request.publish(req)


# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()