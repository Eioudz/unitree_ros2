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

        # 点云处理参数
        self.declare_parameter('cloud_topic', '/utlidar/cloud_base')
        self.declare_parameter('max_distance', 1.7)
        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('half_angle_deg', 60.0)
        self.declare_parameter('point_threshold', 7)
        self.declare_parameter('sector_threshold', 3)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('sector_deg', 20.0)
        self.declare_parameter('cluster_gap_deg', 5.0)
        self.declare_parameter('near_band', 0.3)

        # 控制参数
        self.declare_parameter('request_topic', '/api/sport/request')
        self.declare_parameter('kp_angular', 1.2)
        self.declare_parameter('max_angular_z', 1.2)
        self.declare_parameter('angle_deadband_deg', 3.0)
        self.declare_parameter('forward_speed', 0.4)      # 前进线速度 m/s
        self.declare_parameter('forward_start_dist', 1.2) # 超过此距离开始前进
        self.declare_parameter('forward_stop_dist', 0.9)  # 低于此距离停止前进

        # 读取参数
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
        self.forward_speed = self.get_parameter('forward_speed').value
        self.forward_start_dist = self.get_parameter('forward_start_dist').value
        self.forward_stop_dist = self.get_parameter('forward_stop_dist').value

        half_deg = math.degrees(self.half_angle)
        self.sector_edges = np.arange(
            -half_deg, half_deg + self.sector_deg, self.sector_deg)

        # QoS
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
            f'Kp={self.kp_angular}, max_wz={self.max_angular_z}, '
            f'前进速度={self.forward_speed}, '
            f'前进区间[{self.forward_stop_dist},{self.forward_start_dist}]m')

    def cloud_callback(self, msg: PointCloud2):
        #  1. 抽 x/y/z 
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

        #  2. 距离 / 角度 
        dist = np.sqrt(x**2 + y**2 + z**2)
        angle = np.arctan2(y, x)

        #  3. ROI 过滤 
        mask = (
            (dist > self.min_distance) &
            (dist < self.max_distance) &
            (x > 0) &
            (np.abs(angle) < self.half_angle) &
            (z > self.min_height) &
            (z < self.max_height)
        )
        count = int(np.count_nonzero(mask))

        #  4. 没检测到：停止 
        if count == 0:
            self._publish_move(0.0, 0.0)
            self.get_logger().info(
                f'总:{total} ROI:0 前方安全',
                throttle_duration_sec=2.0)
            return

        roi_angle = angle[mask]
        roi_dist = dist[mask]
        nearest = float(roi_dist.min())

        #  5. 扇区统计日志 
        near_limit = nearest + self.near_band
        near_sel = roi_dist <= near_limit
        near_angle = roi_angle[near_sel]

        hist, edges = np.histogram(near_angle, bins=self.sector_edges)
        bar = ''.join('#' if c > self.sector_threshold else
                      ('+' if c > 0 else '.') for c in hist)
        self.get_logger().info(
            f'总:{total} ROI:{count} 最近:{nearest:.2f}m 扇区[{bar}]',
            throttle_duration_sec=1.0)

        #  6. 计算目标角度 
        target_angle = float(np.median(near_angle))
        if abs(target_angle) < self.angle_deadband:
            target_angle = 0.0

        #  7. 计算角速度（目标在左 angle>0 → z 正 = 左转，与 ROS/固件约定一致）
        wz = self.kp_angular * target_angle
        wz = max(-self.max_angular_z, min(self.max_angular_z, wz))

        #  8. 计算线速度：>1m 前进，0.8~1m 线性减速，<0.8m 停 
        if nearest > self.forward_start_dist:
            vx = self.forward_speed
        elif nearest > self.forward_stop_dist:
            ratio = ((nearest - self.forward_stop_dist) /
                     (self.forward_start_dist - self.forward_stop_dist))
            vx = self.forward_speed * ratio
        else:
            vx = 0.0

        #  9. 发布 
        self._publish_move(vx, wz)

        angle_deg = math.degrees(target_angle)
        side = '左' if angle_deg > 5 else ('右' if angle_deg < -5 else '正')
        self.get_logger().info(
            f'ROI:{count} 最近:{nearest:.2f}m '
            f'目标角度:{angle_deg:+.1f}°({side}) '
            f'vx:{vx:+.2f} wz:{wz:+.2f}',
            throttle_duration_sec=0.5)


    def _publish_move(self, vx: float, vyaw: float):

        req = Request()
        req.header.identity.api_id = 1008  
        req.header.identity.id = 0
        req.header.lease.id = 0
        req.header.policy.noreply = True
        req.parameter = json.dumps({
            "x": float(vx),
            "y": 0.0,
            "z": float(vyaw)
        })
        self.pub_request.publish(req)


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

