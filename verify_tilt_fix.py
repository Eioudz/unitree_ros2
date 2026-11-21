#!/usr/bin/env python3
"""离线验证 pitch=+13° 修正：
不改运行栈，从 /livox/lidar 取一帧，分别用「旧 TF（零旋转）」和「新 TF（pitch=+13°）」
把点变到 base_link，各切 z[-0.15,0.35]，统计前方扇区剩余点数。
期望：旧 TF 下前方 ~500 点，新 TF 下前方 ~0（地面被压到 z≈-0.29 滤掉）。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def rot_pitch(p):
    c, s = math.cos(p), math.sin(p)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class G(Node):
    def __init__(self):
        super().__init__('g')
        self.msg = None
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb,
                                 qos_profile_sensor_data)

    def cb(self, m):
        if self.msg is None:
            self.msg = m


def main():
    rclpy.init()
    n = G()
    while n.msg is None:
        rclpy.spin_once(n, timeout_sec=2.0)
    msg = n.msg
    n.destroy_node()
    rclpy.try_shutdown()

    offs = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [np.float32] * 3,
                   'offsets': [offs['x'], offs['y'], offs['z']],
                   'itemsize': msg.point_step})
    cnt = len(msg.data) // msg.point_step
    p = np.frombuffer(bytes(msg.data), dtype=dt, count=cnt)
    pts = np.stack([p['x'], p['y'], p['z']], axis=1).astype(np.float64)

    for name, R in (('old (identity)', np.eye(3)),
                    ('new (pitch=+13deg)', rot_pitch(math.radians(13)))):
        # 注：base_link 在 livox 上方 0.10m，所以 base 系点 = R@livox_pt + (0,0,0.10)
        tf = pts @ R.T + np.array([0, 0, 0.10])
        z = tf[:, 2]
        x, y = tf[:, 0], tf[:, 1]
        r = np.hypot(x, y)
        ang = np.degrees(np.arctan2(y, x))
        band = (z > -0.15) & (z < 0.35)
        front = band & (np.abs(ang) < 30) & (r > 0.55) & (r < 12)
        near = front & (r < 3)
        print(f'{name}:')
        print(f'  总带内点={band.sum()}  前方带内点={front.sum()}  前方1-3m={near.sum()}')
        if front.sum():
            zf = z[front]
            rf = r[front]
            a, b = np.polyfit(rf, zf, 1)
            print(f'  前方 z = {a:+.3f}*r + {b:+.3f}  (斜率近 0 = 已水平)')
        # 地面分布
        ground = band & (z < -0.2)
        print(f'  带内地面点(z<-0.2)={ground.sum()}  带内立面点(|z|<0.15)={(band & (np.abs(z)<0.15)).sum()}')


if __name__ == '__main__':
    main()
