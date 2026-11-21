#!/usr/bin/env python3
"""把 /livox/lidar 整帧画成：俯视 XY（前方=+x）+ 侧视 XZ，并突出前方±30° 1~3m 扇区。
存 PNG，不发布任何话题，不影响现有栈。
"""
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Grab(Node):
    def __init__(self, n_frames=4):
        super().__init__('grab')
        self.frames = []
        self.n_frames = n_frames
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb,
                                 qos_profile_sensor_data)

    def cb(self, msg):
        if len(self.frames) >= self.n_frames:
            return
        offs = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': [np.float32] * 3,
                       'offsets': [offs['x'], offs['y'], offs['z']],
                       'itemsize': msg.point_step})
        n = len(msg.data) // msg.point_step
        p = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
        self.frames.append(np.stack([p['x'], p['y'], p['z']], axis=1))


def main():
    rclpy.init()
    n = Grab(n_frames=4)
    while len(n.frames) < 4:
        rclpy.spin_once(n, timeout_sec=2.0)
    n.destroy_node()
    rclpy.try_shutdown()

    fig, axes = plt.subplots(len(n.frames), 2, figsize=(14, 4 * len(n.frames)))
    for i, pts in enumerate(n.frames):
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        r = np.hypot(x, y)
        ang = np.arctan2(y, x)
        front = (np.abs(ang) < math.radians(30)) & (r > 1.0) & (r < 3.0)
        near = r < 8

        ax = axes[i, 0]
        ax.scatter(x[near & ~front], y[near & ~front], s=1, c='gray', alpha=.4)
        ax.scatter(x[front], y[front], s=4, c='red')
        ax.scatter([0], [0], c='blue', marker='^', s=60)
        ax.set_xlim(-1, 8); ax.set_ylim(-4, 4); ax.set_aspect('equal')
        ax.set_title(f'frame {i}: XY top view (+x forward), red=front 1-3m ±30°')
        ax.grid(alpha=.3)

        ax = axes[i, 1]
        ax.scatter(r[near & ~front], z[near & ~front], s=1, c='gray', alpha=.4)
        ax.scatter(r[front], z[front], s=4, c='red')
        ax.set_xlim(0, 8); ax.set_ylim(-2, 5)
        ax.set_title('XZ side view (r vs z)')
        ax.axhline(0, color='k', lw=.5)
        ax.grid(alpha=.3)

    plt.tight_layout()
    out = '/home/soyo/unitree_ros2/front_cloud.png'
    plt.savefig(out, dpi=90)
    print('saved', out)


if __name__ == '__main__':
    main()
