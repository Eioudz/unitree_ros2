#!/usr/bin/env python3
"""假设：正前方那片"随距离升高"的点其实是地面。
若真如此，说明 livox_frame->base_link_level 之间少了约 -atan(0.226)≈-12.7° 的俯仰，
即 base_link_level 并未真正水平（或静态 TF 的俯仰未补偿）。

本脚本：对每帧前方扇区点拟合 z = a*r + b，打印斜率 a；
再用 -atan(a) 的反向旋转作用于整帧点云，看地面是否拉平。
只订阅不发布。
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Fit(Node):
    def __init__(self):
        super().__init__('fit')
        self.n = 0
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb,
                                 qos_profile_sensor_data)

    def cb(self, msg):
        self.n += 1
        if self.n > 5:
            return
        offs = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [np.float32] * 3,
                       'offsets': [offs['x'], offs['y'], offs['z']],
                       'itemsize': msg.point_step})
        n = len(msg.data) // msg.point_step
        p = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
        x, y, z = p['x'].astype(np.float64), p['y'].astype(np.float64), p['z'].astype(np.float64)
        r = np.hypot(x, y)
        ang = np.degrees(np.arctan2(y, x))

        # 前方扇区 + 之前的 z 带
        front = (np.abs(ang) < 30) & (r > 1.0) & (r < 3.0) & (z > -0.15) & (z < 0.35)
        # 侧方对照扇区（±90° 附近），同一 z 带
        side = (np.abs(np.abs(ang) - 90) < 30) & (r > 1.0) & (r < 3.0) & (z > -0.15) & (z < 0.35)
        # 后方对照
        back = (np.abs(ang) > 150) & (r > 1.0) & (r < 3.0) & (z > -0.15) & (z < 0.35)

        print(f'--- frame {self.n}')
        for name, m in (('front', front), ('side', side), ('back', back)):
            if m.sum() < 20:
                print(f'  {name:5s}: only {m.sum()} pts in band')
                continue
            a, b = np.polyfit(r[m], z[m], 1)
            resid = z[m] - (a * r[m] + b)
            print(f'  {name:5s}: n={m.sum():4d}  z = {a:+.3f}*r + {b:+.3f}  '
                  f'pitch={math.degrees(math.atan(a)):+.1f}deg  '
                  f'resid_std={resid.std():.3f}m  r=[{r[m].min():.2f},{r[m].max():.2f}]')


def main():
    rclpy.init()
    n = Fit()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
