#!/usr/bin/env python3
"""临时诊断：检查 /livox/lidar 正前方 1~3m 点云的属性。

目的：区分「真实回波」与「无回波(tag=0)占位点 / 陈旧点 / 中继伪造点」。
输出每帧：前方扇区点数、tag 分布、line 分布、intensity 分布、
点自带 timestamp 的跨度、以及 xyz 是否退化(全 0 / NaN / 定值)。
同时对比 /utlidar/cloud_deskewed（狗端直出）做交叉验证。
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def parse(msg):
    offs = {f.name: f.offset for f in msg.fields}
    names, formats, offsets = ['x', 'y', 'z'], [np.float32] * 3, \
        [offs['x'], offs['y'], offs['z']]
    for extra, fmt in (('intensity', np.float32), ('tag', np.uint8),
                       ('line', np.uint8), ('timestamp', np.float64)):
        if extra in offs:
            names.append(extra)
            formats.append(fmt)
            offsets.append(offs[extra])
    dt = np.dtype({'names': names, 'formats': formats,
                   'offsets': offsets, 'itemsize': msg.point_step})
    n = len(msg.data) // msg.point_step
    return np.frombuffer(bytes(msg.data), dtype=dt, count=n)


def col(p, name, n):
    """取列；字段不存在时返回全 0，便于统一打印。"""
    return p[name] if name in (p.dtype.names or ()) else np.zeros(n)


class Probe(Node):
    def __init__(self):
        super().__init__('front_probe')
        self.create_subscription(PointCloud2, '/livox/lidar',
                                 lambda m: self.cb(m, 'livox'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/utlidar/cloud_deskewed',
                                 lambda m: self.cb(m, 'utlidar'), qos_profile_sensor_data)
        self.n = {'livox': 0, 'utlidar': 0}
        self.get_logger().info('probing /livox/lidar + /utlidar/cloud_deskewed')

    def cb(self, msg, tag):
        self.n[tag] += 1
        if self.n[tag] > 6:          # 只看前几帧，避免刷屏
            return
        p = parse(msg)
        n = len(p)
        tagc, linec = col(p, 'tag', n), col(p, 'line', n)
        intc, tsc = col(p, 'intensity', n), col(p, 'timestamp', n)
        finite = np.isfinite(p['x']) & np.isfinite(p['y']) & np.isfinite(p['z'])
        r = np.hypot(p['x'], p['y'])
        ang = np.degrees(np.arctan2(p['y'], p['x']))
        front = finite & (np.abs(ang) < 30) & (r > 1.0) & (r < 3.0)
        all_front = (np.abs(ang) < 30) & (r > 1.0) & (r < 3.0)

        print(f'\n=== {tag} #{self.n[tag]}  frame={msg.header.frame_id} '
              f'pts={len(p)}  finite={finite.sum()}  front(1-3m,±30°)={front.sum()} '
              f'front_any={all_front.sum()}')
        if front.sum():
            ft, fl = tagc[front], linec[front]
            fi, fs = intc[front], tsc[front]
            tags, cnts = np.unique(ft, return_counts=True)
            lines, lcnts = np.unique(fl, return_counts=True)
            print('  tag  :', dict(zip(tags.tolist(), cnts.tolist())))
            print('  line :', dict(zip(lines.tolist(), lcnts.tolist())))
            print(f'  intensity: min={fi.min():.0f} med={np.median(fi):.0f} '
                  f'max={fi.max():.0f}')
            print(f'  ts span : {fs.max() - fs.min():.1f} '
                  f'(单位随驱动, min={fs.min():.1f} max={fs.max():.1f})')
            print(f'  r  : {np.percentile(r[front], [5, 50, 95]).round(2)}')
            print(f'  z  : {np.percentile(p["z"][front], [5, 50, 95]).round(2)}')
            # 与"真实物体"对照：整帧非前方点的 tag 分布
            back = finite & ~front
            if back.sum():
                bt, bc = np.unique(tagc[back], return_counts=True)
                print('  tag(其余区域):', dict(zip(bt.tolist(), bc.tolist())))
        # 非有限值/零点的总体情况
        zero = np.count_nonzero((p['x'] == 0) & (p['y'] == 0) & (p['z'] == 0))
        nan = np.count_nonzero(~finite)
        print(f'  zero-points={zero}  nan/inf={nan}  total={len(p)}')


def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
