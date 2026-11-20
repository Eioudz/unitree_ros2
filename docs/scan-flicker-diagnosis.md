# RViz 点云一闪一闪：排查与修复（2026-11-20）

`ros2 launch nav localization.launch.py` 后，RViz 里 LaserScan 一直闪。
本文记录从定位到修复的全过程、每个数字的来源，以及踩过的四个坑。

**结论：闪烁不是 RViz 的问题，是 `/scan` 相邻两条的重叠率太低（31%）。**
根因是 `livox_scan.py` 覆盖率凑帧逻辑里的两个 bug：重复计数 + 计数口径与输出不一致。
修复后相邻重叠 86%，`/scan` 10.3Hz。

相关文档：
- `scan-rate-spike-diagnosis.md` —— `/scan` 频率异常（数百 Hz / 掉频）的排查手册
- `rviz-freeze-diagnosis.md` —— RViz 界面卡死（NVIDIA 驱动被 Secure Boot 挡住）

---

## 一、TL;DR

**两条链路的点云密度差 20 倍以上，必须用两套凑帧策略**（`aggregate_mode`，
launch 按 `ROS_DOMAIN_ID` 自动选）。

### 有线直连（domain 0，2058 Hz / 15.8 万点/秒）→ `coverage` 模式

| 指标 | 修复前 | 修复后 |
|---|---|---|
| `/scan` 频率 | 197 Hz | **10.3 Hz** |
| 每条 scan 有回波 bin（/720） | 中位 9.8%，57% 的 scan 不足 10% | **70%（最小 70%）** |
| **相邻两条重叠率** | **31%** | **86%** |
| 完全空白 scan | 有 | **0** |
| 凑帧阈值 | 0.95 | 0.70 |

### WiFi 中继（domain 1，~190 Hz / 7000 点/秒）→ `sliding` 模式

| 指标 | coverage 模式（凑帧） | sliding 模式（滑动窗口） |
|---|---|---|
| `/scan` 频率 | 2.0 Hz（每帧等满超时） | **5.0 Hz（固定）** |
| 每条有回波 bin | 40% | **58%** |
| **相邻两条重叠率** | **44%（仍然闪）** | **87%** |
| 完全空白 scan | 0 | 0 |

中继链路带内点只有约 1000 点/秒，物理上凑不出高覆盖率——所以改用滑动窗口：
每条 scan 都包含最近 0.6s 的数据，相邻两条天然共享绝大部分，重叠率上去了。
代价是移动时有轻微拖影，由 `inflation_radius` 兜底。

提交：
- `4701be3` 覆盖率重复计数 + 计数口径修正（coverage 模式）
- 后续：`aggregate_mode` 双模式 + launch 按 domain 自动选

---

## 二、先讲方法：判断闪烁只看两个指标

RViz 每收到一条 LaserScan 就擦掉上一条重画。所以**闪烁 = 相邻两条 scan
重叠的部分太少**，中间那些方位在帧与帧之间忽有忽无。

判定只需要两个数，不要先怀疑 RViz 显示设置：

1. `/scan` 频率（过高或过低都异常）
2. **相邻两条 scan 的重叠率**（交并比）

重叠率低于 ~70% 就会肉眼可见地闪。把这条当唯一验收标准，能省掉大量弯路。

探针脚本（可直接复用）：

```python
#!/usr/bin/env python3
"""量 /scan 的覆盖率与相邻重叠率。"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        self.S = []
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.cb, q)

    def cb(self, m):
        self.S.append(np.isfinite(np.array(m.ranges)))


rclpy.init()
p = Probe()
t0 = time.time()
while time.time() - t0 < 20:
    rclpy.spin_once(p, timeout_sec=0.2)

S = np.array(p.S)
n = len(S)
cnt = S.sum(axis=1)
ov = [(a & b).sum() / max((a | b).sum(), 1) for a, b in zip(S, S[1:])]
print('条数=%d (%.1f Hz)' % (n, n / 20))
print('每条覆盖: p50=%.0f%% min=%.0f%%' % (100 * np.median(cnt) / 720, 100 * cnt.min() / 720))
print('相邻重叠: p50=%.0f%% min=%.0f%%' % (100 * np.median(ov), 100 * np.min(ov)))
print('空白 scan: %d 条' % (cnt == 0).sum())
```

> 注意：订阅 `/scan` 必须用 **BEST_EFFORT**。发布端是 BEST_EFFORT/depth=5，
> 用 RELIABLE 订阅匹配不上（或匹配后反压发布端，反而改变被测对象）。

---

## 三、排查过程

### 3.1 第一层：点云本身太稀（WiFi 中继链路）

一开始的怀疑对象是"每条 scan 只画得出小半圈"。实测（WiFi / domain 1，经中继）：

```
/livox/lidar   143~219 Hz，每片仅 ~42 点，合计 ~6500 点/秒
```

Mid360 原厂是二十万点/秒量级。再看高度带内（`z∈[-0.15, 0.35]`）的点只占 **22%**，
于是每条 `/scan` 的有效回波 bin 中位数只有 143/720 = **20%**，还有 **13% 的 scan
完全空白**。

这时怀疑方向是"数据不够"，但先别急着改中继——继续往下查。

### 3.2 试 `/utlidar/cloud_base`（否决）

狗上 LIO 自己算好的点云，理论上比中继透传的更完整。实测：

```
/utlidar/cloud_base   13.7 Hz，727 点/帧，~9890 点/秒
```

点率确实高 1.5 倍，但**高度带内只占 6.7%**（几乎全在 z≈-0.35 的地面）。
单帧方位覆盖只有 4%，连续 5 帧累积才 17%。

用同样的切片逻辑起影子节点实测：

| | `/scan`（livox） | `/scan_utlidar` |
|---|---|---|
| 频率 | 7.8 Hz | **1.6 Hz** |
| 覆盖 | 18% | 25% |
| 空白 | 11% | 0% |
| 相邻重叠 | 11% | 39% |

更差。原因是这个点云以地面为主，切出来的高度带内回波太少，凑帧要等很久。
**结论：不用它。**

### 3.3 试改狗端中继关掉抽样（否决）

`docs/scan-rate-spike-diagnosis.md` 记过中继的 `RATE_SKIP` / `DOWNSAMPLE`。
这次连上狗看了实际配置：

```python
# /home/unitree/go2_relay/relay.py
RATE_SKIP  = {'/livox/lidar': 2}   # 每 2 帧转发 1 帧
DOWNSAMPLE = {'/livox/lidar': 2}   # 每 2 个点留 1 个
```

改成全 1（并把 `/livox/lidar` 从 `DOWNSAMPLE` 移出，走 raw 直通省 CPU），
重启中继后结果**更差**：

| | 改前 | 改后 |
|---|---|---|
| 狗 CPU load | ~4 | **21.6**（shard 2 打满 99.5%） |
| `/livox/lidar`（笔记本侧） | 143 Hz | **109 Hz** |
| 点数/秒 | 6500 | 8413 |

原因是驱动原始输出是 **2058 Hz / 15.8 万点/秒**，Python 中继每个 shard 是单线程
反序列化+转发的，扛不住这个消息率——CPU 打满后转发反而变慢。

**已完整回滚**（备份：`relay.py.bak.livoxfull.1795145575`）。
**结论：不要试图通过改中继拿全密度点云。**

### 3.4 改走有线直连 domain 0（成立）

插上网线后 `setup.sh` 自动选 `enp130s0` → **domain 0**。这时笔记本看到的是
驱动原始话题，不经过中继：

```
/livox/lidar   2058 Hz，每片 85 点，合计 158407 点/秒
```

比中继链路高 **24 倍**。凑满 95% 方位覆盖只需 **38ms**（≈26Hz）。

**但有个前提**：狗不发布 TF。`/tf_static`（`livox_static_tf`）和
`odom->base_link`（`odom_tf_bridge`）都是笔记本侧节点发的。所以**不能只把
`livox_scan` 切到 domain 0，必须整套导航栈都在 domain 0**。

```bash
source ~/unitree_ros2/setup.sh    # 网线插着时自动 ROS_DOMAIN_ID=0
ros2 launch nav localization.launch.py
```

### 3.5 全密度下，原来的 bug 被放大到 197Hz

换成 domain 0 之后，`/scan` 不但没变好，反而冲到了 **197 Hz**：

```
scans=3941 / 20s = 197 Hz
每条有回波 bin 中位 9.8%
57% 的 scan 不足 10%
相邻重叠 31%
```

到这里才明白：**点云密度不是主因，凑帧逻辑本身是坏的**。密度越高，坏得越明显。

---

## 四、两个 bug

### 4.1 重复计数

```python
# 错误写法（先数后置位）
self.group_covered_cnt += int((~self.group_covered[idx_seen]).sum())
self.group_covered[idx_seen] = True
```

同一片点云里多个点落在同一个 bin 时，这个 bin 被**重复计数**。
全密度下每片 85 个点挤在几个相邻 bin，计数瞬间虚高冲过阈值 → 每片都发 → 197Hz。

复现实验（订阅 `/livox/lidar`，模拟节点逻辑）：

```
节点写法触发 98 次 / 15s = 6.5 Hz
触发时真实覆盖 bin 数: mean=568/720 = 78.9%   （阈值是 95%）
```

即：**计数说"够了"，实际只覆盖 79%**。

### 4.2 计数口径与输出不一致（更隐蔽）

覆盖率统计的是"**所有点**的方位"，包含：

- 地面回波（俯仰 -20° 下缘打地，0.8~2.4m 一圈，几乎每个方位都有）
- 5m 外的远点

这两类几乎铺满 360°，所以计数 38ms 就凑满。但 `/scan` **只输出高度带内、
量程内的回波**（只占全部点的 13.8%）。于是：

> 计数说"覆盖 95%"，实际输出的 scan 只有 44% 的方位有东西。

叠加 4.1 的重复计数，最终结果就是每条 scan 只有小半圈、相邻两条重叠 31% → 闪烁。

### 4.3 修法

```python
# 只统计"会真正输出"的 bin：group_ranges 里的有限值个数
covered = int(np.isfinite(self.group_ranges).sum()) if self.use_inf \
    else int((self.group_ranges < self.rmax).sum())
```

同时删掉了 `group_covered` / `group_covered_cnt` 两个状态（不再需要单独的
方位覆盖位图，`group_ranges` 本身就是真值来源）。

---

## 五、阈值 0.70 是怎么定的

修复计数逻辑后，先沿用原阈值 0.95，结果 `/scan` 恒为 **2.00 Hz**、覆盖率稳在
78.2% —— 每次都等满 0.5s 组龄兜底。原因是**场景带内覆盖的天花板约 78%**，
阈值 0.95 永远凑不满。

用真实数据离线模拟不同阈值（720 bins，组龄兜底 0.5s，2058Hz 全密度）：

| 阈值 | 频率 | 覆盖率 | 相邻重叠 | 凑帧耗时 p50 |
|---|---|---|---|---|
| 0.65 | 13.6 Hz | 66% | 78% | 84 ms |
| **0.70** | **10.3 Hz** | **70%** | **86%** | **99 ms** |
| 0.75 | 5.9 Hz | 75% | 94% | 179 ms |

（360 bins 的对照：0.70 → 18.7Hz / 71% / 85%，若将来想再提速可考虑。）

选 0.70：**重叠率 86% 是消除闪烁的关键**，同时保住 10Hz 频率。
0.75 虽然重叠更高，但频率掉到 5.9Hz，对 AMCL 和避障反应不利。

### 累积窗口与覆盖率的权衡（同一次实测）

| 累积时长 | 覆盖率 | 相邻重叠 |
|---|---|---|
| 40 ms | 44% | 31% |
| 80 ms | 66% | 77% |
| 120 ms | 73% | 89% |
| 200 ms | 76% | 95% |
| 500 ms | 78% | 98% |

这张表说明两件事：
1. 覆盖率天花板确实在 78% 附近（再久也不会更高）
2. **40ms 的累积正是修复前 31% 重叠的来源**，与 bug 分析对得上

---

## 六、修复后的实测

### 有线 domain 0（coverage 模式）

```
条数=209 (10.4 Hz)
每条覆盖: p50=70% min=70%
相邻两条重叠: p50=86% min=83%
空白 scan: 0
```

`/scan` 频率 10.3~10.8Hz，TF 链 `map -> odom -> base_link_level` 正常，
AMCL 设初始位姿后正常发布 `map->odom`，日志无 `Transform data too old`。

### WiFi domain 1（sliding 模式）

```
条数=93 (4.7 Hz)      频率稳定 5.0Hz
覆盖 p50=58%
相邻重叠 p50=87%
空白=0
```

前 6 条是窗口预热（覆盖 25%~45%），之后稳定在 55%~65%。
覆盖率比有线低是数据量决定的（带内约 1000 点/秒），但**重叠率 87% 已经不闪**。

### 如果还想更稳

- 有线：`aggregate_min_coverage` 提到 **0.75**，重叠升到 ~94%，频率降到 ~5.9Hz。
  **不建议超过 0.75**，会撞上 78% 的覆盖天花板。
- 中继：`aggregate_window_sec` 从 0.6 提到 0.9，重叠升到 ~84%（实测模拟），
  代价是移动时拖影更明显。

> ⚠️ 这两个参数**运行时 `ros2 param set` 不生效**：`min_cover` / `agg_window`
> 只在 `__init__` 读一次。要改必须改 launch 再重启。

---

## 七、为什么调 RViz 的 Decay Time 不解决问题

RViz LaserScan 的 `Decay Time` 会让旧点保留一会儿，视觉上确实"不那么闪"。
实测（修复前，`/scan` 13Hz）：

| Decay Time | 画面上稳定显示 |
|---|---|
| 0.5 s | 56% 方位 |
| 1.0 s | 69% 方位 |
| 2.0 s | 74% 方位 |

但这只是把稀疏的点叠起来，**代价是看到的是几百毫秒前的历史点**，移动时会拖影，
而且下游（AMCL、costmap）拿到的还是那条残缺的 scan。
治标不治本，只适合临时看一眼。

---

## 八、双链路：自动切换的凑帧策略

| 链路 | domain | `/livox/lidar` | 点数/秒 | 凑帧模式 | 实测结果 |
|---|---|---|---|---|---|
| 有线直连 `enp130s0` | 0 | 2058 Hz | 158000 | `coverage` 0.70 / 0.5s | 10.3Hz / 70% / 重叠 86% |
| WiFi 中继 | 1 | ~190 Hz | ~7000 | `sliding` 0.6s / 5Hz | 5.0Hz / 58% / 重叠 87% |

`localization.launch.py` 读 `ROS_DOMAIN_ID` 自动选，启动日志会打印选了哪套：

```
[localization.launch] ROS_DOMAIN_ID=1 -> livox_scan
  {'aggregate_mode': 'sliding', 'aggregate_window_sec': 0.6, 'sliding_publish_hz': 5.0}
```

`setup.sh` 会自动判断网卡：网线 UP → 域 0，否则 WiFi → 域 1。

### 为什么中继链路必须换策略

中继链路的总点率 7000 点/秒，但**带内（z∈[-0.15,0.35]）只有约 1000 点/秒**。
720 bins 里要 70% 有回波，理论上需要 0.5s 以上——coverage 模式于是每帧都等满
超时兜底，退化成 2Hz 且重叠只有 44%（仍然闪）。

试过的调参（都救不回来）：

| 配置 | 频率 | 覆盖 | 重叠 |
|---|---|---|---|
| coverage th=0.70 / win 0.5 | 1.9 Hz | 40% | 44% |
| coverage th=0.45 / win 0.2 | 4.7 Hz | 21% | 16% |
| coverage bins=180 th=0.70 / win 0.4 | 2.4 Hz | 55% | 65% |
| **sliding T=0.6s / 5Hz** | **4.7 Hz** | **44%** | **73%** |
| sliding T=0.9s / 5Hz | 4.7 Hz | 52% | 84% |

滑动窗口的本质区别：不是"攒够了才发"，而是"每条都带最近 T 秒的全部数据"。
相邻两条天然共享窗口内绝大部分，重叠率不依赖数据密度。

**注意两套栈不能混**：

**注意两套栈不能混**：
- domain 0 上跑栈时，笔记本直连狗固件，`/api/sport/request` 直接生效
- domain 1 上跑栈时，命令经中继注入 domain 0

同一时刻只跑一套。切换网络后要**重启整个 launch**（domain 是进程启动时读的环境变量）。

中继本身的参数**不要动**（`RATE_SKIP` / `DOWNSAMPLE` 保持 2/2）：
Python 中继扛不住原始消息率，改全量反而更慢（见 3.3）。

---

## 九、踩坑记录

### 9.1 重复进程导致"点云时有时无"

排查中发现同时跑着**两套** `livox_scan` / `odom_tf_bridge` / `amcl` / `map_server` /
`static_transform_publisher`（两个不同的 PID 段）。两套栈互相抢 TF 和话题，
症状会非常迷惑。

原因是 `pkill -f "ros2 launch ..."` 杀不干净，子进程变孤儿继续跑。
正确清理方式：

```bash
pkill -f "ros2 launch nav"
pkill -f "nav/lib/nav/livox_scan|nav/lib/nav/odom_tf_bridge|nav/lib/nav/cmd_vel_sport_bridge"
pkill -f "nav2_controller|nav2_planner|nav2_bt_navigator|nav2_behaviors|nav2_amcl|nav2_map_server|nav2_lifecycle_manager"
pkill -f "rviz2 -d .*install/nav"
pkill -f "static_transform_publisher"
```

或者干脆 Ctrl-C 让 launch 自己优雅关闭，别用 kill。

### 9.2 domain 0 上没有 TF

以为"只要把点云源切到 domain 0 就行"，结果影子节点一直没输出——因为
`livox_static_tf` 和 `odom_tf_bridge` 都在 domain 1，domain 0 上根本没有
`livox_frame -> base_link_level` 的变换。**整套栈必须一起切。**

### 9.3 `map` 帧不存在是正常的

`ros2 run tf2_ros tf2_echo map base_link_level` 报 `Invalid frame ID "map"`，
不是配置错误：AMCL 没收到初始位姿就不发布 `map->odom`。
先发一个初始位姿再验：

```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

### 9.4 狗端中继的 systemd 管理

`go2_wifi_relay.service` 是 `Restart=always` + `KillMode=control-group`，
`sudo` 需要密码时**杀主进程即可触发自动重启**（不要直接 kill 各个 shard，
它们由主进程的 `start_shards.sh` 统一拉起）：

```bash
ssh unitree@192.168.123.18 'kill $(pgrep -f start_shards.sh)'
```

改动前务必备份，并确认回滚：

```bash
cd /home/unitree/go2_relay && cp relay.py relay.py.bak.$(date +%s)
```

---

## 十、改动清单

```
cyclonedds_ws/src/nav/nav/livox_scan.py
  - 覆盖率改为数 group_ranges 的有限值（= 真正输出的 bin）
  - 删除 group_covered / group_covered_cnt（重复计数 bug 的来源）
  - 新增 aggregate_mode（coverage / sliding 双模式）
  - 新增 sliding_publish_hz（滑动模式固定发布频率）
  - 抽出 _publish() / _sliding() 两个方法

cyclonedds_ws/src/nav/launch/localization.launch.py
  - 按 ROS_DOMAIN_ID 自动选凑帧参数，启动日志打印所选配置

docs/scan-flicker-diagnosis.md（本文）
docs/scan-rate-spike-diagnosis.md
  - 新增第七节：闪烁的两个判据、修复前后对比、有线 vs 中继密度差

cyclonedds_ws/src/nav/README.md
  - §4.1 改写凑帧说明与两条调参红线
  - 新增 §4.1.1 点云密度链路对比
```

编译（install 是拷贝式，必须重编）：

```bash
cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select nav
```

**不要加 `--symlink-install`**（会破坏本工作区的 Python 安装，见
`LOCAL_COSTMAP_2026-11-18.md` 7.1）。

### 验收

```bash
source ~/unitree_ros2/setup.sh          # 确认 domain 与网卡
ros2 launch nav localization.launch.py  # 日志第一行会打印所选模式
ros2 topic hz /scan                     # 有线 ~10Hz / 中继 ~5Hz
# 用第二节的探针脚本确认：相邻重叠 >= 80%，空白 scan = 0
```

切换网络（插拔网线）后**必须重启 launch**：domain 是进程启动时读的环境变量，
运行时改不了。重启后日志会显示新的模式。

---

## 十一、下次遇到同类问题

按这个顺序查，别跳步：

1. **量两个指标**：`/scan` 频率 + 相邻重叠率（第二节脚本）
2. **量输入密度**：`ros2 topic hz /livox/lidar` 和每片点数
   - 每片只有几十点、频率一两百 Hz → 走的是 WiFi 中继，`sliding` 模式
   - 每片八九十点、2000Hz 左右 → 驱动原始数据，`coverage` 模式
3. **确认凑帧计数口径**：数的必须是"高度带内、量程内、会真正输出"的 bin，
   且必须**先置位后计数**
4. **调阈值前先量天花板**：如果覆盖率永远停在某个值（比如 40% / 78%）不动，
   说明阈值设高了，每帧都在等超时兜底 —— 要么降阈值，要么换 `sliding` 模式
5. **低频链路别硬凑**：带内点数不足以在可接受延迟内凑满时，滑动窗口的
   重叠率天然更高，比死磕阈值有效
6. **别动中继的 `RATE_SKIP`/`DOWNSAMPLE`**，也别用 `/utlidar/cloud_base` 替代
   （这两个都试过，更差，见 3.2 / 3.3）
