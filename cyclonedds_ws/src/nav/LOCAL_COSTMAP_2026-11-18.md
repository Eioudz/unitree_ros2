# 局部代价地图实时点云膨胀（2026-11-18 晚）

本文记录给 `nav` 包补上**局部代价地图实时障碍层**的过程，以及排查中发现的
`/scan` 掉频问题。承接同日早前的 `NAV_FIX_2026-11-18.md`（那份文档第九节
"local_costmap 只认静态图"的结论已由本文推翻）。

---

## 一、目标与结果

**目标**：在 RViz 里看到实时点云被膨胀成代价区域（红色梯度圈），随机器人移动刷新。

**结果**：已实现。局部代价地图现在有三层 —— 预建墙 + 雷达实时障碍 + 膨胀。

| 项目 | 状态 |
|------|------|
| `local_costmap` 插件链 | `static_layer` + `obstacle_layer` + `inflation_layer` |
| RViz 显示层 | 新增 `LocalCostmap` |
| `/scan` 频率 | 1.4 Hz → 5.0 Hz |
| 局部地图实测内容 | 3506 障碍格 + 1781 膨胀格 |

---

## 二、改动清单

```
cyclonedds_ws/src/nav/
├── config/costmap.yaml     # local_costmap 加回 obstacle_layer
├── rviz/nav.rviz           # 新增 LocalCostmap 显示层
└── nav/livox_scan.py       # 修 /scan 掉频（见第四节）
```

### 2.1 `config/costmap.yaml`

`local_costmap.plugins` 从 `["static_layer", "inflation_layer"]` 改为三层：

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan
  scan:
    topic: /scan
    sensor_frame: base_link_level      # /scan 输出在 base_link_level
    data_type: "LaserScan"
    min_obstacle_height: -0.15         # 低于地面是噪声
    max_obstacle_height: 0.35          # 桌面以上狗能钻，不算墙
    marking: true
    clearing: true
    raytrace_max_range: 4.0            # 见下方说明
    raytrace_min_range: 0.0
    obstacle_max_range: 4.0
    obstacle_min_range: 0.55           # 与 livox_scan range_min 一致
    inf_is_valid: true
```

**几个关键参数的取舍**：

- **`inf_is_valid: true`**：`livox_scan` 对空 bin 发 `inf`（`publish_inf=True`）。
  必须当作有效量程参与 raytrace，雷达扫过的空区域才会被清空。设 `false` 的话
  空 bin 被整条丢弃，障碍物走开后旧标记不消失，通道会被历史障碍堵死。

- **量程压到 4.0m 而非 12.0m**：局部滚动窗只有 4×4m，中心到对角 2.83m，超出
  窗外的标记和射线清除都会被裁掉，是纯浪费 CPU。按 12m 做 raytrace 是 4m 的
  3 倍开销，与 11-04 的降载结论冲突。

- **`sensor_frame: base_link_level`**：`/scan` 就输出在这个帧，与
  `robot_base_frame` 重合，传感器原点即机身中心。

`inflation_layer` 未改动，仍是 `inflation_radius: 0.55` / `cost_scaling_factor: 3.0`。

### 2.2 `rviz/nav.rviz`

原来只有 `GlobalCostmap` 显示层，新增 `LocalCostmap`（alpha 0.7，不遮住地图）：

```yaml
- Class: rviz_default_plugins/Map
  Name: LocalCostmap
  Alpha: 0.7
  Color Scheme: costmap
  Topic:
    Value: /local_costmap/costmap
  Update Topic:
    Value: /local_costmap/costmap_updates
```

---

## 三、验证方法

### 3.1 离线验证（不接狗）

用假传感器隔离验证过配置本身正确：假 TF 链 + 前方 1.5m 一堵墙的假 `/scan`
+ 全空地图，controller_server 起来后：

```
frame=odom size=80x80 res=0.05 origin=(-1.95,-1.95)
occupied cells=117  inflated cells=513
occupied world x[1.30,1.65] y[-0.45,0.40]
inflated world x[0.90,2.00] y[-0.85,0.80]
```

障碍落在 1.30~1.65m，膨胀扩散到 0.90~2.00m，符合 `inflation_radius: 0.55`。

### 3.2 实机验证

```bash
source ~/unitree_ros2/setup.sh
ros2 launch nav navigation.launch.py
```

RViz 左侧勾上 **LocalCostmap** 和 **LaserScan**，点 **2D Pose Estimate** 给初始位姿后，
应能看到扫描点周围由深到浅的膨胀梯度，狗移动时跟随滚动窗刷新。

命令行确认：

```bash
ros2 topic hz /local_costmap/costmap      # ~1Hz（publish_frequency）
ros2 topic echo /local_costmap/costmap --once | head    # 有非 0 值
```

---

## 四、排查中发现的问题：`/scan` 掉频到 1.4Hz

这是本次改动过程中撞到的独立 bug，已一并修掉。

### 4.1 现象

局部代价地图加上后，机器人执行导航目标时**原地不停打转**（`/cmd_vel` 恒定
`vx=0.4, wz=-0.4`）。日志里：

```
[controller_server] Received a goal, begin computing control effort.
[bt_navigator] Behavior Tree tick rate 100.00 was exceeded!
[planner_server] Planner loop missed its desired rate of 0.5000 Hz. Current loop rate is 0.0386 Hz
[tf_help] Transform data too old when converting from map to odom
          Data time: ...204  Transform time: ...186     ← 差 18 秒
```

### 4.2 根因链

四个环节串成一条：

1. **`livox_scan` 用单线程执行器** —— 点云回调（2 万点 numpy 运算）把 `/tf`
   回调堵在队列后面，TF buffer 相对墙钟滞后几百毫秒
2. 代码按 `now-40ms` 查 TF，滞后就报 `extrapolation-into-the-future`，
   **整帧点云丢弃** → `/scan` 从 5Hz 掉到 1.4Hz
3. **AMCL 收不到足够扫描** → 不更新 `map->odom` → 时间戳冻住
4. 代价地图基于冻结位姿 → DWB 每帧算出同一条轨迹 → 恒定 `vx=0.4, wz=-0.4`
   → **原地绕圈**

而且 `get_logger().warn(..., throttle_duration_sec=2.0)` 把 TF 失败限流成
每 2 秒一条，看起来像偶发，实际每帧都在失败。

### 4.3 修复（`nav/livox_scan.py`）

**1) 多线程执行器 + 独立 callback group**

点云回调和 `/tf` 回调不再互相排队：

```python
self.cloud_group = MutuallyExclusiveCallbackGroup()
self.create_subscription(PointCloud2, topic, self.cloud_cb,
                         qos_profile_sensor_data, callback_group=self.cloud_group)
...
executor = MultiThreadedExecutor(num_threads=3)
executor.add_node(node)
executor.spin()
```

**2) TF 查询兜底**

精确时间戳查不到时退到"最新可用变换"，而不是丢弃整帧：

```python
try:
    tf = self.tf_buf.lookup_transform(self.target, msg.header.frame_id, t)
except Exception as e:
    try:
        tf = self.tf_buf.lookup_transform(
            self.target, msg.header.frame_id, rclpy.time.Time())
        self.get_logger().warn('TF exact lookup failed, used latest instead', ...)
    except Exception as e2:
        return
```

0.5m/s 下车速下几帧的位姿偏差 < 数 cm，远小于 `range_min 0.55` 的自点屏蔽余量。

**3) 聚合窗口改为可配置、默认 0**

原来硬编码 90ms 聚合窗口，是为"狗端把整帧切成 ~600Hz 小片突发到达"的旧驱动
写的。现网 Mid360 驱动已改发整帧（实测 227KB/帧、5.3Hz、帧间隔 57~366ms），
90ms 窗口会把间隔 <90ms 的两帧并成一片，直接把频率砍半。

```python
self.declare_parameter('aggregate_window_sec', 0.0)   # 0 = 不聚合
```

### 4.4 效果

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| `/scan` | 1.4 Hz | 5.0 Hz |
| TF 查询失败 | 每帧 | 0 |
| `map->odom` | 冻结 | 持续更新 |

---

## 五、`/scan` 为什么到不了 10Hz

狗端雷达驱动确实是 `publish_freq:=10.0`，但笔记本只能收到 5Hz。**瓶颈在狗端
中继脚本** `/home/unitree/go2_relay/relay.py`：

```python
RATE_SKIP  = {..., '/livox/lidar': 2}   # 每 2 帧只转发 1 帧 → 10Hz 砍成 5Hz
DOWNSAMPLE = {..., '/livox/lidar': 2}   # 每 2 个点只留 1 个 → 单帧体积减半
```

实测对得上：狗端中继每 30 秒转发 150 帧 = **5Hz**，笔记本收到 4.55Hz
（差值是无线丢包）。

这两个参数是当初为在 WiFi 上稳定传点云特意加的 —— 每帧 228KB 拆成上百个
UDP 包，丢一个包整帧作废，必须压体积。

**决定：不改。** 5Hz 对 AMCL 和代价地图完全够用（AMCL 默认 `update_min_d: 0.1`
本来就不需要 10Hz 扫描）。要真 10Hz 需改狗端 `RATE_SKIP`，代价是 WiFi 带宽
翻倍、可能因丢包反而更不稳。

---

## 六、关于 `range_max` 的结论

有人可能想调大 `livox_scan` 的 `range_max`（默认 12.0）让点云看更远。
**实测证明没用**：

```
声明 range_max = 12.0
实际有限值: p50=1.7m  p90=3.2m  max=7.1m
≥10m 的点: 0 个
前方 ±20° 扇区: max=3.0m  median=1.3m
```

场景里最远的点只有 7.1m，`range_max=12.0` 一个点都没砍掉。改成 20 或 50，
RViz 不会有任何变化。

管线上的四道限制，`range_max` 是最松的那道：

| 环节 | 当前值 | 实际效果 |
|------|--------|----------|
| `livox_scan.range_max` | 12.0m | 没起作用（数据最长 7.1m） |
| `amcl.laser_max_range` | 12.0m | 定位用，同样没起作用 |
| `obstacle_layer.obstacle_max_range` | 4.0m | 次要限制 |
| **`local_costmap` 窗口 4×4m** | **±2.0m** | **← 真正的瓶颈** |

滚动窗只有 4×4m，代价地图最多只能显示狗周围 ±2m。前方 3m 的障碍物即使
雷达扫到了，也落在窗口外被裁掉，不会进膨胀。

**若要让膨胀延伸更远**，该改的是窗口尺寸：

```yaml
width: 6      # 4 → 6，前方覆盖到 ±3m
height: 6
```

代价是面积从 16m² 涨到 36m²（2.25 倍 CPU）。当前未改。

---

## 七、排查过程中的两个假警报

记录下来避免以后重复踩。

### 7.1 `colcon build --symlink-install` 会破坏本工作区的 Python 安装

本工作区原本是**不带** `--symlink-install` 构建的。中途用 `--symlink-install`
重编 nav 后，`install/nav/lib/python3.10/site-packages/` 里只剩一个
`nav.egg-link`，配套的 `easy-install.pth` 丢失，导致：

```
PackageNotFoundError: No package metadata was found for nav
[ERROR] [livox_scan.py-1]: process has died [exit code 1]
[ERROR] [odom_tf_bridge.py-3]: process has died [exit code 1]
```

**修复**：删掉 `install/nav/lib/python3.10/site-packages` 和
`build/nav/nav.egg-info`，用普通 `colcon build --packages-select nav` 重编。

**以后改这个工作区的 nav 包，不要加 `--symlink-install`。**

### 7.2 AMCL 报 `initialpose message is malformed` 未必是 AMCL 的问题

调试时用命令行手写四元数 `z=-0.966, w=0.257` 发测试位姿，AMCL 报：

```
[ERROR] [amcl]: Received initialpose message is malformed. Rejecting.
```

查下来是**测试数据不规范**：该四元数模长² = 0.999205，偏差 8e-4 超了
`tf2::isValid` 的容差。换成 RViz 实际发出的 `-0.966398/0.257052`
（模长²=1.0000008）就正常接受。

**结论**：RViz 拖出来的位姿不会触发这个，命令行手写测试数据时要归一化。

### 7.3 用 `pkill -f "ros2 launch ..."` 杀不干净

`ros2 launch` 被 kill 后，子进程会变孤儿继续跑，导致重启后出现**两套重复节点**
（`lifecycle_manager_navigation` ×2、`livox_static_tf` ×2），互相抢占 TF 和
话题，症状是"点云时有时无"。

**正确做法**：先 `pkill -f "ros2 launch nav"`，再按进程名清理：

```bash
pkill -f "nav2_controller|nav2_planner|nav2_bt_navigator|nav2_behaviors|nav2_amcl|nav2_map_server|nav2_lifecycle_manager"
pkill -f "nav/lib/nav/livox_scan|nav/lib/nav/odom_tf_bridge|nav/lib/nav/cmd_vel_sport_bridge"
pkill -f "rviz2 -d .*install/nav"
```

或者用 Ctrl-C 让 launch 自己优雅关闭。

---

## 八、遗留与注意

1. **全局代价地图仍只用静态图**：`global_costmap.plugins` 只有
   `static_layer` + `inflation_layer`，实时障碍只进局部。这是 11-18 早前的
   设计选择（当时实测"贴障碍不绕、只会顶停"，根因在 DWB 得分梯度和
   collision_monitor 抢控权，不在 costmap）。若要让导航真正绕开实时障碍，
   需要连带评估那套调参结论，不是一个 yaml 能包住的。

2. **局部窗口 4×4m 是膨胀范围的实际上限**（见第六节）。想扩大改
   `width`/`height`，注意 CPU。

3. **`/scan` 上限 5Hz 由狗端中继决定**（见第五节），不在本仓库范围内。

4. **本轮改动未提交**。涉及三个文件：`config/costmap.yaml`、
   `rviz/nav.rviz`、`nav/livox_scan.py`。

---

## 九、相关文档

- `NAV_FIX_2026-11-18.md` —— 同日早前一轮修复（点云显示慢、2D Goal Pose 不走、
  禁倒车、左右摇摆）。其中第九节"local_costmap 只认静态图"的结论已由本文
  第二节推翻，阅读时以本文为准。
