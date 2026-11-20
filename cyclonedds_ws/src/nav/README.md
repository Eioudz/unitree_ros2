# nav 包总览（改代码前先读这里）

Go2 四足真机的 **定位 + Nav2 导航栈 + 运动指令桥接 + 语义单点导航**。
本文件是包的"地图"：数据流、每个文件负责什么、参数在哪、出问题先看哪。

> 历史排查细节见包内 `NAV_FIX_2026-11-18.md`、`LOCAL_COSTMAP_2026-11-18.md`
> 和仓库 `docs/scan-rate-spike-diagnosis.md`、`docs/rviz-freeze-diagnosis.md`。
> 那几份是"某天发生了什么"，本文件是"现在是什么样"。

---

## 1. 快速启动

```bash
# 终端 A：机器人模型 + 关节状态 + 厂商坐标系
source ~/unitree_ros2/setup.sh
ros2 launch go2_description go2_real.launch.py use_rviz:=false

# 终端 B：定位 + costmap + 规划/控制 + 运动桥 + RViz
source ~/unitree_ros2/setup.sh
ros2 launch nav navigation.launch.py                    # 完整导航
ros2 launch nav navigation.launch.py use_rviz:=false    # 无 RViz
ros2 launch nav navigation.launch.py map:=/home/soyo/maps/xxx.yaml

# 终端 C（可选）：语义指令
ros2 run nav goto_node        # 终端直跑时可直接打字：去前门/去后门/去电视机前/停止
ros2 topic pub --once /goto_cmd std_msgs/msg/String "{data: 去前门}"
```

启动后 **必须** 在 RViz 点 `2D Pose Estimate` 给初始位姿（AMCL `set_initial_pose: false`），
粒子云 `/particlecloud` 收敛后再点 `2D Goal Pose` 下发目标。

编译（**不要加 `--symlink-install`**，会破坏本工作区的 Python 安装）：

```bash
cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select nav
```

---

## 2. 目录结构

```
nav/
├── nav/                          # 4 个 Python 节点
│   ├── livox_scan.py             # Mid360 点云 -> /scan（时间戳重打 + 水平切片 + 可选聚合）
│   ├── odom_tf_bridge.py         # /utlidar/robot_odom -> TF + /odom_local（本机时钟重打）
│   ├── cmd_vel_sport_bridge.py   # /cmd_vel -> 1008 MOVE（限幅 + 看门狗）
│   └── goto_node.py              # 中文指令 -> NavigateToPose action
├── launch/
│   ├── localization.launch.py    # 纯定位：livox_scan + 静态TF + odom桥 + map_server + AMCL
│   └── navigation.launch.py      # 完整栈：include 定位 + controller/planner/BT/behavior + 运动桥
├── config/
│   ├── localization.yaml         # map_server / amcl / lifecycle_manager_localization
│   └── costmap.yaml              # controller_server(DWB) / planner / 全局+局部 costmap / BT / behavior / lifecycle
├── behavior_trees/
│   ├── navigate_to_pose_no_backup.xml        # 官方树删掉 BackUp
│   └── navigate_through_poses_no_backup.xml  # 同上
├── rviz/
│   ├── localization.rviz         # 定位用（Map/scan/粒子云/点云）
│   └── nav.rviz                  # 导航用（+Global/LocalCostmap、/plan、SetGoal）
├── resource/nav, setup.py, package.xml
└── test/                         # ament 默认的 flake8/pep257/copyright 模板
```

`setup.py` 的 entry_points：

| 可执行名 | 模块 |
|---|---|
| `livox_scan.py` | `nav.livox_scan:main` |
| `odom_tf_bridge.py` | `nav.odom_tf_bridge:main` |
| `cmd_vel_sport_bridge` | `nav.cmd_vel_sport_bridge:main` |
| `goto_node` | `nav.goto_node:main` |

---

## 3. TF 树与话题图

### TF

```
map ──(AMCL, 定位修正)──> odom ──(odom_tf_bridge, 150Hz)──> base_link
                                                          └> base_link_level   (roll/pitch 置零，仅留 yaw)

base_link ──(livox_static_tf, 静态)──> livox_frame   (z=0.10m, 前倾 13°)
go2_description 侧另有：base_link -> base -> radar -> utlidar_lidar（厂商坐标系，恒等/URDF 外参）
```

- `base_link_level` 是本包自造的"水平化"坐标系：`livox_scan` 切片的 target_frame、
  costmap 的 `robot_base_frame`、AMCL 的 `base_frame_id` 全都用它，保证 2D 切片平面
  永远平行地面，狗身俯仰不会把地面点切进 `/scan`。
- **不要同时跑 `slam.launch.py` 和 `localization.launch.py`**：两者都发 `map->odom`。
- `odom->base_link` 只应有 `odom_tf_bridge` 一个发布者（go2_description 只发静态段）。

### 话题

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/livox/lidar` | PointCloud2 (livox_frame) | 狗端 Mid360 驱动（经中继） | livox_scan |
| `/scan` | LaserScan (base_link_level) | livox_scan（BEST_EFFORT d5） | AMCL、local_costmap.obstacle_layer、RViz |
| `/utlidar/robot_odom` | Odometry（**狗时钟域**） | 狗端 LIO | odom_tf_bridge |
| `/odom_local` | Odometry（**本机时钟**） | odom_tf_bridge | controller_server、bt_navigator |
| `/map` | OccupancyGrid | map_server | AMCL、global/local static_layer、RViz |
| `/particlecloud` | PoseArray | AMCL | RViz |
| `/plan` | Path | planner_server | RViz |
| `/global_costmap/costmap` | OccupancyGrid | planner_server（0.5Hz） | RViz |
| `/local_costmap/costmap` | OccupancyGrid | controller_server（1Hz） | RViz |
| `/goal_pose` | PoseStamped | RViz `2D Goal Pose` | bt_navigator |
| `/initialpose` | PoseWithCovarianceStamped | RViz `2D Pose Estimate` | AMCL |
| `/cmd_vel` | Twist | **DWB 直接输出** | cmd_vel_sport_bridge |
| `/api/sport/request` | unitree_api/Request (api_id=1008) | cmd_vel_sport_bridge | 狗端 sport 服务 |
| `/goto_cmd` | String | 用户 | goto_node |

### 时钟域（最容易踩的坑）

狗主控 RTC 曾漂到 1970 年（现已用 `sync_dog_clock.py` 对齐），但**全链路仍统一到本机时钟**，防止狗重启后再次漂移：

- `odom_tf_bridge`：TF stamp 和 `/odom_local` 都用本机 `now()`
- `livox_scan`：`/scan` stamp = 本机 `now() - 40ms`
- 因此 AMCL/Nav2/RViz 查 TF 自洽；`/utlidar/robot_odom` 原始话题**不要**直接喂 Nav2
  （`costmap.yaml` 里 `odom_topic: "/odom_local"` 就是这个原因）。

---

## 4. 各节点说明

### 4.1 `livox_scan.py` —— 点云切片

```
/livox/lidar -> stamp=本机now-40ms -> TF livox_frame->base_link_level
             -> [可选: 聚合窗口] -> 高度带 z∈[-0.15,0.35] -> 极坐标
             -> 720 bins (0.5°/bin) 同 bin 取最近 -> /scan
```

参数（launch 可覆盖）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `lidar_topic` | `/livox/lidar` | 输入点云 |
| `scan_topic` | `/scan` | 输出扫描 |
| `target_frame` | `base_link_level` | 切片坐标系，必须是水平化帧 |
| `min_height` / `max_height` | -0.15 / 0.35 | 高度带：低于 -0.15 是地面噪声，高于 0.35 是桌面以上（狗能钻过） |
| `range_min` / `range_max` | 0.55 / 12.0 | 0.55 屏蔽前腿自点；12 实际没起作用（场景最远 7m） |
| `scan_bins` | 720 | 角分辨率 |
| `publish_inf` | true | 空 bin 发 inf（costmap 靠它做射线清除，**别关**） |
| `aggregate_min_coverage` | 0.70 | **launch 固化为 0.70**，见下 |
| `aggregate_window_sec` | 0.5 | 组龄兜底，超时未凑满也发布 |

实现要点（改之前务必知道）：

- 多线程执行器 + 点云独立 callback group：否则 2 万点 numpy 运算会把 `/tf` 回调堵住，
  TF buffer 滞后 -> 按 `now-40ms` 查 TF 报 extrapolation -> 整帧丢弃。
- 精确 stamp 查不到 TF 时退回"最新可用变换"，不丢帧。
- 发布 QoS 是 **BEST_EFFORT/depth=5**：RELIABLE 会被慢订阅者反压，消息排队后
  stamp 变旧，AMCL 直接丢（症状是 `/scan` 掉到 0.45Hz）。加 RELIABLE 订阅端要慎重。

**凑帧逻辑是本包最容易复发的坑。** 节点把多片点云累积成一个"组"，组内某 bin
有回波就记下，**带内回波 bin 比例达 `aggregate_min_coverage` 才发布**，
超时 `aggregate_window_sec` 兜底发布。狗端驱动会在"整帧直发（~5Hz）"和
"~2058Hz 小切片突发（每片 85 点）"之间来回回退，这套凑帧两种模式都能出
覆盖完整的 scan。

调参时必须守住两条（都踩过）：

1. **计数口径 = 输出内容**。只能数高度带内、量程内的回波 bin。若数"所有点的
   方位"（地面回波 + 远点几乎铺满 360°），计数几十毫秒就满，而输出只有小半圈
   -> 相邻两条 scan 重叠仅 31% -> RViz 一闪一闪。
2. **先置位再数**。`cnt += (~covered[idx]).sum()` 这种"先数后置位"的写法，
   同一片里多个点落在同一 bin 会重复计数，全密度下瞬间冲过阈值（实测 197Hz）。

判据只有一个：**相邻两条 `/scan` 的重叠率**，低于 ~70% 就会闪。
0.70/0.5s 在 2058Hz 下给出 10.3Hz、覆盖 70%、重叠 86%。
完整诊断流程见 `docs/scan-rate-spike-diagnosis.md`。

### 4.1.1 点云密度：走有线，别走中继

| 链路 | `/livox/lidar` | 点数/秒 |
|---|---|---|
| 有线直连（domain 0，`enp130s0`） | 2058 Hz | 158000 |
| WiFi 中继（domain 1） | 143~219 Hz | ~6500 |

中继的 `RATE_SKIP=2` + `DOWNSAMPLE=2` 砍掉 3/4，而且 Python 中继扛不住
2058Hz 的消息率（实测关掉抽样后狗 CPU 冲到 load 21，中继自身反而更慢）。
**要完整点云就走有线 domain 0，不要试图改中继。**

### 4.2 `odom_tf_bridge.py` —— 里程计桥

| 参数 | 默认 | 说明 |
|---|---|---|
| `odom_topic` | `/utlidar/robot_odom` | 输入 |
| `level_frame` | `base_link_level` | 水平化虚拟帧名（要与 `livox_scan.target_frame`、costmap `robot_base_frame`、AMCL `base_frame_id` 一致） |
| `publish_level_frame` | true | 是否发 `odom->base_link_level` |
| `publish_local_odom` | true | 是否发 `/odom_local` |

每帧发两个 TF（`odom->base_link` 原样、`odom->base_link_level` 只留 yaw），
stamp 全部重打为本机时钟；再发一条 `/odom_local` 给 Nav2。

### 4.3 `cmd_vel_sport_bridge.py` —— 运动指令桥

| 参数 | 默认 | 说明 |
|---|---|---|
| `min_vx` | 0.0 | **禁倒车硬约束**，负值会被夹到 0 |
| `max_vx` | 0.5 | 必须 >= DWB `max_vel_x`，否则提速无效 |
| `max_wz` | 1.2 | 必须 >= DWB `max_vel_theta` |
| `watchdog` | 0.5s | 超过这么久没收到 `/cmd_vel` 就发 0 速停车 |

输出 `unitree_api/Request`：`api_id=1008`，`parameter={"x":vx,"y":0,"z":wz}`，
`id` 必须自增（固定 0 会被 sport 服务丢弃）。退出时补发一次 0 速。

**禁倒车是四处联动**，只改一处会被绕过：
DWB `min_vel_x: 0.0` + 本节点 `min_vx: 0.0` + `behavior_plugins` 无 `backup`
+ 行为树无 `<BackUp>`。

### 4.4 `goto_node.py` —— 语义单点导航

| 参数 | 默认 |
|---|---|
| `front_door` | `[-3.61, -2.52]` |
| `back_door` | `[-6.81, 3.69]` |
| `tv` | `[-4.88, 4.25]` |

- 坐标是 RViz Publish Point 实测标定，换场地要改这三个参数（launch 里没传，改默认值或自己 `--ros-args -p`）。
- 目标朝向取"机器人->目标"方位角，已在 0.2m 内则保持当前朝向。
- 用 `ActionClient` 而不是 `BasicNavigator`（后者会和本节点主 spin 并发，报
  "generator already executing"）。
- 每次下发 `goal_seq += 1`，迟到回调凭代号丢弃，避免旧目标清掉新目标状态。
- stdin 是 TTY 时开打字线程，launch 下自动退化为 `/goto_cmd` 话题。
- "停止"只是 `cancel_goal`，真正停车靠运动桥看门狗。

---

## 5. 两个 launch 文件

### `localization.launch.py`

| 节点 | 说明 |
|---|---|
| `livox_scan` | `aggregate_min_coverage: 0.70` + `aggregate_window_sec: 0.5`（硬编码在 launch 里） |
| `livox_static_tf` | `base_link -> livox_frame`，`z=0.10`，`RPY(0, 13°, 0)`。**13° 是雷达安装俯仰**，去掉它会让切片平面斜 13°，把地面误投成立面鬼影 |
| `odom_tf_bridge` | 全默认参数 |
| `map_server` | `yaml_filename` 由 `map:=` 覆盖 |
| `amcl` | `config/localization.yaml` |
| `lifecycle_manager_localization` | autostart，激活 map_server + amcl |
| `rviz2` | `IfCondition(use_rviz)` |

参数：`use_rviz`(true)、`map`(/home/soyo/maps/lab_map.yaml)。

### `navigation.launch.py`

| 节点 | 说明 |
|---|---|
| `localization.launch.py` | include，强制 `use_rviz:=false` |
| `controller_server` | 局部 costmap + DWB |
| `planner_server` | 全局 costmap + NavFn |
| `lifecycle_manager_navigation` | autostart：controller / planner / behavior / bt_navigator |
| `behavior_server` | 只有 spin + wait |
| `bt_navigator` | 入口，接 `/goal_pose` |
| `cmd_vel_sport_bridge` | DWB `/cmd_vel` 直连（**没有 collision_monitor**） |
| `rviz2` | `nav.rviz`，用 `OpaqueFunction` 判断 `use_rviz`（本机 `IfCondition` 会静默跳过） |

速度链路是 `DWB -> /cmd_vel -> cmd_vel_sport_bridge`，中间没有
`collision_monitor`（2026-11-18 移除，`/cmd_vel_raw` 话题已不存在）。

---

## 6. 参数速查：想改什么改哪里

| 目标 | 位置 | 参数 |
|---|---|---|
| 走更快 / 更慢 | `config/costmap.yaml` DWB + `cmd_vel_sport_bridge` 默认值 | `max_vel_x`、`max_speed_xy`、`max_vel_theta`、`max_vx`、`max_wz`（**两处必须同步**） |
| 到点更准 | `costmap.yaml` | `goal_checker.xy_goal_tolerance`、`yaw_goal_tolerance`、`FollowPath.xy_goal_tolerance`（三处同步） |
| 离墙更远 | `costmap.yaml` | `inflation_radius`(0.55)、`cost_scaling_factor`(3.0) |
| 允许过更窄的缝 | `costmap.yaml` local `footprint` | 当前 `[[0.35,0.16],...]`（半长 0.35 / 半宽 0.16） |
| 局部障碍看更远 | `costmap.yaml` local | `width`/`height`(4)、`obstacle_layer.*_max_range`(4.0)。注意 CPU 按面积涨 |
| 切片高度带 | `livox_scan` 参数 / launch | `min_height`、`max_height` |
| 屏蔽自点范围 | `livox_scan.range_min` + `local costmap.obstacle_min_range` | 两处要一致 |
| `/scan` 频率异常 | `launch/localization.launch.py` | `aggregate_window_sec` |
| 雷达安装角 | `launch/localization.launch.py` | `livox_static_tf` 的 `0 0.1132032 0 0.9935719`（绕 y 13°） |
| 定位收敛快慢 | `config/localization.yaml` | `alpha1..5`、`min/max_particles`、`update_min_d/a` |
| TF 容忍度（丢帧/TF too old） | 两处 | `amcl.transform_tolerance`(0.5)、costmap `transform_tolerance`(local 1.0 / global 2.0) |
| 摇摆 | `costmap.yaml` critics | `Oscillation.scale` |
| 避障力度 | `costmap.yaml` critics | `ObstacleFootprint.scale`(0.5)、`PathAlign/GoalAlign/PathDist/GoalDist.scale` |
| 恢复行为转速 | `costmap.yaml` behavior_server + BT xml | `max/min_rotational_vel`、`rotational_acc_lim`、`Spin spin_dist` |
| 语义目标点 | `goto_node.py` 参数默认值 | `front_door`、`back_door`、`tv` |
| 地图 | `map:=` 或 `localization.yaml` | `yaml_filename` |

### 当前 costmap 设计（一句话）

- **全局**：`static_layer + inflation_layer`，外接圆 `robot_radius 0.39`，只用预建地图 —— 规划走"提前绕开墙"的宽路径。
- **局部**：`static_layer + obstacle_layer(/scan) + inflation_layer`，矩形 footprint —— 雷达实时障碍进膨胀，DWB 按真实轮廓精执行。
- **动态障碍只进局部**，全局不实时避障。这是 11-18 的设计选择（加全局实时障碍会引入 DWB 梯度/抢控权那套老问题）。

---

## 7. 行为树

`behavior_trees/*.xml` 是 humble 官方树的副本，**删掉了 `<BackUp>`**，
recovery 轮换改为：清局部/全局 costmap -> `Spin(1.0rad)` -> `Wait(5s)`。

`costmap.yaml` 里 `bt_navigator.default_nav_*_bt_xml` 写的是 **install 绝对路径**：

```yaml
/home/soyo/unitree_ros2/cyclonedds_ws/install/nav/share/nav/behavior_trees/navigate_to_pose_no_backup.xml
```

所以改完 xml **必须 `colcon build`** 才生效，且换机器/换工作区要改这两行。

---

## 8. 故障速查表

| 症状 | 先查 | 修法 |
|---|---|---|
| 狗完全不动，`controller_server` 刷 `Transform data too old when converting from map to odom` | `ros2 topic hz /scan` | 数百 Hz -> 看 §4.1 凑帧逻辑；正常 -> 看 `amcl.transform_tolerance`、狗时钟 |
| `/scan` 频率异常 / RViz 闪烁 | `ros2 topic hz /scan`、相邻两条重叠率 | 见 §4.1：计数口径 + 先置位后数，阈值 0.70 |
| `/scan` 掉到 1Hz 以下 / RViz 冻结 | QoS、`ros2 topic info /scan -v` | 发布端应 BEST_EFFORT；有 RELIABLE 慢订阅者会反压 |
| 局部 costmap 全 0 | `ros2 topic hz /local_costmap/costmap`、`ros2 topic echo /scan --once` | `obstacle_layer` 是否在 plugins、`inf_is_valid: true`、scan stamp 是否太旧 |
| 局部障碍时有时无 | 每条 scan 是否 360° 覆盖 | 聚合窗口 < 簇周期，改 0.2 |
| 导航左右摇摆 | — | `Oscillation` critic 是否在列表、`scale` |
| 贴障碍顶停不绕 | — | `inflation_radius`、`ObstacleFootprint.scale`、是否被禁倒车策略限制（贴脸只能 Spin） |
| `bt_navigator` activate 失败 | 日志找 `backup` / BT xml 路径 | `behavior_plugins` 不含 backup，xml 路径存在 |
| RViz 一点就卡死 | `nvidia-smi`、`mokutil --sb-state` | Secure Boot 挡住 NVIDIA 驱动，llvmpipe 软渲染（见 `docs/rviz-freeze-diagnosis.md`） |
| `PackageNotFoundError: nav` | `install/nav/lib/python3.10/site-packages` | 误用了 `--symlink-install`，删 install/build 后普通重编 |
| AMCL `initialpose message is malformed` | 手写四元数模长 | 归一化后再发 |
| 点云"时有时无"、TF 抢 | `ros2 node list` 是否有重复节点 | `ros2 launch` 被 kill 后子进程变孤儿，按进程名清理 |

清理残留进程：

```bash
pkill -f "ros2 launch nav"
pkill -f "nav2_controller|nav2_planner|nav2_bt_navigator|nav2_behaviors|nav2_amcl|nav2_map_server|nav2_lifecycle_manager"
pkill -f "nav/lib/nav/livox_scan|nav/lib/nav/odom_tf_bridge|nav/lib/nav/cmd_vel_sport_bridge"
pkill -f "rviz2 -d .*install/nav"
```

---

## 9. 验证清单

```bash
ros2 topic hz /scan                      # ~10Hz（有线 domain 0）/ ~5Hz（WiFi 中继）
ros2 topic hz /odom_local                # ~150Hz
ros2 topic hz /local_costmap/costmap     # ~1Hz
ros2 topic hz /global_costmap/costmap    # ~0.5Hz
ros2 run tf2_ros tf2_echo map base_link_level
ros2 topic echo /cmd_vel                 # 导航时 vx 应恒 >= 0
ros2 topic echo /api/sport/request --once
```

---

## 10. 已知限制 / 注意

1. 走 WiFi 中继时 `/scan` 密度上限由狗端 `RATE_SKIP`/`DOWNSAMPLE` 决定（砍到 ~6500 点/秒），不在本仓库；有线 domain 0 直连可拿到 15.8 万点/秒。
2. 全局规划只用静态地图，实时障碍不进全局。
3. 局部窗口 4×4m（±2m）是膨胀范围的实际上限，`range_max` 调到 12 没有意义。
4. `rviz/localization.rviz` 的点云层默认 `Enabled: false`，看原始点云要手动勾选（topic 是 `/utlidar/cloud_base`，不是 `/livox/lidar`）。
5. 本包所有节点都是 Python，改完要 `colcon build --packages-select nav`（install 是拷贝式）。
