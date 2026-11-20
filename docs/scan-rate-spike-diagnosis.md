# /scan 频率异常飙升（数百 Hz）排查手册

这个问题至少复发过 3 次（11-13 前、11-18、11-20），症状固定、根因固定，
按本文从定位到修复 5 分钟内可完成。**先说结论：狗端 Mid360 驱动的发送模式
回退了，不是 nav 栈的问题。**

## 一、症状

```bash
ros2 topic hz /scan
average rate: 459.220        # 正常应为 ~5Hz（整帧直发）或 ~10Hz（聚合后）
```

典型连带症状（同一根因的不同表现，按出现顺序）：

1. `ros2 topic hz /scan` 显示数百 Hz（实测 409~459Hz）
2. controller_server 每个控制周期（10Hz）刷错：
   `Transform data too old when converting from map to odom`
   （Transform time 落后 Data time 数秒）
3. DWB 拿不到位姿 → 一帧 /cmd_vel 都不发 → **狗不动**
4. AMCL / RViz 卡顿，RViz 里 LaserScan/点云显示冻结
5. CPU 占用异常高（AMCL 粒子滤波 + costmap 被每秒数百条 scan 淹没）

## 二、根因

`/scan` 全链路只有 livox_scan.py 一个发布者，其逻辑是
**收到一片点云就产出一个 LaserScan（聚合窗口为 0 时）**。
所以 /scan 频率 ≈ /livox/lidar 到达频率。

狗端 Mid360 驱动有两种发送模式：

| 模式 | /livox/lidar 频率 | 帧形态 | /scan 频率（窗口=0） |
|------|------------------|--------|---------------------|
| 整帧直发（11-18 改的，正常） | ~5.3Hz | 227KB/帧，帧间隔 57~366ms | ~5Hz ✔ |
| 小切片突发（旧模式，问题态） | ~600Hz | 60 片/帧，每片 ~6° 扇区，突发到达 | **~460Hz ✘** |

狗端驱动一重启/回退/刷机就会变回切片模式（改动似乎没有持久化，**复发次数
多就是这个原因**）。而 livox_scan 默认 `aggregate_window_sec: 0`（不聚合），
切片模式下每个 ~6° 的小扇区都被当成"整帧"直发 → /scan 数百 Hz。

### 为什么 460Hz 会把导航链路搞死

- AMCL 按条处理 scan 并发布 map->odom：输入 460Hz 远超其处理能力，
  消息排队/丢弃，**map->odom 长时间不更新**
- controller_server（10Hz）按最新位姿查 map->odom → TF 已落后数秒 →
  `Transform data too old` → DWB 无位姿、无 cmd_vel → 狗静止
- costmap obstacle_layer、RViz 显示同步被打满，全机 CPU 飙升进一步放大延迟

## 三、诊断（按顺序执行，3 条命令定位）

```bash
# 1. /scan 频率：数百 Hz 即确认本问题
ros2 topic hz /scan

# 2. /livox/lidar 频率：~600Hz = 狗端切片模式复发；~5Hz = 狗端正常（另查）
ros2 topic hz /livox/lidar

# 3. 当前聚合窗口参数：0 = 不聚合（切片模式下必炸）
ros2 param get /livox_scan aggregate_window_sec
```

判读：

- /scan 数百 Hz + /livox/lidar 数百 Hz + 窗口 0 → **本文问题，走第四节修复**
- /scan 数百 Hz 但 /livox/lidar ~5Hz → 不可能出现（ livox_scan 一进一出），
  检查是否有别的节点也在发 /scan：`ros2 topic info /scan -v`
- /scan 正常但仍有 TF 报错 → 不是本问题，查 /scan stamp 陈旧（RELIABLE
  反压问题，见 livox_scan.py 头注释 2026-11-19 段）或时钟漂移（sync_dog_clock.py）

## 四、修复

### 笔记本端兜底（已固化，2026-11-20）

[localization.launch.py](../cyclonedds_ws/src/nav/launch/localization.launch.py)
的 livox_scan 节点已带：

```python
parameters=[{'aggregate_min_coverage': 0.9,
             'aggregate_window_sec': 0.5}],
```

凑帧策略（2026-11-20 二次改版）：按**角度覆盖率**聚合——累计到 >=90% 的
bin 有真实回波才发布，丢片/空窗时自动跨帧凑满，每条 /scan 接近 360° 全覆盖，
频率随流况 ~3-8Hz 自适应。**此配置不要删**——它是专门为"狗端驱动随时可能
回退"设计的保险。

为什么弃用固定时窗（0.09/0.2 都试过）：切片流存在丢片和 >200ms 空窗（实测
/scan 间隔 0.2~0.47s），任何固定时窗都可能凑出"只盖部分扇区、其余 bins 全
inf"的半帧；半帧的无回波 bins 被 costmap clearing 当自由空间清理 -> 局部
障碍时有时无 + RViz 点云闪烁。覆盖率凑帧从机制上杜绝半帧出街。
window 0.5 仅作组龄兜底（防持续遮挡凑不满卡死）。

改完记得编译生效（install 是拷贝式，非 symlink）：

```bash
cd ~/unitree_ros2/cyclonedds_ws && colcon build --packages-select nav
```

### 狗端根治

把狗上的 Mid360 驱动改回**整帧直发**模式（2026-11-18 做过一次：
实测 227KB/帧、5.3Hz、帧间隔 57~366ms）。做完后建议：

1. `ros2 topic hz /livox/lidar` 确认 ~5Hz
2. 若要关闭聚合窗口，改回 `aggregate_window_sec: 0` 再编译重启
3. **检查狗端改动是否持久化**（重启狗后再看一眼频率），否则还会复发

## 五、修复后验收

```bash
ros2 topic hz /scan                     # ~5-10Hz，稳定
ros2 topic hz /local_costmap/costmap    # ~1Hz，costmap 活了
# controller_server 不再刷 Transform data too old
# 下发导航目标后 /cmd_vel_raw、/cmd_vel 有输出，狗开始走
```

## 六、历次复发记录

| 日期 | 现象 | 处置 |
|------|------|------|
| 11-13 前 | 切片模式：RViz 点云逐扇区闪烁、下游 60 倍负载 | 写 livox_scan.py，内置 90ms 聚合窗口 |
| 11-18 | 狗端改整帧直发后，90ms 窗口反把 <90ms 间隔的整帧并帧（5.3→2.2Hz） | 窗口改参数化，默认 0（整帧驱动） |
| 11-19 | /scan 掉频到 0.45Hz（RELIABLE 反压 stamp 冻结，与本文不同因） | 发布改 BEST_EFFORT/depth=5 |
| 11-20 | 狗端驱动回退切片模式，/scan 459Hz，TF 报错复发、狗不动 | launch 固化 aggregate_window_sec: 0.09 |
| 11-20 晚 | 90ms 窗口 < 簇周期（130ms），部分 scan 只盖小角扇区 -> costmap 盲区清障，局部障碍时有时无 | 窗口 0.09 -> 0.2，每条 scan 必含完整簇（360°），~5Hz |
| 11-20 深夜 | 切片流有丢片和 >200ms 空窗（实测 /scan 间隔 0.2~0.47s），固定时窗仍凑出半帧 -> 点云/代价地图闪烁 | livox_scan 改按角度覆盖率凑帧（>=90% bins 有回波才发，跨帧补丢片），window 仅作组龄兜底 |
| 11-20 午后 | **RViz 点云一闪一闪**。有线直连 domain 0 实测：/scan 197Hz、每条仅 9.8% 方位有回波、57% 的 scan 不足 10%、相邻两条重叠仅 31% | 两个 bug 叠加：①覆盖率"先数后置位"重复计数，全密度下一片 85 点挤同 bin 瞬间虚高；②计数口径数的是所有点方位（地面+远点铺满 360°），而输出只含带内回波。改为按 `group_ranges` 的有限值计数 + 先置位后数，阈值 0.95->0.70。结果 10.3Hz / 覆盖 70% / 相邻重叠 86% |

## 七、为什么"点云一闪一闪"（11-20 午后）

RViz 每收到一条 LaserScan 就擦掉上一条重画。**闪烁 = 相邻两条 scan 的重叠太低**，
不是显示设置问题。判定指标只有两个：

```bash
ros2 topic hz /scan                 # 过高（数百 Hz）或过低都异常
# 每条 scan 的有效回波 bin 数 / 720，相邻两条的交并比
```

修复前实测（有线 domain 0，驱动原始 2058Hz / 15.8 万点/秒）：

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| /scan 频率 | 197 Hz | 10.3 Hz |
| 每条有回波 bin | 中位 9.8%（57% 不足 10%） | 70%（最小 70%） |
| 相邻两条重叠 | 31% | **86%** |
| 完全空白 scan | 有 | 0 |

两个根因都在 `livox_scan.py` 的覆盖率凑帧逻辑：

1. **重复计数**：`group_covered_cnt += (~covered[idx]).sum()` 先数后置位，同一片
   多个点落在同一 bin 会被重复计入。全密度下每片 85 点挤在几个相邻 bin，
   计数瞬间冲过阈值 -> 每片都发 -> 197Hz。
2. **计数口径错**：数的是"所有点的方位"（含地面回波和 5m 外远点，几乎铺满
   360°），但 `/scan` 只输出高度带内的回波。于是计数 38ms 就满，实际输出
   只有 44% 的方位 -> 相邻重叠 31% -> 闪烁。

现逻辑：覆盖率 = `group_ranges` 里的有限值个数（即真正会输出的 bin），
阈值 0.70、组龄兜底 0.5s。0.70 是因为场景带内覆盖天花板约 78%，设 0.95
会永远凑不满、每帧都等满超时（实测 /scan 恒 2Hz）。

### 有线直连 vs 中继

有线接狗（`enp130s0` / domain 0）时笔记本直接看到驱动原始数据，密度差 24 倍：

| 链路 | /livox/lidar | 点数/秒 | 凑满 95% 覆盖 |
|------|--------------|---------|---------------|
| 中继（WiFi domain 1） | 143~219 Hz | ~6500 | 需数百 ms，且丢片 |
| 有线直连（domain 0） | 2058 Hz | 158000 | 38ms |

**中继的 `RATE_SKIP=2` + `DOWNSAMPLE=2` 把点云砍到 1/4 后仍只有 6500 点/秒**，
因为 Python 中继扛不住 2058Hz 的消息率（实测关掉抽样后狗 CPU 冲到 load 21，
中继自身反而更慢）。**结论：要完整点云就走有线 domain 0，不要试图改中继。**
