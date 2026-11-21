# RViz 卡死排查记录

日期：2026-11-19 ~ 2026-11-20
环境：ASUS ROG Strix G16 / Ubuntu 22.04 / ROS 2 Humble / RViz2

## 现象

`ros2 launch nav navigation.launch.py` 启动后：

- 一点 RViz 的 **2D Goal Pose**，RViz 界面就卡住无响应
- 同时狗不动（这是另一个问题，见文末）
- 进程没崩，但界面冻结

## 结论

**RViz 跑在 CPU 软件渲染上（llvmpipe），NVIDIA 显卡完全没参与。**

NVIDIA 驱动装了但内核模块没加载，被 **Secure Boot** 挡住。RViz 只能用 Intel 核显的软件光栅化器（llvmpipe）渲染，而导航栈每秒往 RViz 灌 150+ Hz 的 TF、30 Hz 的刷新，软件渲染扛不住就卡死。

这是独立于导航配置的问题，改 RViz 配置只能缓解，根治要处理 Secure Boot。

## 证据

### 1. NVIDIA 驱动未加载

```bash
$ nvidia-smi
NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver.

$ ls /proc/driver/nvidia
不存在（驱动未加载）

$ mokutil --sb-state
SecureBoot enabled

$ dkms status
nvidia/595.99.02, 6.8.0-138-generic, x86_64: installed
```

驱动包 `nvidia-driver-595-open 595.99.02` 已装、DKMS 模块已编译，但内核拒绝加载未签名的模块。

注意：`lsmod | grep nvidia` 里能看到 `nvidia_wmi_ec_backlight`，那是华硕主板的背光控制模块，**不是显卡驱动**，容易误判。

### 2. RViz 实际用的是软件渲染

```
/proc/<rviz_pid>/maps 中出现：
  /usr/lib/x86_64-linux-gnu/dri/swrast_dri.so
  /usr/lib/x86_64-linux-gnu/libGLX_mesa.so.0.0.0

没有 libGLX_nvidia，没有 /dev/dri 的 nvidia 节点
```

`/dev/dri` 下只有 Intel 核显：

```
card1
renderD128          ← by-path 显示属于 pci-0000:00:02.0（Intel）
```

### 3. CPU 占用

```
rviz2  77 ~ 138 %CPU      ← 软件渲染把多核跑满
其他节点（amcl/controller/planner）各 1~3 %
```

### 4. RViz 日志里的直接症状

```
[INFO] [rviz2]: Message Filter dropping message: frame 'odom'
       at time ... for reason 'discarding message because the queue is full'
```

单次运行中出现 **198 条**。渲染线程跟不上，消息队列堆满后整批丢弃，界面就表现为"卡死"。

另有一条无害但显眼的 GLSL 报错（llvmpipe 的已知限制，不影响功能）：

```
[ERROR] [rviz2]: ... GLSL link result :
active samplers with a different type refer to the same texture image unit
```

## 已做的缓解（已改，未提交）

`cyclonedds_ws/src/nav/rviz/nav.rviz`：

| 项 | 原值 | 现值 | 理由 |
|---|---|---|---|
| `Frame Rate` | 30 | **10** | 软件渲染下 30Hz 刷不动 |
| `ParticleCloud`（PoseArray） | enabled | **disabled** | AMCL 最多 2000 个粒子，每个都是一个箭头图元，软件渲染极耗 CPU |

改完重新 `colcon build --packages-select nav` 即可生效（`--symlink-install`，配置直接同步）。

这两项能显著降低负载，但**治标不治本**。若 RViz 仍卡，还可以继续关：

- `GlobalCostmap` / `LocalCostmap` 显示层（代价地图是 602×488 的大图，重绘开销大）
- `LaserScan` 显示层
- `TF` 显示层（树状结构每帧重建）

## 根治方案

### 方案 A：关闭 Secure Boot 校验（推荐，一次性）

```bash
sudo mokutil --disable-validation
```

执行后会要求设置一个**一次性密码**（8~16 位，随便设，重启后就用不到了）。
重启时 BIOS 会进入蓝色 MOK 管理界面，选择 `Change Secure Boot state`，
按提示输入那个一次性密码，然后重启。之后：

```bash
nvidia-smi          # 应能正常输出
lsmod | grep nvidia # 应能看到 nvidia / nvidia_drm / nvidia_modeset
```

**代价**：降低系统启动链的安全校验。这是个人开发机的常见取舍。

### 方案 B：给 DKMS 模块签名（保留 Secure Boot）

需要自己生成 MOK 密钥对、签名 `.ko`、注册公钥。步骤较多，且每次内核更新后要重新签名（可配 DKMS 自动签）。若实验室有统一安全要求，走这条路。

### 方案 C：不开 RViz，用其他方式观察

```bash
ros2 launch nav navigation.launch.py use_rviz:=false
```

导航本身不依赖 RViz。只是失去了 2D Goal Pose 点击和可视化，需要改用
`ros2 run nav goto_node` 或 `ros2 topic pub /goal_pose` 下发目标。

## 相关但不属于本问题的发现

同一次排查中还发现一个**独立**的严重问题，尚未解决：

ROS 域 1 里存在广播**空节点名**的 participant（`_NODE_NAME_UNKNOWN_`，
至少 31 个端点，属于同事那套程序），会导致域内所有节点同时段错误：

```
dq.builtin: type [INVALID 000...]: ddsi_xt_type_init_impl with invalid type object
→ 12 个进程同时 exit code -11
```

这个问题会让**整个导航栈起不来**，与 RViz 无关。详见另一次排查的结论：
等域里不再有空节点名的 participant 后，再重新验证导航栈。

## 复现检查清单

下次再遇到 RViz 卡死，按顺序确认：

```bash
# 1. 驱动到底加载没有（别看 nvidia_wmi_ec_backlight 就以为加载了）
nvidia-smi
ls /proc/driver/nvidia

# 2. RViz 用的是哪套 GL
PID=$(pgrep -x rviz2)
grep -oE "/usr/lib/x86_64-linux-gnu/(libGLX_[a-z]+|dri/[a-z_]+)\.so[^ ]*" /proc/$PID/maps | sort -u

# 3. CPU 占用
ps -o pid,pcpu,cmd -p $PID

# 4. 日志里的丢包
grep -c "queue is full" ~/.ros/log/<最新运行目录>/rviz2_*.log
```
