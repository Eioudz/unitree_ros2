#!/bin/bash
# verify_go2_topics.sh —— 在新电脑上完整验证能否发现并接收 Go2 中继话题
#
# 用法:  bash verify_go2_topics.sh
# 可覆盖: DOG_IP=172.18.115.230  DOMAIN_ID=1  IFACE=wlp129s0f0  bash verify_go2_topics.sh
#
# 说明:
#   - 狗上 go2_wifi_relay 通过 wlan0(USB 网卡 rtw_8822bu)连 lab-410,在 ROS 域 1 广播
#   - KEEP_FWD 优化后正常可见约 46 个话题 = 12 正向(建图/控制必需) + 32 反向 + rosout/parameter_events
#     (另有 4 个反向话题因狗上缺 unitree_arm/unitree_interfaces 消息包被跳过,属预期)
#   - 本脚本只做只读验证(list/hz/echo/ping),不会向狗发送任何控制指令
#   - unitree_go 等自定义类型话题,在没有编译消息包的机器上只能 list,不能 echo/hz

set -u

DOG_IP="${DOG_IP:-172.18.115.230}"
DOMAIN_ID="${DOMAIN_ID:-1}"
IFACE="${IFACE:-}"   # 留空则自动检测(取通往狗的网卡)

PASS=0; FAIL=0; WARN=0
SUMMARY=()

red()    { printf '\033[31m%s\033[0m\n' "$*"; }
green()  { printf '\033[32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[33m%s\033[0m\n' "$*"; }
info()   { printf '\033[36m%s\033[0m\n' "$*"; }

record() {  # record PASS|WARN|FAIL "描述"
  SUMMARY+=("$1|$2")
  case "$1" in
    PASS) green  "  [PASS] $2"; PASS=$((PASS+1)) ;;
    WARN) yellow "  [WARN] $2"; WARN=$((WARN+1)) ;;
    FAIL) red    "  [FAIL] $2"; FAIL=$((FAIL+1)) ;;
  esac
}

echo "============================================================"
echo " Go2 中继话题验证脚本(只读,不控狗)"
echo " 目标: $DOG_IP   ROS 域: $DOMAIN_ID   WiFi: lab-410 (2.4G)"
echo "============================================================"

# ---------- [1/7] ROS 2 环境 ----------
echo
echo "[1/7] 检查 ROS 2 环境"
# 注意: ROS 的 setup 脚本在 set -u 下会报未绑定变量,source 前后临时切换
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  DISTRO=""
  for d in humble foxy galactic iron jazzy; do
    [ -f "/opt/ros/${d}/setup.bash" ] && DISTRO="$d" && break
  done
  if [ -z "$DISTRO" ]; then
    for f in /opt/ros/*/setup.bash; do
      [ -f "$f" ] && DISTRO="$(basename "$(dirname "$f")")" && break
    done
  fi
  if [ -z "$DISTRO" ]; then
    record FAIL "未找到 /opt/ros/<发行版>/setup.bash,请先安装 ROS 2"
    red "结果: 环境验证未通过"; exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${DISTRO}/setup.bash"
  set -u
fi
record PASS "ROS 2 发行版: ${ROS_DISTRO}"
command -v ros2 >/dev/null 2>&1 || { record FAIL "ros2 CLI 不可用"; exit 1; }

# ~/.ros 不可写时避免 ros2 CLI 刷权限错误
mkdir -p "$HOME/.ros/log" 2>/dev/null || export ROS_LOG_DIR=/tmp/roslog

# 可选:若本机编译过 unitree_ros2 工作区则加载(解锁自定义类型 echo/hz/pub)
if [ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
  set -u
  record PASS "已加载 ~/unitree_ros2 工作区,自定义类型(unitree_go 等)可用"
else
  record WARN "未找到 ~/unitree_ros2 工作区:unitree_go 类型话题只能 list,不能 echo/hz/pub"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "/opt/ros/${ROS_DISTRO}/lib/librmw_cyclonedds_cpp.so" ] || python3 -c "import rmw_cyclonedds_cpp" >/dev/null 2>&1; then
  record PASS "RMW: rmw_cyclonedds_cpp 可用"
else
  record FAIL "rmw_cyclonedds_cpp 不可用 → sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-dds"
  red "结果: 环境验证未通过"; exit 1
fi

if [ "${ROS_LOCALHOST_ONLY:-0}" != "0" ]; then
  unset ROS_LOCALHOST_ONLY
  record WARN "ROS_LOCALHOST_ONLY 非 0,已自动 unset"
else
  record PASS "ROS_LOCALHOST_ONLY 未限制本机"
fi

# ---------- [2/7] 网络连通 ----------
echo
echo "[2/7] 检查网络(需在 172.18.115.0/24 网段)"
if ip -4 -o addr show scope global 2>/dev/null | grep -q 'inet 172\.18\.115\.'; then
  record PASS "本机已获得 172.18.115.x 地址"
else
  record FAIL "本机无 172.18.115.x 地址:请先连上 WiFi Xiaomi_B3B5_5G"
  red "结果: 环境验证未通过"; exit 1
fi

ROUTE_LINE=$(ip route get "$DOG_IP" 2>/dev/null | head -1)
if [ -z "$ROUTE_LINE" ]; then
  record FAIL "无到 $DOG_IP 的路由,检查 WiFi 连接"
  exit 1
fi
if ping -c 2 -W 2 "$DOG_IP" >/dev/null 2>&1; then
  record PASS "ping $DOG_IP 正常"
else
  record WARN "ping $DOG_IP 不通(DDS 仍可能工作,但需警惕狗离线/AP 隔离)"
fi

# ---------- [3/7] DDS 网卡绑定 ----------
echo
echo "[3/7] 配置 CycloneDDS(显式绑单网卡,规避多网卡 locator 歧义)"
if [ -z "$IFACE" ]; then
  IFACE=$(echo "$ROUTE_LINE" | grep -oP 'dev \K\S+' | head -1)
fi
if [ -z "$IFACE" ] || ! ip link show "$IFACE" >/dev/null 2>&1; then
  record FAIL "无法确定使用的网卡,请手动指定: IFACE=<网卡名> bash $0"
  exit 1
fi
MY_IP=$(echo "$ROUTE_LINE" | grep -oP 'src \K\S+' | head -1)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'"${IFACE}"'" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
export ROS_DOMAIN_ID="$DOMAIN_ID"
record PASS "CycloneDDS 绑定网卡 $IFACE (本机 IP: ${MY_IP:-未知}),ROS_DOMAIN_ID=$DOMAIN_ID"

OTHER_IF=$(ip -4 -o addr show scope global 2>/dev/null | awk -v keep="$IFACE" '$2 != keep {print $2}' | sort -u | tr '\n' ' ')
if [ -n "$OTHER_IF" ]; then
  record WARN "存在其他全局网卡($OTHER_IF) —— 已强制绑定 $IFACE,防止 DDS 选错"
fi

# ---------- [4/7] 话题发现 ----------
echo
echo "[4/7] 发现中继话题(先停 daemon 防缓存)"
ros2 daemon stop >/dev/null 2>&1
TOPICS=$(timeout 20 ros2 topic list 2>/dev/null | sort)
if [ -z "$TOPICS" ]; then
  record FAIL "topic list 为空:狗未开机/中继未运行/域ID不对/网卡绑错"
  exit 1
fi
COUNT=$(echo "$TOPICS" | grep -c .)
	if [ "$COUNT" -ge 40 ]; then
		record PASS "共发现 $COUNT 个话题(参考值约 46 = 12 正向 KEEP_FWD + 32 反向 + 2 系统话题)"
	else
		record WARN "仅发现 $COUNT 个话题(参考约 46):可能中继部分分片掉线"
	fi

for t in /utlidar/robot_odom /utlidar/cloud_base; do
  echo "$TOPICS" | grep -qx "$t" && record PASS "核心话题存在: $t" || record FAIL "缺少核心话题: $t"
done
for t in /lowstate /sportmodestate /livox/lidar /api/sport/request; do
  echo "$TOPICS" | grep -qx "$t" && record PASS "关键话题存在: $t" || record WARN "缺少话题: $t"
done

# ---------- [5/7] 数据速率 ----------
echo
echo "[5/7] 数据流验证(每项约 8 秒)"
hz_check() {  # hz_check <话题> <最低Hz>
  local rate
  if ! echo "$TOPICS" | grep -qx "$1"; then
    record FAIL "$1 不在话题列表,跳过 hz"
    return
  fi
  rate=$(timeout 8 stdbuf -oL ros2 topic hz "$1" 2>&1 | grep -oP 'average rate: \K[0-9.]+' | tail -1)
  if [ -z "$rate" ]; then
    record FAIL "$1: 订阅后未收到数据"
  elif awk -v r="$rate" -v m="$2" 'BEGIN{exit !(r>=m)}'; then
    info "  $1 平均速率: ${rate} Hz"
    record PASS "$1 速率 ${rate} Hz(≥ $2)"
  else
    record FAIL "$1 速率仅 ${rate} Hz(应 ≥ $2)"
  fi
}
hz_check /utlidar/robot_odom 90    # 原生 150Hz
hz_check /utlidar/cloud_base 5     # 原生 ~14.4Hz
hz_check /livox/lidar 5            # 原生 10Hz

if timeout 10 ros2 interface show unitree_go/msg/LowState >/dev/null 2>&1; then
  record PASS "unitree_go 消息包可用,继续测自定义类型"
	hz_check /lowstate 70              # 中继源 ~100Hz
	hz_check /sportmodestate 180       # 原生 ~297Hz
else
  record WARN "无 unitree_go 消息包:跳过 /lowstate /sportmodestate 测试(如需,编译 unitree_ros2)"
fi

# ---------- [6/7] 反序列化冒烟 ----------
echo
echo "[6/7] 反序列化冒烟测试"
ECHO_OUT=$(timeout 5 ros2 topic echo /utlidar/robot_odom --no-arr 2>/dev/null | head -n 4)
if [ -n "$ECHO_OUT" ]; then
  record PASS "echo /utlidar/robot_odom 正常(nav_msgs/Odometry)"
else
  record WARN "echo 无输出:老版本可能不支持 --no-arr,可手动 ros2 topic echo /utlidar/robot_odom"
fi

# ---------- [7/7] 汇总 ----------
echo
echo "[7/7] 汇总"
echo "------------------------------------------------------------"
for s in "${SUMMARY[@]}"; do
  printf ' %-4s %s\n' "${s%%|*}" "${s#*|}"
done
echo "------------------------------------------------------------"
echo
echo "以后日常使用(连上 lab-410 后执行):"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo "  export ROS_DOMAIN_ID=${DOMAIN_ID}"
echo "  unset ROS_LOCALHOST_ONLY"
echo "  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>'"
echo
echo "安全提醒: 域 ${DOMAIN_ID} 无鉴权,同 WiFi 的任何设备都能发 /api/sport/request 控狗;"
echo "          紧急断链: ssh unitree@${DOG_IP} 后执行 sudo systemctl stop go2_wifi_relay"
echo "============================================================"
if [ "$FAIL" -gt 0 ]; then
  red "结果: ${PASS} 通过 / ${WARN} 警告 / ${FAIL} 失败 —— 未达可用标准"
  exit 1
fi
green "结果: ${PASS} 通过 / ${WARN} 警告 / 0 失败 —— 验证通过,可正常使用"
