#!/bin/bash
echo "Setup unitree ros2 environment"
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 自动检测可用网卡：优先有线 (enp)，其次 WiFi (wlp)，选已 UP 的
# 插网线时走有线→域0，拔网线自动回退 WiFi→域1
NET_IF=$(ip -o link show | awk -F': ' '/enp/{print $2}' | while read iface; do
    if ip addr show "$iface" 2>/dev/null | grep -q 'state UP'; then
        echo "$iface"; break
    fi
done)
[ -z "$NET_IF" ] && NET_IF=$(ip -o link show | awk -F': ' '/wlp/{print $2}' | while read iface; do
    if ip addr show "$iface" 2>/dev/null | grep -q 'state UP'; then
        echo "$iface"; break
    fi
done)
# 兜底
[ -z "$NET_IF" ] && NET_IF=$(ip -o link show | awk -F': ' '/(wlp|enp)/{print $2; exit}')

# WiFi(wlp) 走域1，有线(enp) 走域0
if echo "$NET_IF" | grep -q '^wlp'; then
    export ROS_DOMAIN_ID=1
else
    export ROS_DOMAIN_ID=0
fi

if [ -n "$NET_IF" ]; then
    echo "[setup] 使用网卡: $NET_IF, ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                                <NetworkInterface name=\"$NET_IF\" priority=\"default\" multicast=\"default\" />
                            </Interfaces></General></Domain></CycloneDDS>"
else
    echo "[setup] 警告: 未找到合适网卡，CycloneDDS 将自动选择"
fi
