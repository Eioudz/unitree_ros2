# Go2 接入 DeepSeek：语音控制机器人前进

## Context

用户要用自然语言语音控制 Go2（例："前进一步"）。链路为：笔记本麦克风录音 → 本地语音识别 → DeepSeek 大模型解析意图 → 通过已验证的 `/api/sport/request`（api_id 1008 MOVE）运动链路驱动 Go2。

环境侦察已确认：
- 麦克风：笔记本内置 ALC294 ✓
- 外网/DeepSeek API/PyPI 全部可达 ✓
- 用户已有 DeepSeek API key（稍后提供），按键触发录音，1 步 = 0.4m

## 架构

```
[回车] → sounddevice 录音(16kHz mono, 静音0.8s自动断句, 最长5s)
      → faster-whisper small(int8) 中文识别
      → DeepSeek chat API (temperature=0, 强约束输出 JSON)
      → 白名单校验 + 限幅（禁后退/步数≤3/角度≤90°）
      → 动作线程 20Hz 发布 1008 MOVE，按时间走完 → 尾发1s零速
```

单一 ROS2 节点，跑在笔记本（域 1，走 WiFi 中继，已验证满速率）。

## 新建文件

包：`cyclonedds_ws/src/voice_control`（ament_python）

| 文件 | 内容 |
|---|---|
| `voice_control/voice_node.py` | 主节点（唯一源文件，~200 行） |
| `package.xml` / `setup.py` | 标准 ament_python 骨架 |

## voice_node.py 关键设计

**1. 运动执行（复用 [follow.py#L177-L189](file:///home/soyo/unitree_ros2/cyclonedds_ws/src/follow/follow/follow.py) 已验证的 Request 构造）**
```python
req.header.identity.api_id = 1008  # MOVE
req.header.lease.id = 0; req.header.policy.noreply = True
req.parameter = json.dumps({"x": vx, "y": 0.0, "z": wz})
```
- QoS/publisher 声明与 follow.py 相同
- `forward(n步)`: vx=0.4 持续 n 秒（0.4m/s×1s=0.4m/步），20Hz 发
- `turn(方向, 角度)`: wz=0.5rad/s 持续 (角度/0.5) 秒
- `stop()`: 立即零速
- 动作在子线程执行，新指令可打断当前动作（abort event）；动作结束追加 1s 零速确保停稳

**2. DeepSeek 调用**
- `POST https://api.deepseek.com/chat/completions`，model=deepseek-chat，temperature=0
- system prompt 强约束：只输出 JSON `{"action":"forward|turn_left|turn_right|stop","value":N}`；指令含"后退/倒退"时返回 `{"action":"rejected"}`；解析不了返回 `{"action":"unclear"}`
- key 读取顺序：环境变量 `DEEPSEEK_API_KEY` → `~/.deepseek_key` 文件（600 权限）
- 超时 8s；失败不执行任何动作并日志说明
- LLM 输出必须过白名单校验：action 合法、步数 1~3、角度 1~90，越界一律拒绝

**3. 录音（sounddevice）**
- 默认输入设备（ALC294），16kHz mono；回车开始，静音 0.8s 或满 5s 自动停（能量阈值断句）

**4. 语音识别（faster-whisper）**
- 模型 `small` + int8（CPU ~2-4s 处理 5s 音频，中文够用）
- 模型下载走 HF 镜像：`HF_ENDPOINT=https://hf-mirror.com`（首帧前设置）
- 识别文本打印到日志（全链路可观测：识别文本 → LLM JSON → 执行动作）

**5. 安全硬约束（来自 project memory）**
- 白名单无 backward——物理上禁后退（vx ≥ 0）
- LLM 任何非法输出 → 不动
- 无 key / 断网 / 识别空文本 → 不动，仅日志提示

## 安装步骤

1. `pip install --user faster-whisper sounddevice`（PyPI 已确认可达）
2. 用户提供 key 后：`echo 'sk-xxx' > ~/.deepseek_key && chmod 600 ~/.deepseek_key`
3. `colcon build --packages-select voice_control`

## 验证（分层，先安全后真机）

1. **STT 层**：跑节点前先用一次性脚本录 4s 说"前进一步"，whisper 转写打印——确认录音设备+模型+中文识别
2. **LLM 层**：用 key 直接 curl DeepSeek 一次，验证 key 有效 + prompt 输出格式正确
3. **dry-run**：节点加 `--dry-run` 参数，只打印将执行的动作不发布——全链路文字验证
4. **真机**：空旷处低速测试
   - "前进一步" → 走 0.4m 停 ✓
   - "左转 90 度" → 原地转 ✓
   - "后退一步" → 拒绝（日志显示 rejected）✓
   - "停止" → 立即零速 ✓

## 交用户提供

- DeepSeek API key（`sk-` 开头），我负责写入 `~/.deepseek_key`
