#!/usr/bin/env python3
"""语音控制 Go2：回车录音 -> faster-whisper 中文识别 -> DeepSeek 解析意图
-> /api/sport/request (1008 MOVE) 执行。

安全约束：白名单内无后退动作（vx>=0），步数 1~3，转角 1~90 度，
LLM 任何非法输出一律不执行。--dry-run 只打印不发布。
"""
import json
import math
import os
import re
import sys
import threading
import time

os.environ.setdefault('HF_ENDPOINT', 'https://hf-mirror.com')
os.environ.setdefault('HF_HUB_DISABLE_XET', '1')   # xet 协议绕过镜像会被 401

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request

WAV_RATE = 16000
STEP_METERS = 0.4      # "一步" = 0.4m（0.4m/s 走 1s）
SPEED = 0.4            # m/s
TURN_SPEED = 1       # rad/s
RATE_HZ = 20.0         # MOVE 发布频率
MAX_STEPS = 5
MAX_TURN_DEG = 360.0
REC_MAX_S = 2.0        # 单次录音上限
SILENCE_STOP_S = 0.5   # 静音自动断句（压缩执行延迟）
VOICE_RMS = 0.04       # 起始语音能量阈值（持续监听模式下防环境噪声误唤醒）

ALLOWED = {'forward', 'turn_left', 'turn_right', 'stop', 'handshake', 'sit', 'stand_up'}

SYSTEM_PROMPT = (
    '你是机器人语音指令解析器。把用户的中英文口语解析为一个 JSON 动作指令，'
    '只输出一个 JSON 对象，不要输出任何其他文字。规则（中英均可）：\n'
    '- 前进 n 步 / go/walk/move forward n steps → {"action":"forward","value":n}\n'
    '- 向后转 → {"action":"turn_right","value":180}\n'
    '- 左转 x 度 / turn left x degrees → {"action":"turn_left","value":x}\n'
    '- 右转 x 度 / turn right x degrees → {"action":"turn_right","value":x}\n'
    '- 停止 / stop → {"action":"stop","value":0}\n'
    '- 握手/打招呼/挥挥爪/你好 / shake hands/hello/hi → {"action":"handshake","value":0}\n'
    '- 坐下/坐 / sit down → {"action":"sit","value":0}\n'
    '- 起立/站起来/起来 / stand up → {"action":"stand_up","value":0}\n'
    '- 后退/倒退/向后 / go back/backward/step back → {"action":"rejected","value":0}\n'
    '- 听不懂或与机器人运动无关 → {"action":"unclear","value":0}\n'
    'n 为 1~5 的整数（未说步数则默认 1）；x 为 1~360 的整数（未说角度则默认 90）。'
)


def load_api_key():
    key = os.environ.get('DEEPSEEK_API_KEY', '').strip()
    if key:
        return key
    path = os.path.expanduser('~/.deepseek_key')
    if os.path.exists(path):
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            return None
    return None


def ask_deepseek(text, key):
    for attempt in range(3):    # 偶发网络超时自动重试
        try:
            r = requests.post(
                'https://api.deepseek.com/chat/completions',
                headers={'Authorization': 'Bearer ' + key},
                json={
                    'model': 'deepseek-chat',
                    'messages': [
                        {'role': 'system', 'content': SYSTEM_PROMPT},
                        {'role': 'user', 'content': text},
                    ],
                    'temperature': 0,
                    'max_tokens': 100,
                },
                timeout=8)
            r.raise_for_status()
            break
        except (requests.Timeout, requests.ConnectionError):
            if attempt == 2:
                raise
            time.sleep(0.5)
    content = r.json()['choices'][0]['message']['content'].strip()
    m = re.search(r'\{.*\}', content, re.S)
    if not m:
        return {'action': 'unclear', 'value': 0}
    try:
        obj = json.loads(m.group(0))
    except json.JSONDecodeError:
        return {'action': 'unclear', 'value': 0}
    if not isinstance(obj, dict) or 'action' not in obj:
        return {'action': 'unclear', 'value': 0}
    obj.setdefault('value', 0)
    return obj


def validate(obj):
    """白名单二次校验（不信任 LLM 输出），返回 (action, value) 或 None。"""
    action = str(obj.get('action', '')).lower()
    try:
        value = int(obj.get('value') or 0)
    except (TypeError, ValueError):
        return None
    if action not in ALLOWED:
        return None
    if action == 'stop':
        return ('stop', 0)
    if action == 'handshake':
        return ('handshake', 0)
    if action == 'sit':
        return ('sit', 0)
    if action == 'stand_up':
        return ('stand_up', 0)
    if action == 'forward':
        return ('forward', value) if 1 <= value <= MAX_STEPS else None
    if 1 <= value <= MAX_TURN_DEG:      # turn_left / turn_right
        return (action, value)
    return None


def record_audio():
    """按回车后开始录音；静音 0.8s 或满 3s 停止。返回 float32 数组或 None。"""
    import sounddevice as sd
    frames = []
    hot = 0            # 防抖：连续 3 块（300ms）超阈值才算开始说话
    started = False
    silent = 0.0
    total = 0.0
    with sd.InputStream(samplerate=WAV_RATE, channels=1, dtype='float32') as stream:
        while total < REC_MAX_S:
            data, _ = stream.read(WAV_RATE // 10)     # 100ms 一块
            data = data[:, 0]
            rms = float(np.sqrt(np.mean(data * data)))
            frames.append(data)
            total += 0.1
            if rms > VOICE_RMS:
                hot += 1
                if hot >= 3:
                    started = True
                    silent = 0.0
            elif started:
                silent += 0.1
                if silent >= SILENCE_STOP_S:
                    break
    if not started:
        return None
    return np.concatenate(frames)


class VoiceNode(Node):
    def __init__(self, dry_run=False):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(
            Request, '/api/sport/request',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.dry_run = dry_run
        self.key = load_api_key()
        if not self.key and not dry_run:
            self.get_logger().warn(
                '未找到 DeepSeek key（环境变量 DEEPSEEK_API_KEY 或 ~/.deepseek_key）')
        self.abort = threading.Event()
        self.action_thread = None

    # ---------- 运动执行：20Hz 发布 1008 MOVE ----------
    def _publish(self, vx, wz):
        req = Request()
        req.header.identity.api_id = 1008
        req.header.identity.id = 0
        req.header.lease.id = 0
        req.header.policy.noreply = True
        req.parameter = json.dumps({'x': float(vx), 'y': 0.0, 'z': float(wz)})
        self.pub.publish(req)

    def _move(self, vx, wz, seconds, abort):
        for _ in range(int(seconds * RATE_HZ)):
            if abort.is_set():
                break
            self._publish(vx, wz)
            time.sleep(1.0 / RATE_HZ)
        for _ in range(int(RATE_HZ)):    # 尾部零速 1s，确保停稳
            if abort.is_set():
                return
            self._publish(0.0, 0.0)
            time.sleep(1.0 / RATE_HZ)

    def _publish_skill(self, api_id):
        """触发一次性技能动作（1016=握手），固件自动完成整个动作序列。"""
        req = Request()
        req.header.identity.api_id = api_id
        req.header.identity.id = 0
        req.header.lease.id = 0
        req.header.policy.noreply = True
        req.parameter = '{}'
        self.pub.publish(req)

    def run_action(self, action, value):
        """打断旧动作，启动新动作。stop=立即连发零速。"""
        self.abort.set()
        if self.action_thread and self.action_thread.is_alive():
            self.action_thread.join(timeout=1.0)
        self.abort = threading.Event()
        ev = self.abort
        if action == 'stop':
            self.get_logger().info('执行：停止')
            args = (0.0, 0.0, 0.0)
        elif action == 'handshake':
            self.get_logger().info('执行：握手（挥爪致意）')
            self._publish_skill(1016)
            return
        elif action == 'sit':
            self.get_logger().info('执行：坐下')
            self._publish_skill(1009)
            return
        elif action == 'stand_up':
            self.get_logger().info('执行：起立')
            self._publish_skill(1010)
            return
        elif action == 'forward':
            self.get_logger().info(
                '执行：前进 %d 步（约 %.1fm）' % (value, value * STEP_METERS))
            args = (SPEED, 0.0, value * STEP_METERS / SPEED)
        else:
            dur = math.radians(value) / TURN_SPEED
            sign = 1.0 if action == 'turn_left' else -1.0   # z 正=左转
            side = '左' if action == 'turn_left' else '右'
            self.get_logger().info('执行：%s转 %d 度（约 %.1fs）' % (side, value, dur*1.3))
            args = (0.0, sign * TURN_SPEED, dur*1.3)
        t = threading.Thread(target=self._move, args=(*args, ev), daemon=True)
        self.action_thread = t
        t.start()

    # ---------- 主循环 ----------
    def main_loop(self):
        from faster_whisper import WhisperModel
        self.get_logger().info('加载 whisper small(int8)…')
        model = WhisperModel('small', device='cpu', compute_type='int8')
        self.get_logger().info('就绪。持续监听中，直接说话即可。Ctrl+C 退出')
        while rclpy.ok():
            try:
                audio = record_audio()
            except KeyboardInterrupt:
                break
            except Exception as exc:
                self.get_logger().error('录音失败：%s' % exc)
                break
            if audio is None:
                continue    # 待机中未检测到语音，继续监听
            segments, _ = model.transcribe(
                audio, language=None, beam_size=1,
                initial_prompt='机器人中英文口令：前进一步、左转九十度、右转四十五度、'
                               '停止、握手、坐下、起立、stand up、sit down、'
                               'turn left、turn right、stop、shake hands、go forward.')
            text = ''.join(s.text for s in segments
                           if s.no_speech_prob < 0.6).strip()
            if not text:
                self.get_logger().info('识别为空，再试一次')
                continue
            self.get_logger().info('识别：%s' % text)
            if not self.key:
                self.get_logger().warn('无 DeepSeek key，跳过解析（key 放 ~/.deepseek_key）')
                continue
            try:
                obj = ask_deepseek(text, self.key)
            except Exception as exc:
                self.get_logger().error('DeepSeek 调用失败：%s' % exc)
                continue
            self.get_logger().info('LLM：%s' % json.dumps(obj, ensure_ascii=False))
            act = validate(obj)
            if act is None:
                self.get_logger().warn(
                    '安全校验拒绝（仅支持：前进1~5步 / 360度 / 停止 / 握手 / 坐下 / 起立）')
                continue
            action, value = act
            if self.dry_run:
                self.get_logger().info('[dry-run] 将执行 %s value=%d' % (action, value))
                continue
            self.run_action(action, value)
        self.abort.set()
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass    # 退出时 context 可能已关闭
        self.get_logger().info('退出，已发零速')

def main(args=None):
    rclpy.init(args=args)
    dry = '--dry-run' in sys.argv
    node = VoiceNode(dry_run=dry)
    try:
        node.main_loop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
