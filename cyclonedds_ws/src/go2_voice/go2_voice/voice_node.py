#!/usr/bin/env python3
"""Go2 语音控制：回车触发录音 -> faster-whisper 中英文识别 -> DeepSeek 解析意图
-> /api/sport/request 执行：前进与转弯 (1008 MOVE)、坐下 (1009)、起立 (1010)、打招呼 (1016)。

安全约束：白名单内无后退动作（vx>=0），步数 1~5，转角 1~360 度，
LLM 任何非法输出一律不执行。
--dry-run 只打印不发布；--text "..." 跳过录音直接测解析链路。
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
TURN_SPEED = 1.0       # rad/s
RATE_HZ = 20.0         # MOVE 发布频率
MAX_STEPS = 5
MAX_TURN_DEG = 360.0
REC_MAX_S = 3.0        # 单次录音上限
SILENCE_STOP_S = 0.5   # 静音自动断句（压缩执行延迟）
WAIT_OPEN_S = 4.0      # 回车后等待开口的上限
VOICE_RMS = 0.04       # 检测开口的能量阈值

ALLOWED = {'forward', 'turn_left', 'turn_right', 'stop', 'sit', 'stand_up', 'handshake'}

SYSTEM_PROMPT = (
    '你是机器人语音指令解析器。把用户的中英文口语解析为一个 JSON 动作指令，'
    '只输出一个 JSON 对象，不要输出任何其他文字。规则（中英均可）：\n'
    '- 前进 n 步 / go/walk/move forward n steps → {"action":"forward","value":n}\n'
    '- 左转 x 度 / turn left x degrees → {"action":"turn_left","value":x}\n'
    '- 右转 x 度 / turn right x degrees → {"action":"turn_right","value":x}\n'
    '- 向后转 → {"action":"turn_right","value":180}\n'
    '- 停止/停/别动 / stop → {"action":"stop","value":0}\n'
    '- 坐下/坐 / sit down → {"action":"sit","value":0}\n'
    '- 起立/站起来/起来 / stand up → {"action":"stand_up","value":0}\n'
    '- 打招呼/握手/挥挥爪/你好 / shake hands/hello/hi → {"action":"handshake","value":0}\n'
    '- 后退/倒退/向后 / go back/backward → {"action":"rejected","value":0}\n'
    '- 听不懂或与机器人运动无关 → {"action":"unclear","value":0}\n'
    'n 为 1~5 的整数（未说步数则默认 1）；x 为 1~360 的整数（未说角度则默认 90）。'
)

WHISPER_PROMPT = (
    '机器人中英文口令：前进一步、前进两步、左转九十度、右转四十五度、'
    '停止、坐下、起立、打招呼、shake hands、turn left、turn right、'
    'go forward、stop、sit down、stand up.'
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
    if action in ('stop', 'sit', 'stand_up', 'handshake'):
        return (action, 0)
    if action == 'forward':
        return ('forward', value) if 1 <= value <= MAX_STEPS else None
    if 1 <= value <= MAX_TURN_DEG:      # turn_left / turn_right
        return (action, value)
    return None


def record_audio():
    """回车触发后录音：先等开口（最长 4s），开口后静音 0.5s 或满 3s 停止。"""
    import sounddevice as sd
    frames = []
    hot = 0            # 防抖：连续 3 块（300ms）超阈值才算开口
    started = False
    silent = 0.0
    total = 0.0
    wait = 0.0
    with sd.InputStream(samplerate=WAV_RATE, channels=1, dtype='float32') as stream:
        while True:
            data, _ = stream.read(WAV_RATE // 10)     # 100ms 一块
            data = data[:, 0]
            rms = float(np.sqrt(np.mean(data * data)))
            if not started:                            # 阶段一：等待开口
                wait += 0.1
                if rms > VOICE_RMS:
                    hot += 1
                    if hot >= 3:
                        frames.append(data)           # 开口这一块也要录
                        started = True
                        silent = 0.0
                elif wait >= WAIT_OPEN_S:
                    return None                        # 一直没开口
                continue
            frames.append(data)                       # 阶段二：正式录音
            total += 0.1
            if rms > VOICE_RMS:
                silent = 0.0
            else:
                silent += 0.1
            if silent >= SILENCE_STOP_S or total >= REC_MAX_S:
                break
    return np.concatenate(frames)


class Go2VoiceNode(Node):
    def __init__(self, dry_run=False):
        super().__init__('go2_voice_node')
        self.pub = self.create_publisher(
            Request, '/api/sport/request',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.dry_run = dry_run
        self.key = load_api_key()
        if not self.key:
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
        """触发一次性技能动作（1009=坐下，1010=起立），固件自动完成整个动作序列。"""
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
        elif action == 'sit':
            self.get_logger().info('执行：坐下')
            self._publish_skill(1009)
            return
        elif action == 'stand_up':
            self.get_logger().info('执行：起立')
            self._publish_skill(1010)
            return
        elif action == 'handshake':
            self.get_logger().info('执行：打招呼（挥爪致意）')
            self._publish_skill(1016)
            return
        elif action == 'forward':
            self.get_logger().info(
                '执行：前进 %d 步（约 %.1fm）' % (value, value * STEP_METERS))
            args = (SPEED, 0.0, value * STEP_METERS / SPEED)
        else:
            dur = math.radians(value) / TURN_SPEED
            sign = 1.0 if action == 'turn_left' else -1.0   # z 正=左转
            side = '左' if action == 'turn_left' else '右'
            self.get_logger().info('执行：%s转 %d 度（约 %.1fs）' % (side, value, dur))
            args = (0.0, sign * TURN_SPEED, dur)
        t = threading.Thread(target=self._move, args=(*args, ev), daemon=True)
        self.action_thread = t
        t.start()

    # ---------- 指令处理 ----------
    def handle_text(self, text):
        self.get_logger().info('识别：%s' % text)
        if not self.key:
            self.get_logger().warn('无 DeepSeek key，跳过解析')
            return
        try:
            obj = ask_deepseek(text, self.key)
        except Exception as exc:
            self.get_logger().error('DeepSeek 调用失败：%s' % exc)
            return
        self.get_logger().info('LLM：%s' % json.dumps(obj, ensure_ascii=False))
        act = validate(obj)
        if act is None:
            self.get_logger().warn(
                '安全校验拒绝（仅支持：前进1~5步 / 左右转1~360度 / 停止 / 坐下 / 起立 / 打招呼）')
            return
        action, value = act
        if self.dry_run:
            self.get_logger().info('[dry-run] 将执行 %s value=%d' % (action, value))
            return
        self.run_action(action, value)

    # ---------- 主循环 ----------
    def main_loop(self):
        from faster_whisper import WhisperModel
        self.get_logger().info('加载 whisper small(int8)…')
        model = WhisperModel('small', device='cpu', compute_type='int8')
        self.get_logger().info('就绪。按回车开始说话，说完停顿即识别。Ctrl+C 退出')
        while rclpy.ok():
            try:
                input('按回车开始说话 > ')
            except (KeyboardInterrupt, EOFError):
                break
            try:
                audio = record_audio()
            except KeyboardInterrupt:
                break
            except Exception as exc:
                self.get_logger().error('录音失败：%s' % exc)
                time.sleep(1.0)
                continue
            if audio is None:
                self.get_logger().info('未检测到语音，按回车再试')
                continue
            segments, _ = model.transcribe(
                audio, language=None, beam_size=1, initial_prompt=WHISPER_PROMPT)
            text = ''.join(s.text for s in segments
                           if s.no_speech_prob < 0.6).strip()
            if not text:
                self.get_logger().info('识别为空，再试一次')
                continue
            self.handle_text(text)
        self.abort.set()
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass    # 退出时 context 可能已关闭
        self.get_logger().info('退出，已发零速')


def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv[1:]
    dry = '--dry-run' in argv
    text_mode = '--text' in argv
    node = Go2VoiceNode(dry_run=dry)
    try:
        if text_mode:                      
            i = argv.index('--text')
            text = argv[i + 1] if i + 1 < len(argv) else '前进两步'
            node.handle_text(text)
        else:
            node.main_loop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
