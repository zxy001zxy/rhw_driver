from __future__ import annotations

import audioop
import importlib
import json
import os
import re
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

KaldiRecognizer = None
Model = None


def ensure_vosk_imported() -> tuple[object, object]:
    global KaldiRecognizer, Model
    if KaldiRecognizer is not None and Model is not None:
        return KaldiRecognizer, Model

    try:
        vosk_module = importlib.import_module('vosk')
    except ImportError as exc:
        print('[ERROR] 未安装 vosk，请先执行: python3 -m pip install vosk')
        raise SystemExit(1) from exc

    vosk_module.SetLogLevel(-1)
    KaldiRecognizer = vosk_module.KaldiRecognizer
    Model = vosk_module.Model
    return KaldiRecognizer, Model


PACKAGE_NAME = 'rhw_firedog_voice'
ROS2_TOPIC_NAME = '/voice_command'
GRAMMAR = ['前进', '后退', '左转', '右转']
COMMAND_PATTERNS = {
    'MOVE_FORWARD': [r'前进'],
    'MOVE_BACKWARD': [r'后退'],
    'TURN_LEFT': [r'左转'],
    'TURN_RIGHT': [r'右转'],
}

# 你这台板子上已经确认是这个
AUDIO_DEV = 'plughw:2,0'

CHANNELS = 2
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2
CHUNK_FRAMES = 4000
RAW_CHUNK_BYTES = CHUNK_FRAMES * CHANNELS * SAMPLE_WIDTH
PUBLISH_COOLDOWN_SEC = 1.0


def get_package_base_dir() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass
    return Path(__file__).resolve().parents[1]


PACKAGE_BASE_DIR = get_package_base_dir()
MODEL_PATH = PACKAGE_BASE_DIR / 'model' / 'vosk-model-small-cn-0.22'
LOG_PATH = Path.home() / '.ros' / PACKAGE_NAME / 'asr_demo_log.jsonl'


def normalize_text(text: str) -> str:
    return text.replace(' ', '').strip()


def match_command(text: str):
    normalized = normalize_text(text)
    for command, patterns in COMMAND_PATTERNS.items():
        for pattern in patterns:
            if re.search(pattern, normalized):
                return command
    return None


def pcm_rms(pcm_bytes: bytes, sample_width: int) -> int:
    return audioop.rms(pcm_bytes, sample_width)


def choose_best_candidate(candidates_info: list[dict]):
    matched = [item for item in candidates_info if item['command'] is not None]
    if matched:
        return sorted(matched, key=lambda item: item['rms'], reverse=True)[0]

    with_text = [item for item in candidates_info if item['text']]
    if with_text:
        return sorted(with_text, key=lambda item: item['rms'], reverse=True)[0]

    return sorted(candidates_info, key=lambda item: item['rms'], reverse=True)[0]


def append_log(log_item: dict):
    os.makedirs(LOG_PATH.parent, exist_ok=True)
    with open(LOG_PATH, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(log_item, ensure_ascii=False) + '\n')


class VoiceCommandLiveNode(Node):
    def __init__(self):
        super().__init__('voice_command_live_node')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher_ = self.create_publisher(String, ROS2_TOPIC_NAME, qos)
        self.get_logger().info(f'ROS2 publisher ready, topic = {ROS2_TOPIC_NAME}')

        if not MODEL_PATH.is_dir():
            raise RuntimeError(f'模型目录不存在: {MODEL_PATH}')

        recognizer_cls, model_cls = ensure_vosk_imported()
        self.model = model_cls(str(MODEL_PATH))

        self.recs = {
            'left': recognizer_cls(self.model, SAMPLE_RATE, json.dumps(GRAMMAR, ensure_ascii=False)),
            'right': recognizer_cls(self.model, SAMPLE_RATE, json.dumps(GRAMMAR, ensure_ascii=False)),
            'mix': recognizer_cls(self.model, SAMPLE_RATE, json.dumps(GRAMMAR, ensure_ascii=False)),
        }
        for rec in self.recs.values():
            rec.SetWords(True)

        self.segment_buffers = {
            'left': b'',
            'right': b'',
            'mix': b'',
        }

        self.last_publish_time = 0.0
        self.running = True
        self.arecord_proc = None

    def publish_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published ROS2 topic: {cmd}')

    def wait_for_subscribers(self, timeout_sec: float = 0.5) -> int:
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while self.get_clock().now() < deadline:
            count = self.count_subscribers(ROS2_TOPIC_NAME)
            if count > 0:
                return count
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.count_subscribers(ROS2_TOPIC_NAME)

    def publish_command_burst(self, cmd: str, repeat: int = 5, interval_sec: float = 0.1):
        repeat = max(int(repeat), 1)
        interval_sec = max(float(interval_sec), 0.02)

        count = self.wait_for_subscribers(timeout_sec=0.5)
        if count > 0:
            self.get_logger().info(f'Detected {count} subscriber(s) on {ROS2_TOPIC_NAME}')
        else:
            self.get_logger().warning(f'No subscriber detected on {ROS2_TOPIC_NAME}, still publishing')

        for _ in range(repeat):
            self.publish_command(cmd)
            rclpy.spin_once(self, timeout_sec=interval_sec)

    def start_arecord(self):
        cmd = [
            'arecord',
            '-D', AUDIO_DEV,
            '-c', str(CHANNELS),
            '-r', str(SAMPLE_RATE),
            '-f', 'S16_LE',
            '-t', 'raw',
        ]
        self.get_logger().info(f'开始从麦克风实时采集音频，设备: {AUDIO_DEV}')
        self.get_logger().info(f'arecord 命令: {" ".join(cmd)}')
        self.arecord_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        time.sleep(0.2)
        if self.arecord_proc.poll() is not None:
            stderr_text = ''
            try:
                stderr_text = self.arecord_proc.stderr.read().decode('utf-8', errors='ignore')
            except Exception:
                pass
            raise RuntimeError(
                f'arecord 启动失败，当前设备: {AUDIO_DEV}\n错误信息:\n{stderr_text}'
            )

    def stop_arecord(self):
        if self.arecord_proc is not None:
            try:
                self.arecord_proc.terminate()
                self.arecord_proc.wait(timeout=1)
            except Exception:
                try:
                    self.arecord_proc.kill()
                except Exception:
                    pass
            self.arecord_proc = None

    def reset_segment_buffers(self):
        self.segment_buffers = {
            'left': b'',
            'right': b'',
            'mix': b'',
        }

    def read_exact(self, size: int) -> bytes:
        """
        从 arecord stdout 里累计读满 size 字节。
        只有真正 EOF/中断时才返回短数据。
        """
        if self.arecord_proc is None or self.arecord_proc.stdout is None:
            return b''

        chunks = []
        total = 0
        while total < size and self.running and rclpy.ok():
            chunk = self.arecord_proc.stdout.read(size - total)
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
        return b''.join(chunks)

    def handle_segment_end(self, flags: dict[str, bool]):
        candidates_info = []

        for name in ['left', 'right', 'mix']:
            text = ''
            if flags[name]:
                result = json.loads(self.recs[name].Result())
                text = result.get('text', '').strip()

            rms = 0
            if self.segment_buffers[name]:
                rms = pcm_rms(self.segment_buffers[name], SAMPLE_WIDTH)

            cmd = match_command(text)
            info = {
                'channel': name,
                'text': text,
                'command': cmd,
                'rms': rms,
            }
            candidates_info.append(info)

        print('=' * 60)
        for item in candidates_info:
            print(f"[{item['channel']}] RMS      : {item['rms']}")
            print(f"[{item['channel']}] 识别文本 : {item['text'] if item['text'] else '[空]'}")
            print(f"[{item['channel']}] 命令码   : {item['command'] if item['command'] else '[未匹配]'}")
            print('-' * 60)

        best = choose_best_candidate(candidates_info)
        print(f"[最终采用] 通道   : {best['channel']}")
        print(f"[最终采用] RMS    : {best['rms']}")
        print(f"[最终采用] 文本   : {best['text'] if best['text'] else '[空]'}")
        print(f"[最终采用] 命令   : {best['command'] if best['command'] else '[未匹配]'}")

        now = time.time()
        executed = False

        if best['command'] and now - self.last_publish_time >= PUBLISH_COOLDOWN_SEC:
            self.publish_command_burst(best['command'], repeat=5, interval_sec=0.1)
            print(f"[EXEC] ROS2已发布 -> {ROS2_TOPIC_NAME} | {best['command']}")
            self.last_publish_time = now
            executed = True
        elif best['command']:
            print('[INFO] 命令识别成功，但处于 cooldown，未重复发布')
        else:
            print('[INFO] 本段未识别到有效命令')

        log_item = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'channels': CHANNELS,
            'frame_rate': SAMPLE_RATE,
            'candidates': candidates_info,
            'final_channel': best['channel'],
            'final_text': best['text'],
            'final_command': best['command'],
            'executed': executed,
            'ros2_topic': ROS2_TOPIC_NAME,
        }
        append_log(log_item)

        print('=' * 60)
        self.reset_segment_buffers()

    def run(self):
        try:
            self.start_arecord()
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.get_logger().error('请先执行: arecord -l，把输出发我')
            return

        try:
            while rclpy.ok() and self.running:
                raw_chunk = self.read_exact(RAW_CHUNK_BYTES)

                if len(raw_chunk) == 0:
                    self.get_logger().error('音频流结束，程序停止。')
                    break

                if len(raw_chunk) < RAW_CHUNK_BYTES:
                    self.get_logger().warning(
                        f'收到短音频块: {len(raw_chunk)} bytes，继续等待下一块'
                    )
                    continue

                left_chunk = audioop.tomono(raw_chunk, SAMPLE_WIDTH, 1, 0)
                right_chunk = audioop.tomono(raw_chunk, SAMPLE_WIDTH, 0, 1)
                mix_chunk = audioop.tomono(raw_chunk, SAMPLE_WIDTH, 0.5, 0.5)

                self.segment_buffers['left'] += left_chunk
                self.segment_buffers['right'] += right_chunk
                self.segment_buffers['mix'] += mix_chunk

                flags = {
                    'left': self.recs['left'].AcceptWaveform(left_chunk),
                    'right': self.recs['right'].AcceptWaveform(right_chunk),
                    'mix': self.recs['mix'].AcceptWaveform(mix_chunk),
                }

                if flags['left'] or flags['right'] or flags['mix']:
                    self.handle_segment_end(flags)

                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.stop_arecord()


def main():
    rclpy.init()
    node = VoiceCommandLiveNode()

    def _handle_sigint(signum, frame):
        node.running = False

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
