from __future__ import annotations

import audioop
import importlib
import json
import os
import re
import sys
import wave
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
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


def get_package_base_dir() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(PACKAGE_NAME))
        except Exception:
            pass
    return Path(__file__).resolve().parents[1]


PACKAGE_BASE_DIR = get_package_base_dir()
MODEL_PATH = PACKAGE_BASE_DIR / 'model' / 'vosk-model-small-cn-0.22'
DEFAULT_WAV = PACKAGE_BASE_DIR / 'test_audio' / 'forward.wav'
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


def recognize_pcm_bytes(model, pcm_bytes: bytes, sample_rate: int) -> str:
    recognizer_cls, _ = ensure_vosk_imported()
    rec = recognizer_cls(model, sample_rate, json.dumps(GRAMMAR, ensure_ascii=False))
    rec.SetWords(True)

    final_texts: list[str] = []
    step = 4000 * 2

    for i in range(0, len(pcm_bytes), step):
        chunk = pcm_bytes[i:i + step]
        if rec.AcceptWaveform(chunk):
            result = json.loads(rec.Result())
            text = result.get('text', '').strip()
            if text:
                final_texts.append(text)

    final_result = json.loads(rec.FinalResult())
    final_text = final_result.get('text', '').strip()
    if final_text:
        final_texts.append(final_text)

    return normalize_text(' '.join(final_texts))


def pcm_rms(pcm_bytes: bytes, sample_width: int) -> int:
    return audioop.rms(pcm_bytes, sample_width)


def build_channel_candidates(raw: bytes, sample_width: int, channels: int):
    if channels == 1:
        return [('mono', raw)]

    if channels == 2:
        left_pcm = audioop.tomono(raw, sample_width, 1, 0)
        right_pcm = audioop.tomono(raw, sample_width, 0, 1)
        mix_pcm = audioop.tomono(raw, sample_width, 0.5, 0.5)
        return [('left', left_pcm), ('right', right_pcm), ('mix', mix_pcm)]

    raise ValueError(f'暂不支持 {channels} 声道')


def choose_best_candidate(candidates_info: list[dict]):
    matched = [item for item in candidates_info if item['command'] is not None]
    if matched:
        return sorted(matched, key=lambda item: item['rms'], reverse=True)[0]

    with_text = [item for item in candidates_info if item['text']]
    if with_text:
        return sorted(with_text, key=lambda item: item['rms'], reverse=True)[0]

    return sorted(candidates_info, key=lambda item: item['rms'], reverse=True)[0]


class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher_ = self.create_publisher(String, ROS2_TOPIC_NAME, 10)
        self.get_logger().info(f'ROS2 publisher ready, topic = {ROS2_TOPIC_NAME}')

    def publish_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published ROS2 topic: {cmd}')


def publish_ros2_command(cmd: str):
    if not cmd:
        print('[EXEC] 未发布 ROS2 话题，命令为空')
        return False

    node = None
    try:
        rclpy.init()
        node = VoiceCommandPublisher()
        node.publish_command(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        print(f'[EXEC] ROS2已发布 -> {ROS2_TOPIC_NAME} | {cmd}')
        return True
    except Exception as exc:
        print(f'[EXEC] ROS2发布失败: {exc}')
        return False
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def append_log(log_item: dict):
    os.makedirs(LOG_PATH.parent, exist_ok=True)
    with open(LOG_PATH, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(log_item, ensure_ascii=False) + '\n')


def main():
    wav_path = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else DEFAULT_WAV

    if not MODEL_PATH.is_dir():
        print(f'[ERROR] 模型目录不存在: {MODEL_PATH}')
        sys.exit(1)

    if not wav_path.is_file():
        print(f'[ERROR] 音频文件不存在: {wav_path}')
        sys.exit(1)

    with wave.open(str(wav_path), 'rb') as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        frame_rate = wf.getframerate()
        comp_type = wf.getcomptype()
        raw = wf.readframes(wf.getnframes())

    if sample_width != 2 or comp_type != 'NONE':
        print('[ERROR] 只支持 16-bit PCM WAV')
        sys.exit(1)

    if frame_rate != 16000:
        print(f'[ERROR] 只支持 16k 采样率，当前 framerate={frame_rate}')
        sys.exit(1)

    _, model_cls = ensure_vosk_imported()
    model = model_cls(str(MODEL_PATH))
    channel_candidates = build_channel_candidates(raw, sample_width, channels)
    candidates_info: list[dict] = []

    print('=' * 60)
    print('音频文件 :', wav_path)
    print('声道数   :', channels)
    print('采样率   :', frame_rate)
    print('=' * 60)

    for name, pcm in channel_candidates:
        text = recognize_pcm_bytes(model, pcm, frame_rate)
        cmd = match_command(text)
        rms = pcm_rms(pcm, sample_width)
        info = {'channel': name, 'text': text, 'command': cmd, 'rms': rms}
        candidates_info.append(info)

        print(f'[{name}] RMS      : {rms}')
        print(f'[{name}] 识别文本 : {text if text else "[空]"}')
        print(f'[{name}] 命令码   : {cmd if cmd else "[未匹配]"}')
        print('-' * 60)

    best = choose_best_candidate(candidates_info)

    print(f"[最终采用] 通道   : {best['channel']}")
    print(f"[最终采用] RMS    : {best['rms']}")
    print(f"[最终采用] 文本   : {best['text'] if best['text'] else '[空]'}")
    print(f"[最终采用] 命令   : {best['command'] if best['command'] else '[未匹配]'}")

    executed = publish_ros2_command(best['command']) if best['command'] else False
    if not best['command']:
        print('[EXEC] 未发布，未识别到有效命令')

    log_item = {
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'wav_path': str(wav_path),
        'channels': channels,
        'frame_rate': frame_rate,
        'candidates': candidates_info,
        'final_channel': best['channel'],
        'final_text': best['text'],
        'final_command': best['command'],
        'executed': executed,
        'ros2_topic': ROS2_TOPIC_NAME,
    }
    append_log(log_item)

    print('=' * 60)
    print(f'日志已写入: {LOG_PATH}')
    print('=' * 60)


if __name__ == '__main__':
    main()
