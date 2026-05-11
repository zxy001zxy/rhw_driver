"""ROS 2 节点: RTSP 摄像头 → sensor_msgs/Image 话题发布（低延迟）。"""
from __future__ import annotations

import os
import site
import sys
import threading
import time


def _prefer_ros_system_site_packages() -> None:
    user_site = site.getusersitepackages()
    if user_site in sys.path:
        sys.path.remove(user_site)


_prefer_ros_system_site_packages()

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    """RTSP → ROS 2 图像发布节点。

    架构
    ----
    * 后台 **read 线程** 持续调用 ``read()``（grab + decode），以码流原生帧率
      运行，每次成功后覆盖写入 ``_latest_frame`` 槽位，保证其始终是最新帧。
    * ROS **定时器** 按设定帧率从槽位取帧并发布，不做任何网络 I/O，不会阻塞。

    效果：
    * FFmpeg 内部缓冲被持续排空 → 延迟低
    * 定时器不受网络抖动影响 → 发布帧率稳定
    * 每帧只解码一次（在后台线程） → 无重复计算
    """

    def __init__(self) -> None:
        super().__init__('camera_publisher_node')

        # ---- 参数声明 ----
        self.declare_parameter('camera_ip', '192.168.10.64')
        self.declare_parameter('rtsp_port', 554)
        self.declare_parameter('rtsp_username', 'admin')
        self.declare_parameter('rtsp_password', 'rhw1314000')
        self.declare_parameter('rtsp_path', '/Streaming/Channels/101')
        self.declare_parameter('rtsp_url_override', '')        # 非空则直接使用此 URL
        self.declare_parameter('frame_rate', 30.0)             # 发布帧率 Hz
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('reconnect_interval', 3.0)      # 断线重连间隔(秒)
        self.declare_parameter('read_failure_timeout', 2.0)    # 连续读帧失败多久后重连(秒)
        self.declare_parameter('rtsp_transport', 'tcp')        # tcp 稳定 / udp 低延迟
        self.declare_parameter('ffmpeg_low_latency', False)    # True 启用 nobuffer/low_delay
        self.declare_parameter('publish_compressed', True)     # 同时发布 CompressedImage
        self.declare_parameter('jpeg_quality', 70)             # JPEG 压缩质量 1-100
        self.declare_parameter('output_width', 1920)           # 输出宽度，0 表示保持原尺寸
        self.declare_parameter('output_height', 1080)          # 输出高度，0 表示保持原尺寸
        self.declare_parameter('qos_reliability', 'best_effort')  # reliable / best_effort
        self.declare_parameter('qos_depth', 1)                    # 发布队列深度

        # ---- 构建 RTSP URL ----
        override = str(self.get_parameter('rtsp_url_override').value or '')
        if override.strip():
            self._rtsp_url = override.strip()
        else:
            ip = self.get_parameter('camera_ip').value
            port = self.get_parameter('rtsp_port').value
            user = self.get_parameter('rtsp_username').value
            pwd = self.get_parameter('rtsp_password').value
            path = self.get_parameter('rtsp_path').value
            self._rtsp_url = f'rtsp://{user}:{pwd}@{ip}:{port}{path}'

        self._frame_id: str = str(self.get_parameter('frame_id').value)
        self._reconnect_interval: float = max(
            float(self.get_parameter('reconnect_interval').value), 0.1
        )
        self._read_failure_timeout: float = max(
            float(self.get_parameter('read_failure_timeout').value), 0.1
        )
        self._rtsp_transport: str = str(self.get_parameter('rtsp_transport').value).strip().lower()
        if self._rtsp_transport not in ('tcp', 'udp'):
            self.get_logger().warn(
                f'rtsp_transport={self._rtsp_transport!r} 无效，已改用 tcp'
            )
            self._rtsp_transport = 'tcp'
        self._ffmpeg_low_latency: bool = bool(self.get_parameter('ffmpeg_low_latency').value)
        self._publish_compressed: bool = bool(self.get_parameter('publish_compressed').value)
        self._jpeg_quality: int = int(self.get_parameter('jpeg_quality').value)
        self._output_width: int = max(int(self.get_parameter('output_width').value), 0)
        self._output_height: int = max(int(self.get_parameter('output_height').value), 0)

        # ---- QoS 可配置：reliable 兼容性好 / best_effort 延迟低 ----
        qos_rel_str = str(self.get_parameter('qos_reliability').value).strip().lower()
        qos_depth = max(int(self.get_parameter('qos_depth').value), 1)
        if qos_rel_str == 'reliable':
            qos_rel = QoSReliabilityPolicy.RELIABLE
        else:
            qos_rel = QoSReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(
            reliability=qos_rel,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
        )
        self.get_logger().info(f'QoS: reliability={qos_rel_str}  depth={qos_depth}')
        topic = str(self.get_parameter('image_topic').value)
        self._pub = self.create_publisher(Image, topic, qos)
        self._bridge = CvBridge()

        # 可选：同时发布 JPEG 压缩图像
        if self._publish_compressed:
            self._pub_compressed = self.create_publisher(
                CompressedImage, topic + '/compressed', qos
            )
            self._jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        else:
            self._pub_compressed = None
            self._jpeg_params = None

        # ---- 共享状态 ----
        self._cap: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._latest_frame = None       # 最新帧 (numpy ndarray)，由 read 线程写入
        self._frame_seq: int = 0        # 帧序号，read 成功 +1
        self._published_seq: int = 0    # 上次已发布的帧序号
        self._need_reconnect = False
        self._next_reconnect_time = 0.0

        self._stop_event = threading.Event()
        self._read_thread: threading.Thread | None = None

        self._open_camera()

        # ---- 发布定时器 ----
        fps = float(self.get_parameter('frame_rate').value)
        period = 1.0 / max(fps, 1.0)
        self._timer = self.create_timer(period, self._publish_latest)

        self.get_logger().info(
            f'camera_publisher_node 已启动  话题={topic}  帧率={fps:.1f}Hz  '
            f'传输={self._rtsp_transport}  低延迟={self._ffmpeg_low_latency}  '
            f'rtsp={self._mask_url(self._rtsp_url)}'
        )

    # ==================================================================
    #  摄像头 打开 / 关闭
    # ==================================================================
    def _open_camera(self, reason: str = '', *, force: bool = False) -> bool:
        """打开（或重新打开）RTSP 流，成功返回 True。"""
        now = time.monotonic()
        if not force and now < self._next_reconnect_time:
            return False

        self._next_reconnect_time = now + self._reconnect_interval
        if reason:
            self.get_logger().warn(reason)

        self._stop_read_thread()
        self._release_camera()

        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = self._build_ffmpeg_options()

        cap = cv2.VideoCapture(self._rtsp_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 10000)
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 10000)

        if cap.isOpened():
            self._cap = cap
            self._need_reconnect = False
            self._next_reconnect_time = 0.0
            self._start_read_thread()
            self.get_logger().info('摄像头连接成功')
            return True

        cap.release()
        self.get_logger().warn('摄像头连接失败，稍后重试...')
        return False

    def _release_camera(self) -> None:
        """释放 VideoCapture 资源。"""
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    # ==================================================================
    #  后台 read 线程 — 持续 read()，覆盖写 _latest_frame
    # ==================================================================
    def _start_read_thread(self) -> None:
        self._stop_event.clear()
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name='rtsp-read'
        )
        self._read_thread.start()

    def _stop_read_thread(self) -> None:
        self._stop_event.set()
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=3.0)
        self._read_thread = None

    def _read_loop(self) -> None:
        """以码流原生帧率持续 read()。

        read() = grab() + retrieve()，按摄像头实际帧率运行。
        每次成功后覆盖 _latest_frame，确保发布定时器取到的
        始终是最新已解码帧。
        """
        failure_started_at: float | None = None
        last_failure_log_at = 0.0
        while not self._stop_event.is_set():
            cap = self._cap
            if cap is None:
                self._stop_event.wait(0.5)
                continue

            ret, frame = cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._latest_frame = frame
                    self._frame_seq += 1
                failure_started_at = None
            else:
                now = time.monotonic()
                if failure_started_at is None:
                    failure_started_at = now
                failure_duration = now - failure_started_at

                if failure_duration >= self._read_failure_timeout:
                    self._need_reconnect = True
                    break

                if now - last_failure_log_at >= 5.0:
                    self.get_logger().warn(
                        f'读帧失败，等待码流恢复 '
                        f'({failure_duration:.1f}/{self._read_failure_timeout:.1f}s)'
                    )
                    last_failure_log_at = now

                # 短暂等待避免 CPU 空转
                self._stop_event.wait(0.01)

    # ==================================================================
    #  定时器回调 — 发布最新帧（无阻塞）
    # ==================================================================
    def _publish_latest(self) -> None:
        # 检查是否需要重连
        if self._need_reconnect or \
           (self._read_thread is not None and not self._read_thread.is_alive()
            and not self._stop_event.is_set()):
            self._open_camera('read 线程退出，正在重连...')
            return

        if self._cap is None:
            self._open_camera('摄像头未连接，正在连接...')
            return

        # 非阻塞取最新帧
        with self._lock:
            if self._frame_seq == self._published_seq:
                return  # 没有新帧，跳过本轮
            frame = self._latest_frame
            self._published_seq = self._frame_seq

        if frame is None:
            return

        if self._output_width > 0 and self._output_height > 0:
            current_height, current_width = frame.shape[:2]
            if current_width != self._output_width or current_height != self._output_height:
                frame = cv2.resize(
                    frame,
                    (self._output_width, self._output_height),
                    interpolation=cv2.INTER_LINEAR,
                )

        stamp = self.get_clock().now().to_msg()

        # ---- 发布原始 Image ----
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)

        # ---- 发布压缩 CompressedImage (JPEG) ----
        if self._pub_compressed is not None:
            ok, buf = cv2.imencode('.jpg', frame, self._jpeg_params)
            if ok:
                cmsg = CompressedImage()
                cmsg.header.stamp = stamp
                cmsg.header.frame_id = self._frame_id
                cmsg.format = 'jpeg'
                cmsg.data = buf.tobytes()
                self._pub_compressed.publish(cmsg)

    # ==================================================================
    #  工具方法
    # ==================================================================
    @staticmethod
    def _mask_url(url: str) -> str:
        """日志中隐藏密码：rtsp://admin:***@..."""
        try:
            if '://' in url and '@' in url:
                scheme_rest = url.split('://', 1)
                userinfo_host = scheme_rest[1].split('@', 1)
                user_pass = userinfo_host[0].split(':', 1)
                masked_pass = '***' if len(user_pass) > 1 else ''
                return f'{scheme_rest[0]}://{user_pass[0]}:{masked_pass}@{userinfo_host[1]}'
        except Exception:
            pass
        return url

    def _build_ffmpeg_options(self) -> str:
        options = [f'rtsp_transport;{self._rtsp_transport}']
        if self._ffmpeg_low_latency:
            options.extend([
                'fflags;nobuffer',
                'flags;low_delay',
                'max_delay;500000',
            ])
        else:
            options.append('max_delay;2000000')
        return '|'.join(options)

    def destroy_node(self) -> None:
        self._stop_read_thread()
        self._release_camera()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
