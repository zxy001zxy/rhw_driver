from __future__ import annotations

import os
import signal
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import re
import subprocess
from threading import Lock
from typing import Optional

import rclpy
from rclpy.node import Node

from rhw_msgs.msg import ModeState
from rhw_msgs.srv import GetMapList, LoadMap, SaveMap, SwitchMode

# ---------------------------------------------------------------------------
# 日志目录：子进程 stdout/stderr 重定向到此处，方便排查
# ---------------------------------------------------------------------------
_LOG_DIR = Path('/tmp/rhw_map_manager_logs')


def _default_lightning_home() -> Path:
    configured = os.environ.get('LIGHTNING_LM_HOME')
    if configured:
        return Path(os.path.expanduser(os.path.expandvars(configured)))
    return Path.home() / 'lightning-lm'


@dataclass
class ManagedProcess:
    mode: int
    command: str
    process: subprocess.Popen
    active_launch: str
    log_path: Path = field(default_factory=lambda: Path('/dev/null'))


class MapManagerNode(Node):
    # 模式枚举 —— 增加 IDLE 表示真正的空闲态
    MODE_IDLE = 255
    MODE_MAPPING = 0
    MODE_NAVIGATION = 1

    # SaveMap 默认超时（秒）
    _SAVE_MAP_TIMEOUT = 60

    def __init__(self) -> None:
        super().__init__('map_manager_node')
        default_lightning_home = _default_lightning_home()

        # ---- 内部状态 ----
        self._lock = Lock()
        self._switching = False
        self._managed_process: Optional[ManagedProcess] = None
        self._loaded_map_path: Optional[Path] = None

        # ---- 话题 ----
        self._mode_state_pub = self.create_publisher(ModeState, '/mode_state', 10)

        # ---- 参数声明 ----
        self.declare_parameter('mapping_workspace', str(default_lightning_home))
        self.declare_parameter('mapping_command',
                               'ros2 run lightning run_slam_online --config ./config/default.yaml')
        self.declare_parameter('navigation_workspace', str(default_lightning_home))
        self.declare_parameter('navigation_command_template',
                               'ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:="{map_path}"')
        self.declare_parameter('navigation_extra_commands', [])
        self.declare_parameter('save_map_workspace', str(default_lightning_home))

        self.declare_parameter('save_map_command_template',
                               "ros2 service call /lightning/save_map lightning/srv/SaveMap "
                               "\"{{map_id: '{map_name}'}}\"")
        self.declare_parameter('map_dir', str(default_lightning_home / 'data'))
        self.declare_parameter('map_yaml_name', 'map.yaml')
        self.declare_parameter('mode_state_topic_period', 1.0)
        self.declare_parameter('save_map_timeout', float(self._SAVE_MAP_TIMEOUT))

        # ---- 读取参数 ----
        self._mapping_workspace = self._resolve_path_param(self.get_parameter('mapping_workspace').value)
        self._mapping_command = str(self.get_parameter('mapping_command').value)
        self._navigation_workspace = self._resolve_path_param(self.get_parameter('navigation_workspace').value)
        self._navigation_command_template = str(self.get_parameter('navigation_command_template').value)
        self._navigation_extra_commands = [
            str(command) for command in self.get_parameter('navigation_extra_commands').value
        ]
        self._save_map_workspace = self._resolve_path_param(self.get_parameter('save_map_workspace').value)
        self._save_map_command_template = str(self.get_parameter('save_map_command_template').value)
        self._map_dir = self._resolve_path_param(self.get_parameter('map_dir').value)
        self._map_yaml_name = str(self.get_parameter('map_yaml_name').value)
        self._save_map_timeout = float(self.get_parameter('save_map_timeout').value)

        # ---- 定时发布模式状态 ----
        period = float(self.get_parameter('mode_state_topic_period').value)
        self._mode_timer = self.create_timer(period, self._publish_mode_state)

        # ---- 注册服务 ----
        self.create_service(SwitchMode, '/mode_manager/switch_mode', self._handle_switch_mode)
        self.create_service(SaveMap, '/mode_manager/save_map', self._handle_save_map)
        self.create_service(GetMapList, '/map_manager/get_map_list', self._handle_get_map_list)
        self.create_service(LoadMap, '/map_manager/load_map', self._handle_load_map)

        # ---- 准备日志目录 ----
        self._prepare_dir(_LOG_DIR)

        # ---- 启动信息 ----
        self._log_map_dir_status()
        self.get_logger().info(
            f'mapping_workspace={self._mapping_workspace} '
            f'navigation_workspace={self._navigation_workspace} '
            f'save_map_workspace={self._save_map_workspace} '
            f'map_dir={self._map_dir} log_dir={_LOG_DIR}'
        )
        self._publish_mode_state()

    # ===================================================================
    #  SwitchMode 服务
    # ===================================================================
    def _handle_switch_mode(self, request: SwitchMode.Request,
                            response: SwitchMode.Response) -> SwitchMode.Response:
        with self._lock:
            if self._switching:
                response.result = 2
                response.current_mode = self._current_mode()
                response.active_launch = self._current_active_launch()
                response.message = 'mode switching in progress'
                return response
            if request.target_mode not in (self.MODE_MAPPING, self.MODE_NAVIGATION):
                response.result = 3
                response.current_mode = self._current_mode()
                response.active_launch = self._current_active_launch()
                response.message = f'invalid target_mode: {request.target_mode}'
                return response
            self._switching = True

        try:
            current_mode = self._current_mode()
            if (current_mode == request.target_mode
                    and self._process_healthy()
                    and not request.force_restart):
                response.result = 1
                response.current_mode = current_mode
                response.active_launch = self._current_active_launch()
                response.message = 'target mode already active'
                return response

            # 先停掉旧进程
            self._stop_managed_process()

            if request.target_mode == self.MODE_MAPPING:
                command = self._build_mapping_command(request.launch_args)
                active_launch = request.launch_profile or 'mapping'
                process, log_path = self._spawn_process(command, self._mapping_workspace,
                                                        tag='mapping')
            else:
                map_path = self._resolve_navigation_map_path(request.launch_args)
                if map_path is None:
                    response.result = 0
                    response.current_mode = self._current_mode()
                    response.active_launch = self._current_active_launch()
                    response.message = 'no loaded map available for navigation; call LoadMap first'
                    return response
                command = self._build_navigation_command(map_path, request.launch_args)
                active_launch = request.launch_profile or f'navigation:{map_path.parent.name}'
                process, log_path = self._spawn_process(command, self._navigation_workspace,
                                                        tag='navigation')

            self._managed_process = ManagedProcess(
                mode=request.target_mode,
                command=command,
                process=process,
                active_launch=active_launch,
                log_path=log_path,
            )
            response.result = 1
            response.current_mode = request.target_mode
            response.active_launch = active_launch
            response.message = f'started {active_launch}'
            return response

        except Exception as exc:
            self.get_logger().error(f'switch mode failed: {exc}')
            response.result = 0
            response.current_mode = self._current_mode()
            response.active_launch = self._current_active_launch()
            response.message = str(exc)
            return response
        finally:
            with self._lock:
                self._switching = False
            self._publish_mode_state()

    # ===================================================================
    #  SaveMap 服务
    # ===================================================================
    def _handle_save_map(self, request: SaveMap.Request,
                         response: SaveMap.Response) -> SaveMap.Response:
        map_name = request.map_name.strip()
        if not self._is_valid_map_name(map_name):
            response.result = 0
            response.message = 'invalid map_name'
            return response

        # P2: 前置检查 —— 建议在建图模式下保存
        if not self._process_healthy() or self._current_mode() != self.MODE_MAPPING:
            self.get_logger().warning('save_map called but mapping process is not running')

        try:
            command = self._save_map_command_template.format(map_name=map_name)
        except KeyError as exc:
            response.result = 0
            response.message = f'command template format error: {exc}'
            self.get_logger().error(f'save_map_command_template format error: {exc}')
            return response

        self.get_logger().info(f'save map command: {command}')

        try:
            if not self._save_map_workspace.exists():
                response.result = 0
                response.message = (
                    f'save_map_workspace not found: {self._save_map_workspace}; '
                    'please set parameter save_map_workspace or LIGHTNING_LM_HOME'
                )
                return response

            completed = subprocess.run(
                ['/bin/bash', '-lc', command],
                cwd=str(self._save_map_workspace),
                check=False,
                capture_output=True,
                text=True,
                timeout=self._save_map_timeout,
            )
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout or 'unknown error').strip()
                response.result = 0
                response.message = f'save map failed (exit {completed.returncode}): {detail}'
                return response

            yaml_path = self._map_yaml_path(map_name)
            if not yaml_path.exists():
                response.result = 0
                response.message = f'save service returned success but yaml missing: {yaml_path}'
                return response

            self._loaded_map_path = yaml_path
            response.result = 1
            response.message = f'map saved: {yaml_path}'
            return response

        except subprocess.TimeoutExpired:
            response.result = 0
            response.message = f'save map timed out after {self._save_map_timeout}s'
            return response
        except Exception as exc:
            response.result = 0
            response.message = str(exc)
            return response

    # ===================================================================
    #  GetMapList 服务
    # ===================================================================
    def _handle_get_map_list(self, request: GetMapList.Request,
                             response: GetMapList.Response) -> GetMapList.Response:
        del request
        try:
            if not self._map_dir.exists():
                response.result = 1
                response.map_names = []
                response.map_timestamps = []
                response.message = f'map dir not ready: {self._map_dir}'
                return response

            yaml_files = sorted(
                self._iter_map_yaml_files(),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            response.result = 1
            response.map_names = [f.parent.name for f in yaml_files]
            response.map_timestamps = [
                datetime.fromtimestamp(f.stat().st_mtime).strftime('%Y-%m-%d %H:%M:%S')
                for f in yaml_files
            ]
            response.message = f'found {len(yaml_files)} maps'
            return response

        except Exception as exc:
            response.result = 0
            response.map_names = []
            response.map_timestamps = []
            response.message = str(exc)
            return response

    # ===================================================================
    #  LoadMap 服务
    # ===================================================================
    def _handle_load_map(self, request: LoadMap.Request,
                         response: LoadMap.Response) -> LoadMap.Response:
        map_name = request.map_name.strip()
        if not self._is_valid_map_name(map_name):
            response.result = 0
            response.message = 'invalid map_name'
            return response

        if not self._map_dir.exists():
            response.result = 2
            response.message = f'map dir not ready: {self._map_dir}'
            return response

        yaml_path = self._map_yaml_path(map_name)
        if not yaml_path.exists():
            response.result = 2
            response.message = f'map not found: {yaml_path}'
            return response

        self._loaded_map_path = yaml_path
        response.result = 1
        response.message = f'loaded map selected: {yaml_path}'
        return response

    # ===================================================================
    #  命令构建
    # ===================================================================
    def _build_mapping_command(self, launch_args: list[str]) -> str:
        return self._append_launch_args(self._mapping_command, launch_args)

    def _build_navigation_command(self, map_path: Path, launch_args: list[str]) -> str:
        commands = [
            self._append_launch_args(
                self._format_command_template(self._navigation_command_template, map_path),
                launch_args,
            )
        ]
        commands.extend(
            self._format_command_template(command, map_path)
            for command in self._navigation_extra_commands
            if str(command).strip()
        )
        if len(commands) == 1:
            return commands[0]
        background_commands = '\n'.join(f'{command} &' for command in commands)
        return f'{background_commands}\nwait'

    @staticmethod
    def _append_launch_args(command: str, launch_args: list[str]) -> str:
        extras = ' '.join(arg for arg in launch_args if arg)
        return f'{command} {extras}'.strip()

    def _resolve_navigation_map_path(self, launch_args: list[str]) -> Optional[Path]:
        for arg in launch_args:
            if arg.startswith('map:='):
                return Path(arg.split(':=', 1)[1]).expanduser()
        return self._loaded_map_path

    def _format_command_template(self, command: str, map_path: Path) -> str:
        return command.format(
            map_path=str(map_path),
            map_name=map_path.parent.name,
            map_dir=str(map_path.parent),
            navigation_workspace=str(self._navigation_workspace),
        )

    # ===================================================================
    #  地图路径工具
    # ===================================================================
    def _map_yaml_path(self, map_name: str) -> Path:
        """lightning 保存格式: <map_dir>/<map_name>/<map_yaml_name>"""
        return self._map_dir / map_name / self._map_yaml_name

    def _iter_map_yaml_files(self) -> list[Path]:
        return [p for p in self._map_dir.glob(f'*/{self._map_yaml_name}') if p.is_file()]

    # ===================================================================
    #  子进程管理 —— 使用 process group 杀整棵进程树
    # ===================================================================
    def _spawn_process(self, command: str, cwd: Path,
                       tag: str = 'proc') -> tuple[subprocess.Popen, Path]:
        """启动子进程，stdout/stderr 重定向到日志文件，使用新 session 便于整组杀。"""
        if not cwd.exists():
            raise FileNotFoundError(f'workspace not found: {cwd}')

        log_path = _LOG_DIR / f'{tag}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        self.get_logger().info(f'spawn [{tag}]: {command}')
        self.get_logger().info(f'  cwd={cwd}  log={log_path}')

        log_file = open(log_path, 'w')  # noqa: SIM115
        process = subprocess.Popen(
            ['/bin/bash', '-lc', command],
            cwd=str(cwd),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,          # 关键：创建新 session/进程组
        )
        return process, log_path

    def _stop_managed_process(self) -> None:
        """停止当前托管进程，杀整棵进程树。"""
        if self._managed_process is None:
            return

        process = self._managed_process.process
        pgid = None
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            pass  # 进程已退出

        if process.poll() is None:
            self.get_logger().info(
                f'stopping managed process pid={process.pid} pgid={pgid}')
            # 先温和终止整个进程组
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except OSError:
                    pass
            else:
                process.terminate()

            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                # 强杀
                self.get_logger().warning('SIGTERM timed out, sending SIGKILL')
                if pgid is not None:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except OSError:
                        pass
                else:
                    process.kill()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().error('process still alive after SIGKILL')

        self._managed_process = None

    # ===================================================================
    #  状态查询
    # ===================================================================
    def _current_mode(self) -> int:
        """空闲时返回 MODE_IDLE(255)，而非 MODE_MAPPING，避免误导 APP。"""
        if self._managed_process and self._managed_process.process.poll() is None:
            return self._managed_process.mode
        return self.MODE_IDLE

    def _current_active_launch(self) -> str:
        if self._managed_process and self._managed_process.process.poll() is None:
            return self._managed_process.active_launch
        return ''

    def _process_healthy(self) -> bool:
        return (self._managed_process is not None
                and self._managed_process.process.poll() is None)

    # ===================================================================
    #  ModeState 话题发布
    # ===================================================================
    def _publish_mode_state(self) -> None:
        msg = ModeState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self._current_mode()
        msg.switching = self._switching
        msg.healthy = self._process_healthy()
        msg.active_launch = self._current_active_launch()

        if self._process_healthy():
            msg.message = 'running'
        elif self._loaded_map_path is not None:
            msg.message = f'idle, selected map: {self._loaded_map_path.parent.name}'
        else:
            msg.message = 'idle'
        self._mode_state_pub.publish(msg)

    # ===================================================================
    #  工具方法
    # ===================================================================
    def _log_map_dir_status(self) -> None:
        try:
            if self._map_dir.exists():
                return
            self.get_logger().warning(
                f'map_dir does not exist yet: {self._map_dir}; '
                'please set parameter map_dir or LIGHTNING_LM_HOME'
            )
        except Exception as exc:
            self.get_logger().warning(f'cannot access map_dir {self._map_dir}: {exc}')

    @staticmethod
    def _prepare_dir(path: Path) -> None:
        try:
            path.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass

    def destroy_node(self) -> bool:
        self._stop_managed_process()
        return super().destroy_node()

    @staticmethod
    def _is_valid_map_name(map_name: str) -> bool:
        return bool(map_name) and re.fullmatch(r'[A-Za-z0-9_-]+', map_name) is not None

    @staticmethod
    def _resolve_path_param(value: str) -> Path:
        return Path(os.path.expanduser(os.path.expandvars(str(value))))


def main() -> None:
    rclpy.init()
    node = MapManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
