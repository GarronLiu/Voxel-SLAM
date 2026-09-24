#!/usr/bin/env python3
"""Local-only operator console for the real-boat virtual-obstacle workflow."""

from __future__ import annotations

import argparse
import concurrent.futures
import fcntl
import json
import math
import os
import pty
import re
import select
import shlex
import signal
import stat
import struct
import subprocess
import termios
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import ip_address
from pathlib import Path
from typing import Dict, List, Mapping
from urllib.parse import urlparse


REPO_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = Path(__file__).resolve().parent
STATE_ROOT = REPO_ROOT / ".real_boat_operator"
LOG_ROOT = STATE_ROOT / "logs"
CONFIG_PATH = STATE_ROOT / "config.json"
VIZ_SNAPSHOT_PATH = STATE_ROOT / "visualization.json"
ROS_SETUP = "/opt/ros/noetic/setup.bash"
WORKSPACE_SETUP = str(REPO_ROOT.parent / "devel" / "setup.bash")
TOPIC_PATTERN = re.compile(r"^/[A-Za-z0-9_/]+$")
FRAME_PATTERN = re.compile(r"^[A-Za-z0-9_/]+$")

DEFAULT_BAG_TOPICS = [
    "/livox/lidar", "/mavros/local_position/odom", "/mavros/rc/in",
    "/mavros/rc/override", "/move_base_simple/goal", "/tf", "/tf_static",
    "/usv/field_state", "/usv/virtual_target", "/usv/pose", "/usv/path",
    "/usv/perception_status", "/usv/virtual_obstacle_status", "/cluster_info",
    "/usv/lidar_scan_policy", "/usv/raw_obs", "/usv/ppo_cmd",
    "/usv/apf_pid_cmd", "/usv/mptc_cmd", "/usv/model_cmd",
    "/usv/selected_controller", "/usv/controller_mux_status",
]


DEFAULT_CONFIG = {
    "pointcloud_topic": "/livox/lidar",
    "pointcloud_frame": "map",
    "sensor_yaw_offset_deg": 0.0,
    "odom_topic": "/mavros/local_position/odom",
    "rc_in_topic": "/mavros/rc/in",
    "rc_unlock_channel": 8,
    "rc_unlock_channel_backup": 5,
    "rc_unlock_threshold": 1800,
    "manual_obstacle_radius": 1.5,
    "merge_physical_scan": True,
    "use_scenario_yaml": True,
    "scenario_yaml": "usv_obstacle_avoidance/config/scenario_demo.yaml",
    "model_path": "runs/tiv_r3_ablation_0614/continuation/models/usv_obstacle_ckpt/ckpt_10000032.zip",
    "vecnorm_path": "runs/tiv_r3_ablation_0614/continuation/models/usv_obstacle_ckpt/vec_normalize_10000032.pkl",
    "model_search_root": "runs",
    "recent_models": [],
    "virtual_target_mode": 1,
    "virtual_target_wait_dist_start": 5.0,
    "virtual_target_wait_dist_stop": 10.0,
    "virtual_target_speed": 1.0,
    "max_abs_action": 0.1,
    "dry_run": True,
    "no_actuation": True,
    "actuation_ack": "",
    "fcu_url": "/dev/X7:230400",
    "gcs_url": "udp://:14550@10.168.1.202",
    "ublox_serial_port": "/dev/ublox",
    "ublox_rtcm_tcp_host": "192.168.1.100",
    "ublox_workspace": "~/equipment_ros_driver/ublox_ws",
    "ublox_launch": "ublox_driver.launch",
    "camera_workspace": "~/equipment_ros_driver/mvs_ws",
    "camera_launch": "mvs_camera_trigger.launch",
    "mavlink_rates": {"31": 100.0, "32": 100.0, "36": 20.0},
    "livox_workspace": "~/equipment_ros_driver/livox2_ws",
    "livox_launch": "msg_MID360.launch",
    "livox_xfer_format": 2,
    "bag_directory": "../rosbag",
    "bag_session": "virtual_obstacle_trial",
    "bag_tags": "虚拟障碍, neutral, Scratch/R3",
    "bag_record_all": True,
    "bag_topics": DEFAULT_BAG_TOPICS,
    "open_process_terminals": True,
}


def _shell_join(parts: List[str]) -> str:
    return " ".join(shlex.quote(str(part)) for part in parts)


def _ros_shell(parts: List[str], extra_setup: str = "") -> List[str]:
    setup = [f"source {shlex.quote(ROS_SETUP)}", f"source {shlex.quote(WORKSPACE_SETUP)}"]
    if extra_setup:
        setup.append(f"source {shlex.quote(extra_setup)}")
    return ["bash", "-lc", " && ".join(setup + ["exec " + _shell_join(parts)])]


def _ros_shell_script(script: str, extra_setup: str = "") -> List[str]:
    setup = [f"source {shlex.quote(ROS_SETUP)}", f"source {shlex.quote(WORKSPACE_SETUP)}"]
    if extra_setup:
        setup.append(f"source {shlex.quote(extra_setup)}")
    return ["bash", "-lc", " && ".join(setup + ["exec bash -lc " + shlex.quote(script)])]


def _resolve_repo_path(value: object) -> Path:
    path = Path(str(value)).expanduser()
    return (REPO_ROOT / path).resolve() if not path.is_absolute() else path.resolve()


def _portable_path(value: object) -> str:
    raw = str(value).strip()
    if raw.startswith("~"):
        return raw
    path = Path(raw).expanduser()
    resolved = (REPO_ROOT / path).resolve() if not path.is_absolute() else path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        try:
            return str(Path("..") / resolved.relative_to(REPO_ROOT.parent))
        except ValueError:
            return str(resolved)


def validate_config(raw: Mapping[str, object]) -> Dict[str, object]:
    config = dict(DEFAULT_CONFIG)
    config.update(dict(raw))
    for key in ("pointcloud_topic", "odom_topic", "rc_in_topic"):
        if not TOPIC_PATTERN.fullmatch(str(config[key])):
            raise ValueError(f"invalid ROS topic: {key}")
    if not FRAME_PATTERN.fullmatch(str(config["pointcloud_frame"])):
        raise ValueError("invalid pointcloud_frame")
    if float(config["sensor_yaw_offset_deg"]) != 0.0:
        raise ValueError("sensor_yaw_offset_deg is fixed at 0.0 for this installation")
    for key in ("rc_unlock_channel", "rc_unlock_channel_backup"):
        value = int(config[key])
        if not 0 <= value <= 17:
            raise ValueError(f"{key} must be between 0 and 17")
        config[key] = value
    threshold = int(config["rc_unlock_threshold"])
    if not 800 <= threshold <= 2200:
        raise ValueError("rc_unlock_threshold must be between 800 and 2200")
    config["rc_unlock_threshold"] = threshold
    radius = float(config["manual_obstacle_radius"])
    if not 0.1 <= radius <= 10.0:
        raise ValueError("manual_obstacle_radius must be between 0.1 and 10.0")
    config["manual_obstacle_radius"] = radius
    max_action = float(config["max_abs_action"])
    if not 0.0 < max_action <= 0.1:
        raise ValueError("max_abs_action must be in (0, 0.1]")
    config["max_abs_action"] = max_action
    for key in ("scenario_yaml", "model_path", "vecnorm_path", "model_search_root", "bag_directory"):
        config[key] = _portable_path(config[key])
    config["livox_workspace"] = str(config["livox_workspace"]).strip()
    xfer_format = int(config["livox_xfer_format"])
    if xfer_format not in (0, 1, 2):
        raise ValueError("livox_xfer_format must be 0 (PointXYZRTLT), 1 (CustomMsg) or 2 (PointCloud2)")
    config["livox_xfer_format"] = xfer_format
    mode = int(config["virtual_target_mode"])
    if mode not in (1, 3):
        raise ValueError("virtual_target_mode must be 1 or 3")
    config["virtual_target_mode"] = mode
    for key in ("virtual_target_wait_dist_start", "virtual_target_wait_dist_stop", "virtual_target_speed"):
        config[key] = float(config[key])
    if config["virtual_target_speed"] <= 0.0:
        raise ValueError("virtual_target_speed must be positive")
    if config["virtual_target_wait_dist_start"] < 0.0:
        raise ValueError("virtual_target_wait_dist_start must be non-negative")
    if config["virtual_target_wait_dist_stop"] <= config["virtual_target_wait_dist_start"]:
        raise ValueError("virtual_target_wait_dist_stop must exceed virtual_target_wait_dist_start")
    recent = config.get("recent_models", [])
    if not isinstance(recent, list):
        raise ValueError("recent_models must be a list")
    config["recent_models"] = recent[-12:]
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", str(config["livox_launch"])):
        raise ValueError("invalid livox_launch")
    config["ublox_workspace"] = str(config.get("ublox_workspace", "")).strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", str(config.get("ublox_launch", ""))):
        raise ValueError("invalid ublox_launch")
    config["camera_workspace"] = str(config.get("camera_workspace", "")).strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", str(config.get("camera_launch", ""))):
        raise ValueError("invalid camera_launch")
    ublox_port = str(config["ublox_serial_port"]).strip()
    if ublox_port != "auto" and not re.fullmatch(r"/dev/[A-Za-z0-9_.-]+", ublox_port):
        raise ValueError("ublox_serial_port must be auto or a device path under /dev")
    config["ublox_serial_port"] = ublox_port
    try:
        config["ublox_rtcm_tcp_host"] = str(ip_address(str(config["ublox_rtcm_tcp_host"]).strip()))
    except ValueError as exc:
        raise ValueError("ublox_rtcm_tcp_host must be a valid IP address") from exc
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", str(config["bag_session"])):
        raise ValueError("bag_session may contain only letters, digits, dot, dash and underscore")
    topics = config.get("bag_topics", DEFAULT_BAG_TOPICS)
    if not isinstance(topics, list):
        raise ValueError("bag_topics must be a list")
    normalized_topics = []
    for topic in topics:
        topic = str(topic).strip()
        if not TOPIC_PATTERN.fullmatch(topic):
            raise ValueError(f"invalid rosbag topic: {topic}")
        if topic not in normalized_topics:
            normalized_topics.append(topic)
    config["bag_record_all"] = bool(config.get("bag_record_all", True))
    config["bag_topics"] = normalized_topics
    if not config["bag_record_all"] and not normalized_topics:
        raise ValueError("select at least one rosbag topic")
    rates = config.get("mavlink_rates", {})
    if not isinstance(rates, Mapping):
        raise ValueError("mavlink_rates must be an object")
    config["mavlink_rates"] = {str(int(key)): float(value) for key, value in rates.items()}
    if not bool(config["no_actuation"]) and str(config.get("actuation_ack", "")) != "ENABLE CONTROL":
        raise ValueError("type ENABLE CONTROL before disabling no_actuation")
    return config


class TerminalSession:
    def __init__(self, session_id: str, cwd: Path):
        self.session_id = session_id
        self.lock = threading.Lock()
        self.output = ""
        self.master_fd, slave_fd = pty.openpty()
        self.process = subprocess.Popen(
            ["bash", "-l"], cwd=str(cwd), stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
            start_new_session=True, close_fds=True,
        )
        os.close(slave_fd)
        self.resize(100, 28)
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _read_loop(self):
        while self.process.poll() is None:
            try:
                ready, _, _ = select.select([self.master_fd], [], [], 0.25)
                if not ready:
                    continue
                chunk = os.read(self.master_fd, 8192)
                if not chunk:
                    break
                with self.lock:
                    self.output = (self.output + chunk.decode("utf-8", errors="replace"))[-100000:]
            except OSError:
                break

    def write(self, value: str):
        if self.process.poll() is not None:
            raise ValueError("终端会话已结束")
        os.write(self.master_fd, value.encode("utf-8"))

    def resize(self, columns: int, rows: int):
        columns = max(20, min(int(columns), 300))
        rows = max(5, min(int(rows), 120))
        fcntl.ioctl(self.master_fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, columns, 0, 0))
        if getattr(self, "process", None) and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGWINCH)

    def snapshot(self):
        with self.lock:
            output = self.output
        return {"id": self.session_id, "running": self.process.poll() is None, "output": output}

    def stop(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
        try:
            os.close(self.master_fd)
        except OSError:
            pass


class ProcessManager:
    def __init__(self):
        STATE_ROOT.mkdir(parents=True, exist_ok=True)
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        self._clear_runtime_state()
        self.lock = threading.RLock()
        self.processes: Dict[str, subprocess.Popen] = {}
        self.log_viewers: Dict[str, subprocess.Popen] = {}
        self.log_handles = {}
        self.config = self._load_config()
        self.current_controller = "neutral"
        self.active_bag = None
        self.terminals = {}
        self.shutting_down = False

    @staticmethod
    def _clear_runtime_state():
        """Discard prior app-session output without touching saved operator settings."""
        for path in LOG_ROOT.glob("*.log"):
            if path.is_file():
                path.unlink()
        if VIZ_SNAPSHOT_PATH.is_file():
            VIZ_SNAPSHOT_PATH.unlink()

    def _load_config(self):
        if CONFIG_PATH.is_file():
            try:
                raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
                old_bag_default = str(REPO_ROOT.parent / "field_bags")
                if str(raw.get("bag_directory", "")) == old_bag_default:
                    raw["bag_directory"] = "../rosbag"
                if raw.get("gcs_url") == "udp://:14550@10.168.1.21":
                    raw["gcs_url"] = DEFAULT_CONFIG["gcs_url"]
                if raw.get("ublox_serial_port") == "/dev/ttyUSB0":
                    raw["ublox_serial_port"] = "/dev/ublox"
                if raw.get("ublox_serial_port") == "auto":
                    raw["ublox_serial_port"] = "/dev/ublox"
                return validate_config(raw)
            except Exception:
                pass
        return dict(DEFAULT_CONFIG)

    def save_config(self, raw):
        config = validate_config(raw)
        current_pair = {"model": config["model_path"], "vecnorm": config["vecnorm_path"]}
        recent = [
            item for item in config.get("recent_models", [])
            if isinstance(item, Mapping) and item.get("model") != current_pair["model"]
        ]
        config["recent_models"] = (recent + [current_pair])[-12:]
        CONFIG_PATH.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        with self.lock:
            self.config = config
        return config

    def _start(self, name, command, cwd=None):
        with self.lock:
            existing = self.processes.get(name)
            if existing and existing.poll() is None:
                raise ValueError(f"{name} is already running")
            log_path = LOG_ROOT / f"{name}.log"
            handle = log_path.open("a", encoding="utf-8")
            handle.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] START {_shell_join(command)}\n")
            handle.flush()
            proc = subprocess.Popen(
                command,
                cwd=str(cwd or REPO_ROOT),
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            self.processes[name] = proc
            self.log_handles[name] = handle
            self._open_log_terminal(name, log_path)
            return {"name": name, "pid": proc.pid, "log": str(log_path)}

    def _open_log_terminal(self, name: str, log_path: Path):
        """Show live output without making a GUI terminal the managed ROS parent."""
        if not bool(self.config.get("open_process_terminals", True)) or not os.environ.get("DISPLAY"):
            return
        terminal = os.environ.get("TERMINAL") or "x-terminal-emulator"
        try:
            viewer = subprocess.Popen(
                [
                    terminal, "-T", f"USV - {name}", "-e", "bash", "-lc",
                    f"tail -n +1 -F {shlex.quote(str(log_path))}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                text=True,
            )
            self.log_viewers[name] = viewer
        except OSError:
            # The service itself remains correctly managed even if no GUI terminal is available.
            pass

    def _stop_log_terminal(self, name: str):
        viewer = self.log_viewers.pop(name, None)
        if viewer and viewer.poll() is None:
            try:
                os.killpg(viewer.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _close_log_handle(self, name: str):
        handle = self.log_handles.pop(name, None)
        if handle:
            handle.close()

    def stop(self, name, timeout=8.0):
        with self.lock:
            proc = self.processes.get(name)
            if not proc or proc.poll() is not None:
                self._stop_log_terminal(name)
                self._close_log_handle(name)
                return {"name": name, "stopped": True, "already_stopped": True}
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2.0)
        self._close_log_handle(name)
        self._stop_log_terminal(name)
        return {"name": name, "stopped": True, "returncode": proc.returncode}

    def shutdown(self):
        """Stop every app-managed process before the operator console exits."""
        with self.lock:
            if self.shutting_down:
                return {"already_shutting_down": True, "errors": []}
            self.shutting_down = True
        results = {}
        errors = []
        names = ["rosbag", "web_visualization", "rviz", "experiment", "camera", "lidar", "ublox", "mavlink_rates", "mavros", "roscore"]
        for name in names:
            try:
                results[name] = self.stop(name, timeout=30.0 if name == "rosbag" else 8.0)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        for name, terminal in list(self.terminals.items()):
            try:
                terminal.stop()
            except Exception as exc:
                errors.append(f"terminal {name}: {exc}")
        if self.active_bag:
            try:
                results["recording"] = self._finalize_bag_record()
            except Exception as exc:
                errors.append(f"rosbag finalization: {exc}")
        for name in list(self.log_handles):
            self._close_log_handle(name)
        for name in list(self.log_viewers):
            self._stop_log_terminal(name)
        return {"processes": results, "errors": errors}

    def run_once(self, command, timeout=12.0, cwd=None):
        completed = subprocess.run(
            command,
            cwd=str(cwd or REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        output = (completed.stdout + completed.stderr).strip()
        if completed.returncode != 0:
            raise RuntimeError(output or f"command failed with {completed.returncode}")
        return output

    @staticmethod
    def _require_serial_access(device: str, service: str):
        path = Path(device)
        if not path.exists():
            raise ValueError(f"{service} 串口不存在: {device}")
        if not stat.S_ISCHR(path.stat().st_mode):
            raise ValueError(f"{service} 串口不是字符设备: {device}")
        if not os.access(path, os.R_OK | os.W_OK):
            raise ValueError(
                f"当前启动 App 的用户没有 {device} 的读写权限，{service} 未启动。"
                "请由管理员将该用户加入串口设备所属组（通常为 dialout）后重新登录，再重试。"
            )

    def _resolve_ublox_serial_port(self) -> str:
        configured = str(self.config["ublox_serial_port"])
        if configured != "auto":
            self._require_serial_access(configured, "UBlox")
            return configured
        fcu_device = str(self.config["fcu_url"]).rsplit(":", 1)[0]
        candidates = [
            path for pattern in ("ttyACM*", "ttyUSB*") for path in sorted(Path("/dev").glob(pattern))
            if str(path) != fcu_device and stat.S_ISCHR(path.stat().st_mode)
        ]
        usable = [path for path in candidates if os.access(path, os.R_OK | os.W_OK)]
        if len(usable) == 1:
            return str(usable[0])
        found = ", ".join(str(path) for path in candidates) or "无"
        if not usable:
            raise ValueError(
                "未找到可读写的 UBlox 串口（已排除 FCU 串口）。发现："
                f"{found}。请配置 /dev/ublox 的 udev 稳定别名，或在现场参数中明确指定端口。"
            )
        raise ValueError(
            f"发现多个可能的 UBlox 串口：{', '.join(str(path) for path in usable)}。"
            "为避免连接到错误设备，请配置 /dev/ublox 的 udev 稳定别名，或在现场参数中明确指定端口。"
        )

    def _mavlink_rate_guard_command(self):
        calls = " ".join(
            f"rosservice call /mavros/set_message_interval {shlex.quote(str(message_id))} {shlex.quote(str(rate))} >/dev/null 2>&1 || true;"
            for message_id, rate in self.config["mavlink_rates"].items()
        )
        return _ros_shell_script(
            "while ! rosservice list 2>/dev/null | grep -qx /mavros/set_message_interval; do sleep 1; done; "
            f"while true; do {calls} sleep 1; done"
        )

    def _ensure_web_visualization(self):
        existing = self.processes.get("web_visualization")
        if existing and existing.poll() is None:
            return {"name": "web_visualization", "pid": existing.pid, "already_running": True}
        bridge = APP_ROOT / "ros_viz_bridge.py"
        return self._start(
            "web_visualization",
            _ros_shell(["python3", str(bridge), "--output", str(VIZ_SNAPSHOT_PATH)]),
        )

    def _stack_command(self):
        cfg = self.config
        for key in ("scenario_yaml", "model_path", "vecnorm_path"):
            if not _resolve_repo_path(cfg[key]).is_file():
                raise ValueError(f"{key} not found: {cfg[key]}")
        args = [
            "roslaunch",
            "usv_obstacle_avoidance",
            "real_boat_virtual_obstacle_experiment.launch",
            f"pointcloud_topic:={cfg['pointcloud_topic']}",
            f"pointcloud_frame:={cfg['pointcloud_frame']}",
            f"odom_topic:={cfg['odom_topic']}",
            f"rc_in_topic:={cfg['rc_in_topic']}",
            f"rc_in_unlock_channel:={cfg['rc_unlock_channel']}",
            f"rc_in_unlock_threshold:={cfg['rc_unlock_threshold']}",
            f"scenario_yaml:={_resolve_repo_path(cfg['scenario_yaml'])}",
            f"use_scenario_yaml:={str(bool(cfg['use_scenario_yaml'])).lower()}",
            f"manual_obstacle_radius:={cfg['manual_obstacle_radius']}",
            f"merge_physical_scan:={str(bool(cfg['merge_physical_scan'])).lower()}",
            "selected_controller:=neutral",
            f"dry_run:={str(bool(cfg['dry_run'])).lower()}",
            f"no_actuation:={str(bool(cfg['no_actuation'])).lower()}",
            f"max_abs_action:={cfg['max_abs_action']}",
            f"model_path:={_resolve_repo_path(cfg['model_path'])}",
            f"vecnorm_path:={_resolve_repo_path(cfg['vecnorm_path'])}",
            f"virtual_target_mode:={cfg['virtual_target_mode']}",
            f"virtual_target_wait_dist_start:={cfg['virtual_target_wait_dist_start']}",
            f"virtual_target_wait_dist_stop:={cfg['virtual_target_wait_dist_stop']}",
            f"virtual_target_speed:={cfg['virtual_target_speed']}",
        ]
        return _ros_shell(args)

    def action(self, action, payload):
        cfg = self.config
        if action == "start_roscore":
            return self._start("roscore", _ros_shell(["roscore"]))
        if action == "start_mavros":
            fcu_url = str(cfg["fcu_url"])
            if fcu_url.startswith("/dev/"):
                self._require_serial_access(fcu_url.rsplit(":", 1)[0], "MAVROS")
            mavros = self._start(
                "mavros",
                _ros_shell(
                    [
                        "roslaunch",
                        "mavros",
                        "apm.launch",
                        f"fcu_url:={cfg['fcu_url']}",
                        f"gcs_url:={cfg['gcs_url']}",
                    ]
                ),
            )
            try:
                rate_guard = self._start("mavlink_rates", self._mavlink_rate_guard_command())
            except Exception:
                self.stop("mavros")
                raise
            return {"mavros": mavros, "mavlink_rates": rate_guard}
        if action == "start_ublox":
            # 允许从请求中覆盖串口和 RTCM 地址，未提供则沿用配置默认值
            serial_port = str(payload.get("serial_port", "")).strip()
            if not serial_port:
                serial_port = self._resolve_ublox_serial_port()
            else:
                if serial_port != "auto":
                    self._require_serial_access(serial_port, "UBlox")
            rtcm_host = str(payload.get("rtcm_tcp_host", "")).strip()
            if not rtcm_host:
                rtcm_host = str(cfg["ublox_rtcm_tcp_host"])
            else:
                try:
                    rtcm_host = str(ip_address(rtcm_host))
                except ValueError as exc:
                    raise ValueError(f"RTCM TCP 地址无效: {rtcm_host}") from exc
            ublox_workspace = Path(str(cfg.get("ublox_workspace", "")).strip() or "~/equipment_ros_driver/ublox_ws").expanduser().resolve()
            setup = str(ublox_workspace / "devel/setup.bash")
            return self._start(
                "ublox",
                _ros_shell(
                    [
                        "roslaunch",
                        "ublox_driver",
                        str(cfg.get("ublox_launch", "ublox_driver.launch")),
                        f"input_serial_port:={serial_port}",
                        f"rtcm_tcp_host:={rtcm_host}",
                    ],
                    extra_setup=setup,
                ),
                cwd=ublox_workspace,
            )
        if action == "apply_mavlink_rates":
            results = {}
            for message_id, rate in cfg["mavlink_rates"].items():
                results[message_id] = self.run_once(
                    _ros_shell(
                        ["rosservice", "call", "/mavros/set_message_interval", str(message_id), str(rate)]
                    )
                )
            return results
        if action == "start_lidar":
            # 允许从请求中覆盖 xfer_format，未提供则沿用配置默认值
            xfer_format = payload.get("xfer_format")
            if xfer_format is None:
                xfer_format = int(cfg["livox_xfer_format"])
            else:
                xfer_format = int(xfer_format)
                if xfer_format not in (0, 1, 2):
                    raise ValueError("xfer_format must be 0 (PointXYZRTLT), 1 (CustomMsg) or 2 (PointCloud2)")
            livox_workspace = Path(str(cfg["livox_workspace"])).expanduser().resolve()
            setup = str(livox_workspace / "devel/setup.bash")
            return self._start(
                "lidar",
                _ros_shell(
                    [
                        "roslaunch",
                        "livox_ros_driver2",
                        str(cfg["livox_launch"]),
                        f"xfer_format:={xfer_format}",
                    ],
                    extra_setup=setup,
                ),
                cwd=livox_workspace,
            )
        if action == "check_lidar_topic":
            topic = str(cfg["pointcloud_topic"])
            return {
                "topic": topic,
                "expected_type": "sensor_msgs/PointCloud2 (xfer_format=2)",
                "type": self.run_once(_ros_shell(["rostopic", "type", topic]), timeout=4.0),
                "topic_info": self.run_once(_ros_shell(["rostopic", "info", topic]), timeout=4.0),
                "one_message_summary": self.run_once(
                    _ros_shell(["rostopic", "echo", "-n", "1", "--noarr", topic]), timeout=8.0
                ),
            }
        if action == "start_camera":
            camera_workspace = Path(str(cfg.get("camera_workspace", "")).strip() or "~/equipment_ros_driver/mvs_ws").expanduser().resolve()
            setup = str(camera_workspace / "devel/setup.bash")
            return self._start(
                "camera",
                _ros_shell(
                    [
                        "roslaunch",
                        "mvs_ros_pkg",
                        str(cfg.get("camera_launch", "mvs_camera_trigger.launch")),
                    ],
                    extra_setup=setup,
                ),
                cwd=camera_workspace,
            )
        if action == "start_stack":
            return self._start("experiment", self._stack_command())
        if action == "start_rviz":
            rviz_config = REPO_ROOT / "usv_obstacle_avoidance/config/real_boat_operator.rviz"
            return {
                "rviz": self._start("rviz", _ros_shell(["rviz", "-d", str(rviz_config)])),
                "boat_visualization": self._ensure_web_visualization(),
            }
        if action == "start_web_visualization":
            return self._ensure_web_visualization()
        if action == "start_rosbag":
            bag_dir = _resolve_repo_path(cfg["bag_directory"])
            bag_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            prefix = bag_dir / f"{cfg['bag_session']}_{stamp}"
            record = {
                "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "prefix": str(prefix),
                "controller": self.current_controller,
                "model": str(cfg["model_path"]),
                "vecnorm": str(cfg["vecnorm_path"]),
                "tags": str(cfg.get("bag_tags", "")),
                "record_all": bool(cfg.get("bag_record_all", True)),
                "topics": list(cfg.get("bag_topics", [])),
            }
            record_args = ["rosbag", "record"]
            if bool(cfg.get("bag_record_all", True)):
                record_args.append("-a")
            else:
                topics = list(cfg.get("bag_topics", []))
                if not topics:
                    raise ValueError("至少选择一个 Rosbag 话题")
                record_args.extend(topics)
            record_args.extend(["-O", str(prefix)])
            result = self._start("rosbag", _ros_shell(record_args))
            self.active_bag = record
            return result
        if action == "list_rosbag_topics":
            output = self.run_once(_ros_shell(["rostopic", "list"]), timeout=6.0)
            topics = sorted({line.strip() for line in output.splitlines() if TOPIC_PATTERN.fullmatch(line.strip())})
            return {"topics": topics}
        if action == "get_topic_frequencies":
            topic_list = [str(t).strip() for t in payload.get("topics", []) if TOPIC_PATTERN.fullmatch(str(t).strip())]
            if not topic_list:
                raise ValueError("请至少选择一个话题")
            frequencies: Dict[str, float] = {}
            hz_re = re.compile(r'average rate:\s*([\d.]+)')
            def _get_topic_hz(topic: str):
                try:
                    script = f"timeout 2 rostopic hz {shlex.quote(topic)} 2>/dev/null | grep 'average rate' | tail -1"
                    result = subprocess.run(
                        _ros_shell_script(script),
                        capture_output=True, text=True, timeout=5,
                    )
                    match = hz_re.search(result.stdout)
                    if match:
                        return (topic, round(float(match.group(1)), 1))
                except Exception:
                    pass
                return (topic, None)
            with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
                futures = [executor.submit(_get_topic_hz, t) for t in topic_list]
                for future in concurrent.futures.as_completed(futures):
                    try:
                        t, hz = future.result()
                        if hz is not None:
                            frequencies[t] = hz
                    except Exception:
                        pass
            return {"frequencies": frequencies}
        if action.startswith("stop_") and action not in {"stop_all", "stop_terminal"}:
            name = action[len("stop_"):]
            aliases = {"stack": "experiment"}
            actual_name = aliases.get(name, name)
            if actual_name == "mavros":
                result = self.stop(actual_name)
                result["mavlink_rates"] = self.stop("mavlink_rates")
            else:
                result = self.stop(actual_name, timeout=30.0 if actual_name == "rosbag" else 8.0)
            if actual_name == "rosbag":
                result["recording"] = self._finalize_bag_record()
            return result
        if action == "stop_all":
            names = ["rosbag", "web_visualization", "rviz", "experiment", "camera", "lidar", "ublox", "mavlink_rates", "mavros", "roscore"]
            result = {name: self.stop(name, timeout=30.0 if name == "rosbag" else 8.0) for name in names}
            for terminal in list(self.terminals.values()):
                terminal.stop()
            if self.active_bag:
                result["recording"] = self._finalize_bag_record()
            return result
        if action == "create_terminal":
            session_id = str(payload.get("id", "")).strip()
            cwd_value = str(payload.get("cwd", ".")).strip() or "."
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,48}", session_id):
                raise ValueError("无效的终端会话 ID")
            cwd = _resolve_repo_path(cwd_value)
            if not cwd.is_dir():
                raise ValueError(f"工作目录不存在: {cwd_value}")
            existing = self.terminals.get(session_id)
            if existing and existing.process.poll() is None:
                return existing.snapshot()
            self.terminals[session_id] = TerminalSession(session_id, cwd)
            return self.terminals[session_id].snapshot()
        if action == "terminal_input":
            session_id = str(payload.get("id", ""))
            value = str(payload.get("input", ""))
            terminal = self.terminals.get(session_id)
            if not terminal:
                raise ValueError("终端会话不存在")
            terminal.write(value)
            return {"written": len(value)}
        if action == "resize_terminal":
            terminal = self.terminals.get(str(payload.get("id", "")))
            if not terminal:
                raise ValueError("终端会话不存在")
            terminal.resize(int(payload.get("columns", 100)), int(payload.get("rows", 28)))
            return {"resized": True}
        if action == "stop_terminal":
            terminal = self.terminals.get(str(payload.get("id", "")))
            if not terminal:
                return {"stopped": True, "already_stopped": True}
            terminal.stop()
            return {"stopped": True}
        if action == "publish_nav_goal":
            x = float(payload.get("x"))
            y = float(payload.get("y"))
            yaw = float(payload.get("yaw", 0.0))
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("目标坐标必须为有限数")
            qx = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            frame = cfg["pointcloud_frame"]
            message = (
                "header: {{stamp: now, frame_id: '{frame}'}}\n"
                "pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{z: {qx}, w: {qw}}}}}"
            ).format(frame=frame, x=x, y=y, qx=qx, qw=qw)
            return {"output": self.run_once(_ros_shell([
                "rostopic", "pub", "-1", "/move_base_simple/goal", "geometry_msgs/PoseStamped", message
            ]))}
        if action == "publish_initial_pose":
            x = float(payload.get("x"))
            y = float(payload.get("y"))
            yaw = float(payload.get("yaw", 0.0))
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError("初始位姿坐标必须为有限数")
            qx = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            frame = cfg["pointcloud_frame"]
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = 0.25
            covariance[35] = 0.0685389
            message = json.dumps({
                "header": {"stamp": "now", "frame_id": frame},
                "pose": {
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {"z": qx, "w": qw},
                    },
                    "covariance": covariance,
                },
            })
            return {"output": self.run_once(_ros_shell([
                "rostopic", "pub", "-1", "/initialpose", "geometry_msgs/PoseWithCovarianceStamped", message
            ]))}
        if action == "select_controller":
            controller = str(payload.get("controller", "neutral"))
            if controller not in {"neutral", "ppo", "apf_pid", "mptc"}:
                raise ValueError("unsupported controller")
            output = self.run_once(_ros_shell(["rostopic", "pub", "-1", "/usv/selected_controller", "std_msgs/String", f"data: '{controller}'"]))
            self.current_controller = controller
            return {"output": output}
        if action in {"publish_rc_unlock", "publish_rc_unlock_backup", "publish_rc_lock"}:
            channel_index = int(cfg["rc_unlock_channel"])
            if action == "publish_rc_unlock_backup":
                channel_index = int(cfg["rc_unlock_channel_backup"])
            channels = [1500] * 18
            channels[10:] = [0] * 8
            if action != "publish_rc_lock":
                channels[channel_index] = 2000
            message = (
                "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ''}\n"
                f"rssi: 0\n"
                f"channels: {channels}"
            )
            return {
                "channel_index": channel_index,
                "output": self.run_once(
                    _ros_shell(["rostopic", "pub", "-1", str(cfg["rc_in_topic"]), "mavros_msgs/RCIn", message])
                ),
            }
        if action == "e_stop":
            return {"output": self.run_once(_ros_shell(["rostopic", "pub", "-1", "/usv/e_stop", "std_msgs/String", "data: 'true'"]))}
        if action == "clear_e_stop":
            return {"output": self.run_once(_ros_shell(["rostopic", "pub", "-1", "/usv/e_stop", "std_msgs/String", "data: 'false'"]))}
        if action == "scene_command":
            command = str(payload.get("command", ""))
            if command not in {"reset_manual", "reload_scenario", "clear_all"}:
                raise ValueError("unsupported scene command")
            output = self.run_once(
                _ros_shell(["rostopic", "pub", "-1", "/usv/virtual_obstacle_command", "std_msgs/String", f"data: '{command}'"])
            )
            if command == "reload_scenario":
                output += "\n" + self.run_once(
                    _ros_shell(["rostopic", "pub", "-1", "/usv/scenario_reload", "std_msgs/Empty", "{}"])
                )
            return {"output": output}
        if action == "probe":
            return self.probe()
        raise ValueError(f"unsupported action: {action}")

    def _finalize_bag_record(self):
        if not self.active_bag:
            return None
        record = dict(self.active_bag)
        bag_path = Path(f"{record['prefix']}.bag")
        active_path = Path(f"{record['prefix']}.bag.active")
        deadline = time.monotonic() + 20.0
        while not bag_path.is_file() and time.monotonic() < deadline:
            time.sleep(0.25)
        if not bag_path.is_file():
            raise RuntimeError(
                f"Rosbag 进程已退出，但录制文件尚未完成：{active_path}。"
                "请保留 .active 文件并检查磁盘空间与 rosbag 日志。"
            )
        record["stopped_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        bag_dir = Path(record["prefix"]).parent
        index_path = bag_dir / "recordings.md"
        if not index_path.exists():
            index_path.write_text(
                "# 实船 Rosbag 录制索引\n\n"
                "每次停止录制后由操作台自动追加。\n\n",
                encoding="utf-8",
            )
        entry = (
            f"## {Path(record['prefix']).name}\n\n"
            f"- 开始时间：{record['started_at']}\n"
            f"- 结束时间：{record['stopped_at']}\n"
            f"- 控制方式：`{record['controller']}`\n"
            f"- 模型：`{record['model']}`\n"
            f"- VecNormalize：`{record['vecnorm']}`\n"
            f"- 录制话题：{'`-a` 全部话题' if record.get('record_all') else ', '.join(f'`{topic}`' for topic in record.get('topics', []))}\n"
            f"- 标签：{record['tags'] or '未填写'}\n\n"
        )
        with index_path.open("a", encoding="utf-8") as handle:
            handle.write(entry)
        self.active_bag = None
        return {"path": str(bag_path), "index": str(index_path), "size_bytes": bag_path.stat().st_size}

    def model_candidates(self):
        root = _resolve_repo_path(self.config.get("model_search_root", "runs"))
        if not root.is_dir():
            return {"root": _portable_path(root), "models": [], "vecnorms": [], "pairs": []}
        models = sorted(root.rglob("*.zip"))
        vecnorms = sorted(root.rglob("*.pkl"))
        model_values = [_portable_path(path) for path in models]
        vec_values = [_portable_path(path) for path in vecnorms if "vec_normalize" in path.name]
        pairs = []
        vec_by_dir = {}
        for path in vecnorms:
            vec_by_dir.setdefault(path.parent, []).append(path)
        for model in models:
            suffix = model.stem[5:] if model.stem.startswith("ckpt_") else model.stem
            choices = vec_by_dir.get(model.parent, [])
            match = next((item for item in choices if item.stem == f"vec_normalize_{suffix}"), None)
            if match is None:
                match = next((item for item in choices if item.name == "vec_normalize.pkl"), None)
            if match:
                pairs.append({"model": _portable_path(model), "vecnorm": _portable_path(match)})
        return {"root": _portable_path(root), "models": model_values, "vecnorms": vec_values, "pairs": pairs}

    def probe(self):
        cfg = self.config
        checks = {}
        commands = {
            "pointcloud_type": ["rostopic", "type", str(cfg["pointcloud_topic"])],
            "pointcloud_header": ["rostopic", "echo", "-n", "1", f"{cfg['pointcloud_topic']}/header"],
            "odom_header": ["rostopic", "echo", "-n", "1", f"{cfg['odom_topic']}/header"],
            "rc_sample": ["rostopic", "echo", "-n", "1", str(cfg["rc_in_topic"])],
            "virtual_status": ["rostopic", "echo", "-n", "1", "/usv/virtual_obstacle_status"],
            "mux_status": ["rostopic", "echo", "-n", "1", "/usv/controller_mux_status"],
        }
        for name, command in commands.items():
            try:
                checks[name] = {"ok": True, "output": self.run_once(_ros_shell(command), timeout=3.0)}
            except Exception as exc:
                checks[name] = {"ok": False, "output": str(exc)}
        return checks

    def status(self):
        with self.lock:
            processes = {}
            names = ["roscore", "mavros", "ublox", "lidar", "camera", "experiment", "rviz", "web_visualization", "rosbag"]
            for name in names:
                proc = self.processes.get(name)
                running = bool(proc and proc.poll() is None)
                processes[name] = {
                    "running": running,
                    "pid": proc.pid if proc else None,
                    "returncode": None if running or not proc else proc.returncode,
                }
        errors = []
        logs = {}
        for path in sorted(LOG_ROOT.glob("*.log")):
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()[-120:]
            logs[path.stem] = "\n".join(lines[-40:])
            for line in lines:
                if re.search(r"\b(ERROR|FATAL|RLException|Traceback)\b", line, re.IGNORECASE):
                    errors.append(f"{path.stem}: {line[-300:]}")
        terminals = {name: terminal.snapshot() for name, terminal in self.terminals.items()}
        return {"processes": processes, "terminals": terminals, "errors": errors[-30:], "logs": logs, "config": self.config}


MANAGER = ProcessManager()


class Handler(BaseHTTPRequestHandler):
    server_version = "RealBoatOperator/1.0"

    def log_message(self, _format, *_args):
        return

    def _json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length > 1024 * 1024:
            raise ValueError("request too large")
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/status":
            self._json({"ok": True, "data": MANAGER.status()})
            return
        if path == "/api/models":
            self._json({"ok": True, "data": MANAGER.model_candidates()})
            return
        if path == "/api/visualization":
            if VIZ_SNAPSHOT_PATH.is_file():
                try:
                    self._json({"ok": True, "data": json.loads(VIZ_SNAPSHOT_PATH.read_text(encoding="utf-8"))})
                    return
                except Exception:
                    pass
            self._json({"ok": True, "data": {"frame": "map", "pose": None, "path": [], "markers": {}, "stamp": 0.0}})
            return
        if path == "/api/terminals":
            self._json({"ok": True, "data": {name: terminal.snapshot() for name, terminal in MANAGER.terminals.items()}})
            return
        static = {"/": "index.html", "/app.js": "app.js", "/styles.css": "styles.css"}.get(path)
        if not static:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        file_path = APP_ROOT / "static" / static
        body = file_path.read_bytes()
        content_type = "text/html" if static.endswith(".html") else "text/css" if static.endswith(".css") else "application/javascript"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        try:
            body = self._read_json()
            if self.path == "/api/config":
                result = MANAGER.save_config(body)
            elif self.path == "/api/action":
                result = MANAGER.action(str(body.get("action", "")), body)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self._json({"ok": True, "data": result})
        except Exception as exc:
            self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)

    def handle_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, handle_sigterm)
    print(f"Real-boat operator console: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        result = MANAGER.shutdown()
        if result.get("errors"):
            print("Operator shutdown warnings: " + "; ".join(result["errors"]), flush=True)
        server.server_close()
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
