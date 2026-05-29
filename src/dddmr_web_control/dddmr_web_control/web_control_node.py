from __future__ import annotations

import atexit
import json
import math
import mimetypes
import os
import signal
import subprocess
import threading
import time
import urllib.parse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

try:
    from action_msgs.msg import GoalStatusArray
    import rclpy
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, Path as PathMsg
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
except ModuleNotFoundError:
    rclpy = None
    GoalStatusArray = None
    PackageNotFoundError = Exception
    get_package_share_directory = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Odometry = None
    PathMsg = None
    PointCloud2 = None
    point_cloud2 = None
    QoSDurabilityPolicy = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    Node = object

from dddmr_web_control.core import (
    CpuSnapshot,
    SourceCache,
    TerminalLogBuffer,
    ViewerRegistry,
    build_managed_launch_command,
    build_system_usage_payload,
    downsample_points,
    project_points_2d,
    quaternion_from_yaw,
    validate_pose_payload,
    yaw_from_quaternion,
)

LAYER_NAMES = {"global_map", "mapcloud", "mapground"}
MAPPING_LAYER_NAMES = {"global_map", "ground"}
ACTION_STATUS_NAMES = {
    0: "unknown",
    1: "accepted",
    2: "executing",
    3: "canceling",
    4: "succeeded",
    5: "canceled",
    6: "aborted",
}
STOP_SIGINT_TIMEOUT_SEC = 1.5
STOP_SIGTERM_TIMEOUT_SEC = 0.75
STOP_SIGKILL_TIMEOUT_SEC = 0.25
CLIENT_DISCONNECT_ERRORS = (BrokenPipeError, ConnectionResetError)


class ManagedLaunchProcess:
    def __init__(
        self,
        target: str,
        command: list[str],
        log_buffer: TerminalLogBuffer,
        logger: Any,
        on_exit: Callable[[str], None] | None = None,
    ) -> None:
        self.target = target
        self.command = command
        self.log_buffer = log_buffer
        self.logger = logger
        self.on_exit = on_exit
        self.process: subprocess.Popen[str] | None = None
        self.reader_thread: threading.Thread | None = None
        self.descendant_thread: threading.Thread | None = None
        self.pgid: int | None = None
        self.descendant_pids: set[int] = set()
        self.lock = threading.RLock()

    def start(self) -> None:
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return
            self.log_buffer.append("$ " + " ".join(self.command))
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["RCUTILS_COLORIZED_OUTPUT"] = "0"
            try:
                self.process = subprocess.Popen(
                    self.command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    env=env,
                    start_new_session=True,
                )
                self.pgid = os.getpgid(self.process.pid)
                self.descendant_pids = set()
            except FileNotFoundError as exc:
                self.log_buffer.append(f"failed to start: {exc}")
                self.log_buffer.set_process_state(
                    enabled=True, running=False, returncode=127
                )
                self.logger.error(f"Failed to start {self.target}: {exc}")
                return
            except Exception as exc:
                self.log_buffer.append(f"failed to start: {exc}")
                self.log_buffer.set_process_state(
                    enabled=True, running=False, returncode=1
                )
                self.logger.error(f"Failed to start {self.target}: {exc}")
                return

            self.log_buffer.set_process_state(
                enabled=True, running=True, returncode=None
            )
            self.reader_thread = threading.Thread(
                target=self._read_output,
                name=f"{self.target}_launch_log_reader",
                daemon=True,
            )
            self.reader_thread.start()
            self.descendant_thread = threading.Thread(
                target=self._monitor_descendants,
                name=f"{self.target}_launch_descendant_monitor",
                daemon=True,
            )
            self.descendant_thread.start()

    def _read_output(self) -> None:
        process = self.process
        if process is None:
            return
        try:
            if process.stdout is not None:
                for line in process.stdout:
                    self.log_buffer.append(line)
            returncode = process.wait()
        except Exception as exc:  # pragma: no cover - defensive reader guard
            self.log_buffer.append(f"log reader failed: {exc}")
            returncode = process.poll()
        self.log_buffer.set_process_state(
            enabled=True,
            running=False,
            returncode=returncode,
        )
        self._notify_exit()

    def _monitor_descendants(self) -> None:
        process = self.process
        if process is None:
            return
        while process.poll() is None:
            with self.lock:
                self.descendant_pids.update(
                    self._collect_descendant_pids(process.pid)
                )
            time.sleep(0.5)

    def stop(self) -> None:
        with self.lock:
            process = self.process
            if process is None and self.pgid is None:
                return
            if process is not None:
                self.descendant_pids.update(self._collect_descendant_pids(process.pid))
            self.log_buffer.append(f"stopping {self.target}...")
            self._signal_process_group(signal.SIGINT)
            if process is None or process.poll() is not None:
                self.log_buffer.set_process_state(
                    enabled=True,
                    running=False,
                    returncode=None if process is None else process.returncode,
                )
                self._notify_exit()
                return
            try:
                process.wait(timeout=STOP_SIGINT_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                self._signal_process_group(signal.SIGTERM)
                try:
                    process.wait(timeout=STOP_SIGTERM_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    self._signal_process_group(signal.SIGKILL)
                    process.kill()
                    try:
                        process.wait(timeout=STOP_SIGKILL_TIMEOUT_SEC)
                    except subprocess.TimeoutExpired:
                        pass
            self.log_buffer.set_process_state(
                enabled=True,
                running=False,
                returncode=process.returncode,
            )
            self._notify_exit()

    def _notify_exit(self) -> None:
        if self.on_exit is None:
            return
        try:
            self.on_exit(self.target)
        except Exception as exc:  # pragma: no cover - defensive callback guard
            self.logger.error(f"Managed exit callback failed for {self.target}: {exc}")

    def _signal_process_group(self, sig: signal.Signals) -> None:
        if self.pgid is not None:
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                pass
            except Exception:
                pass
        for pid in sorted(self.descendant_pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except Exception:
                continue
        if self.process is None:
            return
        try:
            self.process.send_signal(sig)
        except Exception:
            pass

    def is_running(self) -> bool:
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def _collect_descendant_pids(self, root_pid: int) -> set[int]:
        children_by_parent: dict[int, list[int]] = {}
        proc_dir = Path("/proc")
        for entry in proc_dir.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                stat = (entry / "stat").read_text(encoding="utf-8")
            except OSError:
                continue
            rparen = stat.rfind(")")
            if rparen == -1:
                continue
            fields = stat[rparen + 2 :].split()
            if len(fields) < 2:
                continue
            try:
                ppid = int(fields[1])
            except ValueError:
                continue
            children_by_parent.setdefault(ppid, []).append(pid)

        descendants: set[int] = set()
        stack = list(children_by_parent.get(root_pid, []))
        while stack:
            pid = stack.pop()
            if pid in descendants:
                continue
            descendants.add(pid)
            stack.extend(children_by_parent.get(pid, []))
        return descendants


def normalize_layer_name(name: str) -> str:
    if name not in LAYER_NAMES:
        raise ValueError(f"unknown layer: {name}")
    return name


def normalize_mapping_layer_name(name: str) -> str:
    if name not in MAPPING_LAYER_NAMES:
        raise ValueError(f"unknown mapping layer: {name}")
    return name


def success_response(message: str, stamp: float) -> dict[str, Any]:
    return {"ok": True, "message": message, "stamp": stamp}


def action_status_name(status_values: list[int]) -> str:
    if not status_values:
        return "idle"
    return ACTION_STATUS_NAMES.get(int(status_values[-1]), "unknown")


def navigation_readiness(odom_status: str, mapground_status: str) -> dict[str, Any]:
    if odom_status == "waiting":
        return {
            "ready": False,
            "status": "waiting_for_relocalization",
            "message": "waiting for relocalization",
        }
    if mapground_status != "loaded":
        return {
            "ready": False,
            "status": "waiting_for_navigation",
            "message": "waiting for navigation",
        }
    return {
        "ready": True,
        "status": "ready",
        "message": "ready",
    }


def error_response(message: str, stamp: float | None = None) -> dict[str, Any]:
    response: dict[str, Any] = {"ok": False, "message": message}
    if stamp is not None:
        response["stamp"] = stamp
    return response


def parse_path_topics(value: Any) -> list[str]:
    if isinstance(value, (list, tuple)):
        items = value
    elif isinstance(value, str):
        items = value.replace(";", ",").split(",")
    else:
        return []
    return [str(item).strip() for item in items if str(item).strip()]


def parse_bool_parameter(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def package_static_dir() -> Path:
    try:
        if get_package_share_directory is None:
            raise PackageNotFoundError()
        return Path(get_package_share_directory("dddmr_web_control")) / "static"
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / "static"


class WebControlNode(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("ROS 2 Python environment is not available")
        super().__init__("dddmr_web_control")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("global_map_topic", "/lio_sam/localization/global_map")
        self.declare_parameter("mapcloud_topic", "mapcloud")
        self.declare_parameter("mapground_topic", "mapground")
        self.declare_parameter("odom_topic", "/lio_sam/localization/odometry")
        self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")
        self.declare_parameter("mapping_ground_topic", "/lio_sam/mapping/ground_cloud_global")
        self.declare_parameter("mapping_path_topic", "/lio_sam/mapping/path")
        self.declare_parameter("mapping_odom_topic", "/lio_sam/mapping/odometry")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "goal_pose_3d")
        self.declare_parameter("action_status_topic", "/p2p_move_base/_action/status")
        self.declare_parameter("path_topics", "")
        self.declare_parameter("max_2d_points_per_layer", 60000)
        self.declare_parameter("max_3d_points_per_layer", 40000)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("use_sim", False)
        self.declare_parameter("localization_launch_package", "lio_sam_hesai")
        self.declare_parameter(
            "localization_launch_file", "localization_with_nonground.launch.py"
        )
        self.declare_parameter("localization_launch_args", "use_rviz:=false")
        self.declare_parameter("navigation_launch_package", "p2p_move_base")
        self.declare_parameter(
            "navigation_launch_file", "p2p_move_base_localization.launch.py"
        )
        self.declare_parameter("navigation_launch_args", "")
        self.declare_parameter("mapping_launch_package", "lio_sam_hesai")
        self.declare_parameter("mapping_launch_file", "mapping.launch.py")
        self.declare_parameter("mapping_launch_args", "use_rviz:=false")
        self.declare_parameter("managed_log_lines", 400)
        self.declare_parameter("viewer_timeout_sec", 10.0)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.max_2d_points = int(self.get_parameter("max_2d_points_per_layer").value)
        self.max_3d_points = int(self.get_parameter("max_3d_points_per_layer").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self.use_sim = parse_bool_parameter(self.get_parameter("use_sim").value)
        self.managed_log_lines = int(self.get_parameter("managed_log_lines").value)
        self.viewer_timeout = float(self.get_parameter("viewer_timeout_sec").value)
        self.static_dir = package_static_dir()

        self.lock = threading.RLock()
        self.layers = {
            "global_map": SourceCache("global_map", self.stale_timeout, static_source=True),
            "mapcloud": SourceCache("mapcloud", self.stale_timeout, static_source=True),
            "mapground": SourceCache("mapground", self.stale_timeout, static_source=True),
        }
        self.mapping_layers = {
            "global_map": SourceCache("mapping_global_map", self.stale_timeout, static_source=True),
            "ground": SourceCache("mapping_ground", self.stale_timeout, static_source=True),
        }
        self.odom = SourceCache("odom", self.stale_timeout)
        self.mapping_odom = SourceCache("mapping_odom", self.stale_timeout)
        self.mapping_path = SourceCache("mapping_path", self.stale_timeout)
        self.action_status = SourceCache("action_status", self.stale_timeout)
        self.paths: dict[str, SourceCache] = {}
        self.system_cpu_ticks: CpuSnapshot | None = None
        self.managed_logs = {
            "localization": TerminalLogBuffer(
                "localization", max_lines=self.managed_log_lines
            ),
            "navigation": TerminalLogBuffer(
                "navigation", max_lines=self.managed_log_lines
            ),
            "mapping": TerminalLogBuffer(
                "mapping", max_lines=self.managed_log_lines
            ),
        }
        self.managed_processes: dict[str, ManagedLaunchProcess] = {}
        self.viewers = ViewerRegistry(timeout_sec=self.viewer_timeout)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            5,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            5,
        )

        layer_topics = {
            "global_map": str(self.get_parameter("global_map_topic").value),
            "mapcloud": str(self.get_parameter("mapcloud_topic").value),
            "mapground": str(self.get_parameter("mapground_topic").value),
        }
        static_pointcloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        pointcloud_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for layer_name, topic in layer_topics.items():
            self.create_subscription(
                PointCloud2,
                topic,
                partial(self.pointcloud_callback, layer_name),
                static_pointcloud_qos,
            )

        mapping_layer_topics = {
            "global_map": str(self.get_parameter("mapping_global_map_topic").value),
            "ground": str(self.get_parameter("mapping_ground_topic").value),
        }
        for layer_name, topic in mapping_layer_topics.items():
            self.create_subscription(
                PointCloud2,
                topic,
                partial(self.mapping_pointcloud_callback, layer_name),
                pointcloud_qos,
            )

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            state_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("mapping_odom_topic").value),
            self.mapping_odom_callback,
            state_qos,
        )
        self.create_subscription(
            PathMsg,
            str(self.get_parameter("mapping_path_topic").value),
            self.mapping_path_callback,
            state_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("action_status_topic").value),
            self.action_status_callback,
            state_qos,
        )

        path_topics = parse_path_topics(self.get_parameter("path_topics").value)
        for topic in path_topics:
            self.paths[topic] = SourceCache(topic, self.stale_timeout)
            self.create_subscription(
                PathMsg,
                topic,
                partial(self.path_callback, topic),
                state_qos,
            )

        self.httpd: ThreadingHTTPServer | None = None
        self.http_thread: threading.Thread | None = None
        self.configure_managed_processes()
        self.start_http_server()

    def now_sec(self) -> float:
        return time.time()

    def ros_stamp(self):
        return self.get_clock().now().to_msg()

    def managed_launch_args(self, extra_args: str) -> str:
        use_sim_arg = f"use_sim:={str(self.use_sim).lower()}"
        return " ".join(arg for arg in [use_sim_arg, extra_args.strip()] if arg)

    def configure_managed_processes(self) -> None:
        self.managed_logs["localization"].set_process_state(
            enabled=False, running=False, returncode=None
        )
        self.managed_logs["navigation"].set_process_state(
            enabled=False, running=False, returncode=None
        )
        self.managed_logs["mapping"].set_process_state(
            enabled=False, running=False, returncode=None
        )

        localization_command = build_managed_launch_command(
            str(self.get_parameter("localization_launch_package").value),
            str(self.get_parameter("localization_launch_file").value),
            self.managed_launch_args(str(self.get_parameter("localization_launch_args").value)),
        )
        navigation_command = build_managed_launch_command(
            str(self.get_parameter("navigation_launch_package").value),
            str(self.get_parameter("navigation_launch_file").value),
            self.managed_launch_args(str(self.get_parameter("navigation_launch_args").value)),
        )
        mapping_command = build_managed_launch_command(
            str(self.get_parameter("mapping_launch_package").value),
            str(self.get_parameter("mapping_launch_file").value),
            self.managed_launch_args(str(self.get_parameter("mapping_launch_args").value)),
        )
        self.managed_processes["localization"] = ManagedLaunchProcess(
            "localization",
            localization_command,
            self.managed_logs["localization"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
        )
        self.managed_processes["navigation"] = ManagedLaunchProcess(
            "navigation",
            navigation_command,
            self.managed_logs["navigation"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
        )
        self.managed_processes["mapping"] = ManagedLaunchProcess(
            "mapping",
            mapping_command,
            self.managed_logs["mapping"],
            self.get_logger(),
            on_exit=self.handle_managed_exit,
        )

    def clear_navigation_runtime_state(self) -> None:
        with self.lock:
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()

    def clear_map_runtime_state(self) -> None:
        with self.lock:
            for cache in self.layers.values():
                cache.clear()
            self.odom.clear()
            self.action_status.clear()
            for cache in self.paths.values():
                cache.clear()

    def clear_mapping_runtime_state(self) -> None:
        with self.lock:
            for cache in self.mapping_layers.values():
                cache.clear()
            self.mapping_odom.clear()
            self.mapping_path.clear()

    def handle_managed_exit(self, target: str) -> None:
        if target == "localization":
            self.clear_map_runtime_state()
        elif target == "navigation":
            self.clear_navigation_runtime_state()
        elif target == "mapping":
            self.clear_mapping_runtime_state()

    def stop_managed_processes(self) -> None:
        threads = [
            threading.Thread(
                target=process.stop,
                name=f"managed_launch_stop_{target}",
                daemon=True,
            )
            for target, process in self.managed_processes.items()
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=3.0)

    def start_managed_target(self, target: str) -> dict[str, Any]:
        if target == "all":
            for item in self.managed_processes:
                self.start_managed_target(item)
            return success_response("managed processes starting", self.now_sec())
        if target not in self.managed_logs:
            raise ValueError(f"unknown managed target: {target}")
        process = self.managed_processes[target]
        self.managed_logs[target].set_process_state(
            enabled=True,
            running=process.is_running(),
            returncode=None,
        )
        process.start()
        return success_response(f"{target} starting", self.now_sec())

    def stop_managed_target(self, target: str) -> dict[str, Any]:
        if target == "all":
            self.stop_managed_processes()
            return success_response("managed processes stopping", self.now_sec())
        if target not in self.managed_logs:
            raise ValueError(f"unknown managed target: {target}")
        process = self.managed_processes.get(target)
        if process is None:
            self.managed_logs[target].append(f"{target} is not managed by web_control")
            return success_response(f"{target} is not managed", self.now_sec())
        process.stop()
        return success_response(f"{target} stopping", self.now_sec())

    def pointcloud_callback(self, layer_name: str, msg: PointCloud2) -> None:
        self.update_pointcloud_cache(self.layers[layer_name], msg)

    def mapping_pointcloud_callback(self, layer_name: str, msg: PointCloud2) -> None:
        self.update_pointcloud_cache(self.mapping_layers[layer_name], msg)

    def update_pointcloud_cache(self, cache: SourceCache, msg: PointCloud2) -> None:
        points: list[tuple[float, float, float]] = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))

        if not points:
            return

        points_2d = downsample_points(points, self.max_2d_points)
        points_3d = downsample_points(points, self.max_3d_points)
        payload = {
            "points2d": project_points_2d(points_2d),
            "points3d": [[x, y, z] for x, y, z in points_3d],
        }
        with self.lock:
            cache.update(
                payload,
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        }
        payload = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(q),
            "frame_id": msg.header.frame_id,
        }
        with self.lock:
            self.odom.update(payload, stamp=self.now_sec(), frame_id=msg.header.frame_id)

    def mapping_odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        }
        payload = {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw_from_quaternion(q),
            "frame_id": msg.header.frame_id,
        }
        with self.lock:
            self.mapping_odom.update(payload, stamp=self.now_sec(), frame_id=msg.header.frame_id)

    def path_callback(self, topic: str, msg: PathMsg) -> None:
        points = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ]
        with self.lock:
            self.paths[topic].update(
                {"points": points},
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )

    def mapping_path_callback(self, msg: PathMsg) -> None:
        points = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ]
        with self.lock:
            self.mapping_path.update(
                {"points": points},
                stamp=self.now_sec(),
                frame_id=msg.header.frame_id,
                count=len(points),
            )

    def action_status_callback(self, msg: GoalStatusArray) -> None:
        status_values = [int(status.status) for status in msg.status_list]
        payload = {
            "status": action_status_name(status_values),
            "status_values": status_values,
        }
        with self.lock:
            self.action_status.update(
                payload,
                stamp=self.now_sec(),
                frame_id="",
                count=len(status_values),
            )

    def state_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            odom_status = self.odom.status(now)
            mapground_status = self.layers["mapground"].status(now)
            return {
                "layers": {
                    name: cache.summary(now) for name, cache in self.layers.items()
                },
                "odom": {
                    **self.odom.summary(now),
                    "pose": self.odom.data,
                },
                "paths": {
                    topic: cache.summary(now) for topic, cache in self.paths.items()
                },
                "navigation_action": {
                    **self.action_status.summary(now),
                    "data": self.action_status.data
                    or {"status": "idle", "status_values": []},
                },
                "navigation_ready": navigation_readiness(
                    odom_status, mapground_status
                ),
                "viewers": self.viewers.count(now),
                "map_frame": self.map_frame,
                "server_time": now,
            }

    def viewer_heartbeat(self, payload: dict[str, Any]) -> dict[str, Any]:
        viewer_id = str(payload.get("viewer_id", "")).strip()
        return {
            **success_response("viewer heartbeat", self.now_sec()),
            "viewers": self.viewers.heartbeat(viewer_id, self.now_sec()),
        }

    def mapping_state_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            return {
                "layers": {
                    name: cache.summary(now)
                    for name, cache in self.mapping_layers.items()
                },
                "path": self.mapping_path.summary(now),
                "odom": {
                    **self.mapping_odom.summary(now),
                    "pose": self.mapping_odom.data,
                },
                "map_frame": self.map_frame,
                "server_time": now,
            }

    def pose_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            return {
                "odom": {
                    **self.odom.summary(now),
                    "pose": self.odom.data,
                },
                "mapping_odom": {
                    **self.mapping_odom.summary(now),
                    "pose": self.mapping_odom.data,
                },
                "map_frame": self.map_frame,
                "server_time": now,
            }

    def layer_payload(self, layer_name: str, mode: str) -> dict[str, Any]:
        layer = normalize_layer_name(layer_name)
        now = self.now_sec()
        with self.lock:
            cache = self.layers[layer]
            data = cache.data or {}
            key = "points3d" if mode == "3d" else "points2d"
            return {
                "layer": layer,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get(key, []),
            }

    def mapping_layer_payload(self, layer_name: str) -> dict[str, Any]:
        layer = normalize_mapping_layer_name(layer_name)
        now = self.now_sec()
        with self.lock:
            cache = self.mapping_layers[layer]
            data = cache.data or {}
            return {
                "layer": layer,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get("points3d", []),
            }

    def path_payload(self, topic: str) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            if topic not in self.paths:
                raise ValueError(f"unknown path topic: {topic}")
            cache = self.paths[topic]
            data = cache.data or {"points": []}
            return {
                "topic": topic,
                "status": cache.status(now),
                "frame_id": cache.frame_id,
                "count": cache.count,
                "points": data.get("points", []),
            }

    def mapping_path_payload(self) -> dict[str, Any]:
        now = self.now_sec()
        with self.lock:
            data = self.mapping_path.data or {"points": []}
            return {
                "topic": "mapping_path",
                "status": self.mapping_path.status(now),
                "frame_id": self.mapping_path.frame_id,
                "count": self.mapping_path.count,
                "points": data.get("points", []),
            }

    def terminal_logs_payload(self) -> dict[str, Any]:
        return {
            "logs": {
                target: log_buffer.snapshot()
                for target, log_buffer in self.managed_logs.items()
            }
        }

    def system_payload(self) -> dict[str, Any]:
        with self.lock:
            previous_ticks = self.system_cpu_ticks
        payload, current_ticks = build_system_usage_payload(
            Path("/proc/stat").read_text(encoding="utf-8"),
            Path("/proc/meminfo").read_text(encoding="utf-8"),
            previous_ticks=previous_ticks,
            stamp=self.now_sec(),
        )
        with self.lock:
            self.system_cpu_ticks = current_ticks
        return payload

    def publish_initialpose(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = validate_pose_payload(payload)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.ros_stamp()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = pose["x"]
        msg.pose.pose.position.y = pose["y"]
        msg.pose.pose.position.z = pose["z"]
        q = quaternion_from_yaw(pose["yaw"])
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initialpose_pub.publish(msg)
        return success_response("initial pose published", self.now_sec())

    def publish_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        pose = validate_pose_payload(payload)
        msg = PoseStamped()
        msg.header.stamp = self.ros_stamp()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = pose["x"]
        msg.pose.position.y = pose["y"]
        msg.pose.position.z = pose["z"]
        q = quaternion_from_yaw(pose["yaw"])
        msg.pose.orientation.x = q["x"]
        msg.pose.orientation.y = q["y"]
        msg.pose.orientation.z = q["z"]
        msg.pose.orientation.w = q["w"]
        self.goal_pub.publish(msg)
        return success_response("goal published", self.now_sec())

    def start_http_server(self) -> None:
        handler = self.make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.httpd.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever, name="web_control_http", daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Web control console serving {self.static_dir} on http://{self.host}:{self.port}"
        )

    def stop_http_server(self) -> None:
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
            finally:
                self.httpd.server_close()
            self.httpd = None
        if self.http_thread is not None:
            self.http_thread.join(timeout=0.5)
            self.http_thread = None

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # type: ignore[no-untyped-def]
                node.get_logger().debug(fmt % args)

            def do_GET(self):  # type: ignore[no-untyped-def]
                parsed = urllib.parse.urlparse(self.path)
                query = urllib.parse.parse_qs(parsed.query)
                try:
                    if parsed.path == "/api/state":
                        self.send_json(node.state_payload())
                    elif parsed.path == "/api/pose":
                        self.send_json(node.pose_payload())
                    elif parsed.path == "/api/map2d":
                        layer = query.get("layer", ["global_map"])[0]
                        self.send_json(node.layer_payload(layer, "2d"))
                    elif parsed.path == "/api/map3d":
                        layer = query.get("layer", ["global_map"])[0]
                        self.send_json(node.layer_payload(layer, "3d"))
                    elif parsed.path == "/api/path":
                        topic = query.get("topic", [""])[0]
                        self.send_json(node.path_payload(topic))
                    elif parsed.path == "/api/logs":
                        self.send_json(node.terminal_logs_payload())
                    elif parsed.path == "/api/system":
                        self.send_json(node.system_payload())
                    elif parsed.path == "/api/mapping/state":
                        self.send_json(node.mapping_state_payload())
                    elif parsed.path == "/api/mapping/map3d":
                        layer = query.get("layer", ["global_map"])[0]
                        self.send_json(node.mapping_layer_payload(layer))
                    elif parsed.path == "/api/mapping/path":
                        self.send_json(node.mapping_path_payload())
                    else:
                        self.serve_static(parsed.path)
                except CLIENT_DISCONNECT_ERRORS:
                    return
                except ValueError as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=400)
                except Exception as exc:  # pragma: no cover - defensive HTTP guard
                    node.get_logger().error(f"HTTP GET failed: {exc}")
                    self.send_json(error_response("internal server error", node.now_sec()), status=500)

            def do_POST(self):  # type: ignore[no-untyped-def]
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw = self.rfile.read(length).decode("utf-8")
                    payload = json.loads(raw or "{}")
                    if self.path == "/api/initialpose":
                        self.send_json(node.publish_initialpose(payload))
                    elif self.path == "/api/goal":
                        self.send_json(node.publish_goal(payload))
                    elif self.path == "/api/managed/start":
                        self.send_json(
                            node.start_managed_target(str(payload.get("target", "")))
                        )
                    elif self.path == "/api/managed/stop":
                        self.send_json(
                            node.stop_managed_target(str(payload.get("target", "")))
                        )
                    elif self.path == "/api/viewer":
                        self.send_json(node.viewer_heartbeat(payload))
                    else:
                        self.send_json(error_response("unknown endpoint", node.now_sec()), status=404)
                except CLIENT_DISCONNECT_ERRORS:
                    return
                except (json.JSONDecodeError, ValueError) as exc:
                    self.send_json(error_response(str(exc), node.now_sec()), status=400)
                except Exception as exc:  # pragma: no cover - defensive HTTP guard
                    node.get_logger().error(f"HTTP POST failed: {exc}")
                    self.send_json(error_response("internal server error", node.now_sec()), status=500)

            def send_json(self, payload: dict[str, Any], status: int = 200) -> None:
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def serve_static(self, requested_path: str) -> None:
                rel = "index.html" if requested_path in {"", "/"} else requested_path.lstrip("/")
                rel_path = Path(os.path.normpath(rel))
                if rel_path.is_absolute() or ".." in rel_path.parts:
                    self.send_error(404)
                    return

                path = node.static_dir / rel_path
                if not path.is_file():
                    self.send_error(404)
                    return

                body = path.read_bytes()
                content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler

    def destroy_node(self) -> bool:
        self.stop_managed_processes()
        self.stop_http_server()
        return super().destroy_node()


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS 2 Python environment is not available")
    rclpy.init(args=args)
    node = WebControlNode()
    atexit.register(node.stop_managed_processes)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        atexit.unregister(node.stop_managed_processes)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
