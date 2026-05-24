import math
import re
import shlex
import threading
from dataclasses import dataclass
from collections import deque
from typing import Any, Iterable, Optional

CpuTicks = tuple[int, ...]
CpuSnapshot = dict[str, Any]


class ViewerRegistry:
    def __init__(self, timeout_sec: float = 10.0) -> None:
        self.timeout_sec = timeout_sec
        self.viewers: dict[str, float] = {}
        self.lock = threading.RLock()

    def heartbeat(self, viewer_id: str, now: float) -> int:
        if not viewer_id:
            raise ValueError("viewer_id is required")
        with self.lock:
            self.viewers[viewer_id] = now
            self._prune(now)
            return len(self.viewers)

    def count(self, now: float) -> int:
        with self.lock:
            self._prune(now)
            return len(self.viewers)

    def _prune(self, now: float) -> None:
        cutoff = now - self.timeout_sec
        expired = [viewer_id for viewer_id, stamp in self.viewers.items() if stamp < cutoff]
        for viewer_id in expired:
            del self.viewers[viewer_id]


def quaternion_from_yaw(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def yaw_from_quaternion(q: dict[str, float]) -> float:
    x = float(q.get("x", 0.0))
    y = float(q.get("y", 0.0))
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_number(payload: dict[str, Any], key: str) -> float:
    if key not in payload:
        raise ValueError(f"missing {key}")
    try:
        value = float(payload[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    return value


def validate_pose_payload(payload: dict[str, Any]) -> dict[str, float]:
    x = _finite_number(payload, "x")
    y = _finite_number(payload, "y")
    yaw = _finite_number(payload, "yaw")
    z = float(payload.get("z", 0.0))
    if not math.isfinite(z):
        raise ValueError("z must be finite")
    return {"x": x, "y": y, "z": z, "yaw": yaw}


def downsample_points(
    points: list[tuple[float, float, float]], max_points: int
) -> list[tuple[float, float, float]]:
    if max_points <= 0 or len(points) <= max_points:
        return list(points)
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1] and len(sampled) < max_points:
        sampled.append(points[-1])
    return sampled[:max_points]


def project_points_2d(points: Iterable[tuple[float, float, float]]) -> list[list[float]]:
    return [[float(x), float(y)] for x, y, _z in points]


def parse_proc_stat_cpu_ticks(text: str) -> CpuSnapshot:
    total: CpuTicks | None = None
    cores: list[CpuTicks] = []
    for line in text.splitlines():
        parts = line.split()
        if not parts:
            continue
        name = parts[0]
        if name != "cpu" and not re.fullmatch(r"cpu\d+", name):
            continue
        try:
            ticks = tuple(int(value) for value in parts[1:])
        except ValueError:
            continue
        if name == "cpu":
            total = ticks
        else:
            cores.append(ticks)
    if total is None:
        raise ValueError("missing aggregate cpu line")
    return {"total": total, "cores": cores}


def _cpu_idle_ticks(ticks: CpuTicks) -> int:
    idle = ticks[3] if len(ticks) > 3 else 0
    iowait = ticks[4] if len(ticks) > 4 else 0
    return idle + iowait


def _cpu_percent(previous: CpuTicks | None, current: CpuTicks | None) -> float | None:
    if previous is None or current is None:
        return None
    total_delta = sum(current) - sum(previous)
    idle_delta = _cpu_idle_ticks(current) - _cpu_idle_ticks(previous)
    if total_delta <= 0:
        return None
    busy_delta = max(0, total_delta - idle_delta)
    return round((busy_delta / total_delta) * 100.0, 2)


def calculate_cpu_usage(
    previous: CpuSnapshot | None, current: CpuSnapshot
) -> dict[str, Any]:
    previous_cores = previous.get("cores", []) if previous else []
    current_cores = current.get("cores", [])
    cores = []
    for index, current_ticks in enumerate(current_cores):
        previous_ticks = previous_cores[index] if index < len(previous_cores) else None
        cores.append({"id": index, "percent": _cpu_percent(previous_ticks, current_ticks)})
    return {
        "total_percent": _cpu_percent(
            previous.get("total") if previous else None,
            current.get("total"),
        ),
        "cores": cores,
    }


def parse_proc_meminfo(text: str) -> dict[str, Any]:
    values: dict[str, int] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        parts = raw_value.strip().split()
        if not parts:
            continue
        try:
            values[key] = int(parts[0])
        except ValueError:
            continue

    mem_total = max(0, values.get("MemTotal", 0))
    mem_available = max(0, values.get("MemAvailable", values.get("MemFree", 0)))
    mem_used = max(0, mem_total - mem_available)
    swap_total = max(0, values.get("SwapTotal", 0))
    swap_free = max(0, values.get("SwapFree", 0))
    swap_used = max(0, swap_total - swap_free)

    return {
        "memory": {
            "total_kb": mem_total,
            "used_kb": mem_used,
            "percent": round((mem_used / mem_total) * 100.0, 2) if mem_total else 0.0,
        },
        "swap": {
            "total_kb": swap_total,
            "used_kb": swap_used,
            "percent": round((swap_used / swap_total) * 100.0, 2) if swap_total else 0.0,
        },
    }


def build_system_usage_payload(
    proc_stat_text: str,
    proc_meminfo_text: str,
    previous_ticks: CpuSnapshot | None,
    stamp: float,
) -> tuple[dict[str, Any], CpuSnapshot]:
    current_ticks = parse_proc_stat_cpu_ticks(proc_stat_text)
    payload = {
        "stamp": stamp,
        "cpu": calculate_cpu_usage(previous_ticks, current_ticks),
        **parse_proc_meminfo(proc_meminfo_text),
    }
    return payload, current_ticks


@dataclass
class SourceCache:
    name: str
    stale_timeout_sec: float
    static_source: bool = False
    data: Optional[Any] = None
    stamp: Optional[float] = None
    frame_id: str = ""
    count: int = 0

    def update(
        self, data: Any, stamp: float, frame_id: str = "", count: int = 0
    ) -> None:
        self.data = data
        self.stamp = stamp
        self.frame_id = frame_id
        self.count = count

    def clear(self) -> None:
        self.data = None
        self.stamp = None
        self.frame_id = ""
        self.count = 0

    def status(self, now: float) -> str:
        if self.stamp is None:
            return "waiting"
        if self.static_source:
            return "loaded"
        if now - self.stamp > self.stale_timeout_sec:
            return "stale"
        return "active"

    def summary(self, now: float) -> dict[str, Any]:
        return {
            "status": self.status(now),
            "stamp": self.stamp,
            "age": None if self.stamp is None else max(0.0, now - self.stamp),
            "frame_id": self.frame_id,
            "count": self.count,
        }


class TerminalLogBuffer:
    def __init__(self, target: str, max_lines: int = 400) -> None:
        self.target = target
        self.lines: deque[str] = deque(maxlen=max_lines)
        self.enabled = False
        self.running = False
        self.returncode: int | None = None
        self.lock = threading.RLock()

    def append(self, line: str) -> None:
        clean_lines = line.splitlines() or [line]
        with self.lock:
            for clean_line in clean_lines:
                self.lines.append(ANSI_ESCAPE_RE.sub("", clean_line).rstrip("\r\n"))

    def set_process_state(
        self, enabled: bool, running: bool, returncode: int | None
    ) -> None:
        with self.lock:
            self.enabled = enabled
            self.running = running
            self.returncode = returncode

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "target": self.target,
                "enabled": self.enabled,
                "running": self.running,
                "returncode": self.returncode,
                "lines": list(self.lines),
            }


def build_managed_launch_command(
    package: str, launch_file: str, launch_args: str = ""
) -> list[str]:
    return ["ros2", "launch", package, launch_file, *shlex.split(launch_args)]


ANSI_ESCAPE_RE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
