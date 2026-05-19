# Web Control Console Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an offline ROS 2 web console that replaces RViz for setting localization initial pose, showing point cloud map layers, showing current vehicle pose, and publishing navigation goals.

**Architecture:** Add a new `dddmr_web_control` Python package under the navigation workspace. Keep computation-heavy and testable code in `dddmr_web_control/core.py`, keep ROS and HTTP integration in `dddmr_web_control/web_control_node.py`, and serve local static assets from `static/`.

**Tech Stack:** ROS 2 Humble, `rclpy`, `sensor_msgs_py`, Python standard library HTTP server, vanilla HTML/CSS/JavaScript with Canvas/WebGL, `pytest`/`unittest`.

---

## File Structure

- Create `src/dddmr_web_control/package.xml`: package metadata and ROS dependencies.
- Create `src/dddmr_web_control/setup.py`: Python package entry point and asset installation.
- Create `src/dddmr_web_control/setup.cfg`: script install path.
- Create `src/dddmr_web_control/resource/dddmr_web_control`: ament resource marker.
- Create `src/dddmr_web_control/dddmr_web_control/__init__.py`: package marker.
- Create `src/dddmr_web_control/dddmr_web_control/core.py`: testable math, validation, source status, and downsampling utilities.
- Create `src/dddmr_web_control/dddmr_web_control/web_control_node.py`: ROS node, subscriptions, publications, cache, HTTP API, static file server.
- Create `src/dddmr_web_control/static/index.html`: main 2D console UI.
- Create `src/dddmr_web_control/static/styles.css`: local UI styling.
- Create `src/dddmr_web_control/static/app.js`: frontend polling, map canvas, layer toggles, pose and goal interaction, 3D debug view.
- Create `src/dddmr_web_control/launch/web_control.launch.py`: standalone web console launch.
- Create `src/dddmr_web_control/test/test_core.py`: unit tests for core logic.

## Task 1: Core Utilities With Tests

**Files:**
- Create: `src/dddmr_web_control/test/test_core.py`
- Create: `src/dddmr_web_control/dddmr_web_control/core.py`
- Create: `src/dddmr_web_control/dddmr_web_control/__init__.py`

- [ ] **Step 1: Write failing core tests**

Create `test/test_core.py` with tests for quaternion conversion, request validation, point projection/downsampling, and source status:

```python
import math
import time

from dddmr_web_control.core import (
    SourceCache,
    downsample_points,
    project_points_2d,
    quaternion_from_yaw,
    validate_pose_payload,
    yaw_from_quaternion,
)


def test_yaw_quaternion_round_trip():
    q = quaternion_from_yaw(math.pi / 2.0)
    assert q["x"] == 0.0
    assert q["y"] == 0.0
    assert abs(q["z"] - math.sqrt(0.5)) < 1e-9
    assert abs(q["w"] - math.sqrt(0.5)) < 1e-9
    assert abs(yaw_from_quaternion(q) - math.pi / 2.0) < 1e-9


def test_validate_pose_payload_accepts_numbers_and_defaults_z():
    pose = validate_pose_payload({"x": "1.5", "y": 2, "yaw": "-0.25"})
    assert pose == {"x": 1.5, "y": 2.0, "z": 0.0, "yaw": -0.25}


def test_validate_pose_payload_rejects_missing_and_non_finite_values():
    try:
        validate_pose_payload({"x": 1.0, "yaw": 0.0})
    except ValueError as exc:
        assert "missing y" in str(exc)
    else:
        raise AssertionError("missing y was accepted")

    try:
        validate_pose_payload({"x": 1.0, "y": 2.0, "yaw": float("nan")})
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("NaN yaw was accepted")


def test_project_points_2d_and_downsample_points_are_deterministic():
    points = [(float(i), float(i + 1), float(i + 2)) for i in range(10)]
    assert project_points_2d(points)[:3] == [[0.0, 1.0], [1.0, 2.0], [2.0, 3.0]]
    assert downsample_points(points, 4) == [
        (0.0, 1.0, 2.0),
        (3.0, 4.0, 5.0),
        (6.0, 7.0, 8.0),
        (9.0, 10.0, 11.0),
    ]


def test_source_cache_status_transitions():
    cache = SourceCache("map", stale_timeout_sec=1.0)
    assert cache.status(now=10.0) == "waiting"

    cache.update({"points": []}, stamp=10.0)
    assert cache.status(now=10.5) == "active"
    assert cache.status(now=11.5) == "stale"
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/robot/nav26_ws
python3 -m pytest src/dddmr_navigation-for-Ackerman/src/dddmr_web_control/test/test_core.py -q
```

Expected: fail with `ModuleNotFoundError` because `dddmr_web_control.core` does not exist.

- [ ] **Step 3: Implement core utilities**

Create `dddmr_web_control/core.py`:

```python
import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional


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


def downsample_points(points: list[tuple[float, float, float]], max_points: int) -> list[tuple[float, float, float]]:
    if max_points <= 0 or len(points) <= max_points:
        return list(points)
    step = max(1, math.ceil(len(points) / max_points))
    sampled = points[::step]
    if sampled[-1] != points[-1] and len(sampled) < max_points:
        sampled.append(points[-1])
    return sampled[:max_points]


def project_points_2d(points: Iterable[tuple[float, float, float]]) -> list[list[float]]:
    return [[float(x), float(y)] for x, y, _z in points]


@dataclass
class SourceCache:
    name: str
    stale_timeout_sec: float
    data: Optional[Any] = None
    stamp: Optional[float] = None
    frame_id: str = ""
    count: int = 0

    def update(self, data: Any, stamp: float, frame_id: str = "", count: int = 0) -> None:
        self.data = data
        self.stamp = stamp
        self.frame_id = frame_id
        self.count = count

    def status(self, now: float) -> str:
        if self.stamp is None:
            return "waiting"
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
```

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```bash
cd /home/robot/nav26_ws
PYTHONPATH=src/dddmr_navigation-for-Ackerman/src/dddmr_web_control python3 -m pytest src/dddmr_navigation-for-Ackerman/src/dddmr_web_control/test/test_core.py -q
```

Expected: all tests pass.

## Task 2: Package Skeleton, ROS Node, and HTTP API

**Files:**
- Create: `src/dddmr_web_control/package.xml`
- Create: `src/dddmr_web_control/setup.py`
- Create: `src/dddmr_web_control/setup.cfg`
- Create: `src/dddmr_web_control/resource/dddmr_web_control`
- Create: `src/dddmr_web_control/dddmr_web_control/web_control_node.py`
- Modify: `src/dddmr_web_control/test/test_core.py`

- [ ] **Step 1: Add failing tests for API helpers**

Append tests that exercise JSON response construction and layer name validation:

```python
from dddmr_web_control.web_control_node import normalize_layer_name, success_response


def test_normalize_layer_name_rejects_unknown_layer():
    assert normalize_layer_name("global_map") == "global_map"
    try:
        normalize_layer_name("../bad")
    except ValueError as exc:
        assert "unknown layer" in str(exc)
    else:
        raise AssertionError("unknown layer was accepted")


def test_success_response_has_expected_shape():
    response = success_response("published", stamp=12.5)
    assert response == {"ok": True, "message": "published", "stamp": 12.5}
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cd /home/robot/nav26_ws
PYTHONPATH=src/dddmr_navigation-for-Ackerman/src/dddmr_web_control python3 -m pytest src/dddmr_navigation-for-Ackerman/src/dddmr_web_control/test/test_core.py -q
```

Expected: fail with `ModuleNotFoundError` or missing functions for `web_control_node`.

- [ ] **Step 3: Implement package and ROS/HTTP node**

Implement `web_control_node.py` with:

- `normalize_layer_name(name)` accepting `global_map`, `mapcloud`, `mapground`.
- `success_response(message, stamp)` returning `{"ok": True, "message": message, "stamp": stamp}`.
- `WebControlNode` with parameters from the spec.
- PointCloud2 callbacks that cache downsampled 2D/3D points per layer.
- Odometry callback that caches x, y, z, yaw.
- Dynamic path subscriptions only for `path_topics`; default empty.
- Publishers for `/initialpose` and `goal_pose_3d`.
- HTTP endpoints for `/api/state`, `/api/map2d`, `/api/map3d`, `/api/path`, `/api/initialpose`, and `/api/goal`.
- Static file serving from the installed `static` directory.

Implement package metadata with `console_scripts = web_control_node = dddmr_web_control.web_control_node:main`.

- [ ] **Step 4: Run unit tests and verify GREEN**

Run:

```bash
cd /home/robot/nav26_ws
PYTHONPATH=src/dddmr_navigation-for-Ackerman/src/dddmr_web_control python3 -m pytest src/dddmr_navigation-for-Ackerman/src/dddmr_web_control/test/test_core.py -q
```

Expected: all tests pass.

## Task 3: Offline Frontend

**Files:**
- Create: `src/dddmr_web_control/static/index.html`
- Create: `src/dddmr_web_control/static/styles.css`
- Create: `src/dddmr_web_control/static/app.js`

- [ ] **Step 1: Add static frontend files**

Create a dense operational UI:

- Top-level tabs for `2D Console` and `3D Debug`.
- Main 2D canvas with pan, zoom, current vehicle marker, selected pose preview, and visible layer rendering.
- Right panel with mode buttons, layer checkboxes, status fields, current pose, and publish result fields.
- JavaScript polling `/api/state`, `/api/map2d`, and `/api/path`.
- POST `/api/initialpose` and `/api/goal` from selected map coordinates.
- 3D debug view using local Canvas/WebGL code only.

- [ ] **Step 2: Run a static asset sanity check**

Run:

```bash
cd /home/robot/nav26_ws/src/dddmr_navigation-for-Ackerman/src/dddmr_web_control
python3 -m py_compile dddmr_web_control/web_control_node.py
test -s static/index.html
test -s static/styles.css
test -s static/app.js
```

Expected: exit code 0.

## Task 4: Launch, Build, and Verification

**Files:**
- Create: `src/dddmr_web_control/launch/web_control.launch.py`

- [ ] **Step 1: Add standalone launch file**

Create a launch file with arguments for host, port, topic names, point limits,
stale timeout, and `path_topics`. It starts only `dddmr_web_control`
`web_control_node`.

- [ ] **Step 2: Run package tests**

Run:

```bash
cd /home/robot/nav26_ws
PYTHONPATH=src/dddmr_navigation-for-Ackerman/src/dddmr_web_control python3 -m pytest src/dddmr_navigation-for-Ackerman/src/dddmr_web_control/test/test_core.py -q
```

Expected: all tests pass.

- [ ] **Step 3: Build the package**

Run:

```bash
cd /home/robot/nav26_ws
colcon build --packages-select dddmr_web_control --symlink-install
```

Expected: package builds successfully.

- [ ] **Step 4: Smoke launch the web console**

Run:

```bash
cd /home/robot/nav26_ws
source install/setup.bash
timeout 5s ros2 launch dddmr_web_control web_control.launch.py port:=18080
```

Expected: process starts the web server and exits only because `timeout` stops it.

- [ ] **Step 5: Review git status**

Run:

```bash
git -C /home/robot/nav26_ws/src/dddmr_navigation-for-Ackerman status -s
```

Expected: new `src/dddmr_web_control` files and plan document are present, while pre-existing unrelated modified files remain untouched.
