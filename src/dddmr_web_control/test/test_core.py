import math
import signal
import subprocess
from pathlib import Path

from dddmr_web_control.core import (
    SourceCache,
    TerminalLogBuffer,
    ViewerRegistry,
    build_managed_launch_command,
    build_system_usage_payload,
    calculate_cpu_usage,
    downsample_points,
    parse_proc_meminfo,
    parse_proc_stat_cpu_ticks,
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


def test_static_source_cache_stays_loaded_after_first_message():
    cache = SourceCache("mapground", stale_timeout_sec=1.0, static_source=True)
    assert cache.status(now=10.0) == "waiting"

    cache.update({"points": []}, stamp=10.0)
    assert cache.status(now=10.5) == "loaded"
    assert cache.status(now=30.0) == "loaded"


def test_source_cache_clear_resets_static_source_to_waiting():
    cache = SourceCache("mapground", stale_timeout_sec=1.0, static_source=True)
    cache.update({"points3d": [[1.0, 2.0, 3.0]]}, stamp=10.0, frame_id="map", count=1)

    cache.clear()

    assert cache.status(now=30.0) == "waiting"
    assert cache.data is None
    assert cache.stamp is None
    assert cache.frame_id == ""
    assert cache.count == 0


def test_viewer_registry_counts_recent_heartbeats_and_expires_old_sessions():
    viewers = ViewerRegistry(timeout_sec=10.0)

    assert viewers.count(now=100.0) == 0
    assert viewers.heartbeat("tab-a", now=100.0) == 1
    assert viewers.heartbeat("tab-b", now=104.0) == 2
    assert viewers.count(now=109.9) == 2
    assert viewers.count(now=111.0) == 1


def test_navigation_readiness_waits_for_relocalization_then_navigation():
    from dddmr_web_control.web_control_node import navigation_readiness

    assert navigation_readiness("waiting", "waiting") == {
        "ready": False,
        "status": "waiting_for_relocalization",
        "message": "waiting for relocalization",
    }
    assert navigation_readiness("active", "waiting") == {
        "ready": False,
        "status": "waiting_for_navigation",
        "message": "waiting for navigation",
    }
    assert navigation_readiness("active", "loaded") == {
        "ready": True,
        "status": "ready",
        "message": "ready",
    }


def test_normalize_layer_name_rejects_unknown_layer():
    from dddmr_web_control.web_control_node import normalize_layer_name

    assert normalize_layer_name("global_map") == "global_map"
    try:
        normalize_layer_name("../bad")
    except ValueError as exc:
        assert "unknown layer" in str(exc)
    else:
        raise AssertionError("unknown layer was accepted")


def test_success_response_has_expected_shape():
    from dddmr_web_control.web_control_node import success_response

    response = success_response("published", stamp=12.5)
    assert response == {"ok": True, "message": "published", "stamp": 12.5}


def test_action_status_name_uses_latest_status():
    from dddmr_web_control.web_control_node import action_status_name

    assert action_status_name([]) == "idle"
    assert action_status_name([1]) == "accepted"
    assert action_status_name([2]) == "executing"
    assert action_status_name([4]) == "succeeded"
    assert action_status_name([6]) == "aborted"
    assert action_status_name([4, 2]) == "executing"


def test_view_mode_clears_transient_selection_marker():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'if (mode === "view")' in app_js
    assert "state.selection = null;" in app_js


def test_frontend_uses_threejs_3d_map_without_2d_tab():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    setup_py = (Path(__file__).resolve().parents[1] / "setup.py").read_text(encoding="utf-8")

    assert 'src="/app.js"' in index_html
    assert 'type="module"' in index_html
    assert 'id="tab2d"' not in index_html
    assert 'id="view2d"' not in index_html
    assert 'id="tab3d"' not in index_html
    assert 'id="tab3dMap"' in index_html
    assert ">Navigation</button>" in index_html
    assert 'data-active="3d"' in index_html
    assert 'id="mapCanvas3d"' in index_html
    assert 'import * as THREE from "./vendor/three.module.js";' in app_js
    assert 'view: "3d"' in app_js
    assert 'static/vendor' in setup_py


def test_frontend_exposes_mapping_tab_and_mapping_sidebar():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="tabMapping"' in index_html
    assert "Mapping" in index_html
    assert 'id="mappingSidebar"' in index_html
    assert 'id="navigationSidebar"' in index_html
    assert 'data-panel="navigation"' in index_html
    assert 'data-panel="mapping"' in index_html
    assert 'id="mappingPoseX"' in index_html
    assert 'data-mapping-layer="global_map"' in index_html
    assert 'data-mapping-layer="ground"' in index_html
    assert 'id="mappingPathLayer"' in index_html
    assert 'id="mappingVehicleLayer"' in index_html
    assert 'id="mappingSources"' in index_html
    assert 'document.getElementById("tabMapping").classList.toggle("active", tab === "mapping");' in app_js
    assert 'document.querySelectorAll("[data-panel]")' in app_js
    assert 'panel.classList.toggle("active", panel.dataset.panel === panelName)' in app_js
    assert ".panel-view" in styles_css
    assert ".panel-view.active" in styles_css


def test_frontend_polls_mapping_3d_sources_only_for_mapping_tab():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "const MAPPING_LAYERS = {" in app_js
    assert 'global_map: { color: 0x8ea4b8' in app_js
    assert 'ground: { color: 0xd9a441' in app_js
    assert "const mappingLayerGroup = new THREE.Group();" in app_js
    assert "const mappingPathGroup = new THREE.Group();" in app_js
    assert "const mappingVehicleGroup = new THREE.Group();" in app_js
    assert 'getJson(`/api/mapping/map3d?layer=${encodeURIComponent(name)}`)' in app_js
    assert 'getJson("/api/mapping/path")' in app_js
    assert 'if (state.view !== "mapping") return;' in app_js
    assert "setInterval(() => pollMapping3d(false), 5000)" in app_js


def test_web_control_subscribes_mapping_topics_and_exposes_mapping_api():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("mapping_global_map_topic", "/lio_sam/mapping/map_global")' in web_control_node
    assert 'self.declare_parameter("mapping_ground_topic", "/lio_sam/mapping/ground_cloud_global")' in web_control_node
    assert 'self.declare_parameter("mapping_path_topic", "/lio_sam/mapping/path")' in web_control_node
    assert 'self.declare_parameter("mapping_odom_topic", "/lio_sam/mapping/odometry")' in web_control_node
    assert "self.mapping_layers = {" in web_control_node
    assert "self.mapping_path = SourceCache" in web_control_node
    assert "self.mapping_odom = SourceCache" in web_control_node
    assert "self.mapping_pointcloud_callback" in web_control_node
    assert "self.mapping_path_callback" in web_control_node
    assert "self.mapping_odom_callback" in web_control_node
    assert "def mapping_state_payload(self)" in web_control_node
    assert "def mapping_layer_payload(self, layer_name: str)" in web_control_node
    assert "def mapping_path_payload(self)" in web_control_node
    assert 'elif parsed.path == "/api/mapping/state":' in web_control_node
    assert 'elif parsed.path == "/api/mapping/map3d":' in web_control_node
    assert 'elif parsed.path == "/api/mapping/path":' in web_control_node


def test_web_control_exposes_lightweight_pose_endpoint_for_realtime_vehicle_updates():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "def pose_payload(self)" in web_control_node
    assert '"mapping_odom": {' in web_control_node
    assert 'elif parsed.path == "/api/pose":' in web_control_node
    assert "node.pose_payload()" in web_control_node


def test_frontend_polls_lightweight_pose_endpoint_for_realtime_vehicle_markers():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'getJson("/api/pose")' in app_js
    assert "function pollPose()" in app_js
    assert "function updateLivePose(data)" in app_js
    assert "function updateNavigationPose(odom)" in app_js
    assert "function updateMappingPose(odom)" in app_js
    assert "setInterval(pollPose, 100)" in app_js

    navigation_start = app_js.index("function updateNavigationPose(odom)")
    navigation_block = app_js[
        navigation_start : app_js.index("function updateMappingPose", navigation_start)
    ]
    mapping_start = app_js.index("function updateMappingPose(odom)")
    mapping_block = app_js[
        mapping_start : app_js.index("function updateLivePose", mapping_start)
    ]
    assert "updateVehicleMarker();" in navigation_block
    assert "updateMappingVehicleMarker();" in mapping_block


def test_web_control_uses_compatible_qos_profiles_for_visualization_topics():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "pointcloud_qos = QoSProfile(" in web_control_node
    assert "reliability=QoSReliabilityPolicy.BEST_EFFORT" in web_control_node
    assert "durability=QoSDurabilityPolicy.VOLATILE" in web_control_node
    assert "state_qos = QoSProfile(" in web_control_node
    assert "reliability=QoSReliabilityPolicy.RELIABLE" in web_control_node
    assert "PointCloud2," in web_control_node
    assert "pointcloud_qos," in web_control_node
    assert "Odometry," in web_control_node
    assert "PathMsg," in web_control_node
    assert "GoalStatusArray," in web_control_node
    assert "state_qos," in web_control_node
    assert "TRANSIENT_LOCAL" not in web_control_node


def test_3d_target_selection_uses_shared_map_coordinate_frame():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="targetLayer"' not in index_html
    assert 'id="selLayer"' not in index_html
    assert 'id="goalLayer"' not in index_html
    assert "Target layer" not in index_html
    assert "targetLayer" not in app_js
    assert "layer: state.targetLayer" not in app_js
    assert "LAYER_TARGET_STYLES" not in app_js
    assert "updateTargetMarkers()" in app_js


def test_mode_specific_3d_selection_controls_goal_z_handling():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="selZ"' in index_html
    assert "function pointCloudIntersection(event)" in app_js
    assert "LAYERS.mapground.object" in app_js
    assert "raycaster.intersectObject(LAYERS.mapground.object, false)" in app_js
    assert 'if (state.mode === "initial") return groundIntersection(event);' in app_js
    assert 'if (state.mode === "goal") return pointCloudIntersection(event);' in app_js
    assert "state.selection = { x: pose.x, y: pose.y, z: pose.z, yaw };" in app_js
    assert "function selectedPosePayload(includeZ)" in app_js
    assert "z: includeZ ? state.selection.z : 0" in app_js
    assert 'postJson("/api/initialpose", selectedPosePayload(false))' in app_js
    assert "const payload = selectedPosePayload(true);" in app_js
    assert "group.position.copy(rosToThree(pose.x, pose.y, (pose.z || 0) + 0.1));" in app_js


def test_selection_publish_button_follows_mode():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'id="publishPose"' in index_html
    assert "Publish Pose" in index_html
    assert "publishInitial" not in index_html
    assert "publishGoal" not in index_html
    assert "function updatePublishPoseButton()" in app_js
    assert 'button.disabled = state.mode === "view";' in app_js
    assert 'button.textContent = "Publish Pose";' in app_js
    assert "async function publishPose()" in app_js
    assert "const previousMode = state.mode;" in app_js
    assert "if (previousMode !== mode) {" in app_js
    assert 'if (state.mode === "initial") {' in app_js
    assert 'postJson("/api/initialpose", selectedPosePayload(false))' in app_js
    assert 'if (state.mode === "goal") {' in app_js
    assert 'postJson("/api/goal", payload)' in app_js
    assert 'document.getElementById("publishPose").addEventListener("click", publishPose);' in app_js
    assert "publishInitialPose" not in app_js
    assert "publishGoal()" not in app_js
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")
    assert "grid-template-columns: 1fr;" in styles_css
    assert ".actions button:disabled" in styles_css
    assert "cursor: not-allowed;" in styles_css


def test_3d_pose_markers_do_not_render_text_labels():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function makeArrowMarker(pose, color)" in app_js
    assert "makeTextSprite" not in app_js
    assert "CanvasTexture" not in app_js
    assert "makeArrowMarker(state.selection, TARGET_MARKER_COLOR)" in app_js
    assert "makeArrowMarker(goal, TARGET_MARKER_COLOR)" in app_js
    assert 'makeArrowMarker({ ...state.odom.pose, layer: "mapcloud" }, 0x32d74b)' in app_js


def test_goal_arrival_clears_goal_marker_and_paths_until_new_goal():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")
    clear_start = app_js.index("function maybeClearGoalFromActionStatus()")
    clear_block = app_js[clear_start : app_js.index("function setMode", clear_start)]
    poll_start = app_js.index("async function pollPaths")
    poll_block = app_js[poll_start : app_js.index("function updateStatus", poll_start)]
    publish_start = app_js.index("async function publishPose()")
    publish_block = app_js[publish_start : app_js.index("async function stopManagedTarget", publish_start)]

    assert "hideCompletedPaths: false" in app_js
    assert "state.activeGoal = null;" in clear_block
    assert "state.lastGoal = null;" in clear_block
    assert "state.paths = {};" in clear_block
    assert "state.hideCompletedPaths = true;" in clear_block
    assert "updatePaths();" in clear_block
    assert "if (state.hideCompletedPaths) return;" in poll_block
    assert "state.hideCompletedPaths = false;" in publish_block


def test_3d_paths_use_tube_geometry_for_thicker_visible_tracks():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")
    update_start = app_js.index("function updatePaths()")
    update_block = app_js[update_start : app_js.index("async function pollState", update_start)]

    assert "new THREE.CatmullRomCurve3(points)" in update_block
    assert "new THREE.TubeGeometry(pathCurve, Math.max(2, points.length * 2), 0.16, 8, false)" in update_block
    assert "new THREE.MeshBasicMaterial" in update_block
    assert "new THREE.LineBasicMaterial" not in update_block


def test_frontend_exposes_managed_start_controls():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")

    assert 'data-start-target="localization"' in index_html
    assert 'data-start-target="navigation"' in index_html
    assert 'data-start-target="mapping"' in index_html
    assert 'id="mappingLogState"' in index_html
    assert 'id="mappingLogs"' in index_html
    assert "/api/managed/start" in app_js
    assert 'updateTerminalLog("mapping", data.logs && data.logs.mapping);' in app_js


def test_terminal_logs_section_is_above_layers_section():
    index_html = (
        Path(__file__).resolve().parents[1] / "static" / "index.html"
    ).read_text(encoding="utf-8")

    assert index_html.index("<h2>Terminal Logs</h2>") < index_html.index("<h2>Layers</h2>")


def test_terminal_logs_are_text_selectable():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".terminal pre" in styles_css
    assert "user-select: text;" in styles_css
    assert "-webkit-user-select: text;" in styles_css


def test_web_launch_does_not_expose_auto_start_arguments():
    launch_py = (
        Path(__file__).resolve().parents[1] / "launch" / "web_control.launch.py"
    ).read_text(encoding="utf-8")

    assert "start_localization" not in launch_py
    assert "start_navigation" not in launch_py


def test_web_launch_exposes_single_use_sim_parameter_for_managed_launches():
    launch_py = (
        Path(__file__).resolve().parents[1] / "launch" / "web_control.launch.py"
    ).read_text(encoding="utf-8")
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'use_sim = LaunchConfiguration("use_sim")' in launch_py
    assert 'DeclareLaunchArgument("use_sim", default_value="false")' in launch_py
    assert '"use_sim": use_sim' in launch_py
    assert 'default_value="use_sim:=true"' not in launch_py
    assert 'self.declare_parameter("use_sim", False)' in web_control_node
    assert "self.use_sim = parse_bool_parameter(self.get_parameter(\"use_sim\").value)" in web_control_node
    assert "def parse_bool_parameter(value: Any) -> bool:" in web_control_node
    assert "def managed_launch_args(self, extra_args: str) -> str:" in web_control_node
    assert 'use_sim_arg = f"use_sim:={str(self.use_sim).lower()}"' in web_control_node
    assert 'self.managed_launch_args(str(self.get_parameter("localization_launch_args").value))' in web_control_node
    assert 'self.managed_launch_args(str(self.get_parameter("navigation_launch_args").value))' in web_control_node
    assert 'self.managed_launch_args(str(self.get_parameter("mapping_launch_args").value))' in web_control_node


def test_terminal_log_buffer_keeps_recent_lines_and_metadata():
    logs = TerminalLogBuffer("localization", max_lines=2)
    logs.append("first")
    logs.append("second\n")
    logs.append("third")
    logs.set_process_state(enabled=True, running=False, returncode=3)

    assert logs.snapshot() == {
        "target": "localization",
        "enabled": True,
        "running": False,
        "returncode": 3,
        "lines": ["second", "third"],
    }


def test_terminal_log_buffer_strips_ansi_escape_sequences():
    logs = TerminalLogBuffer("navigation", max_lines=3)
    logs.append("\x1b[0m[INFO] ready\x1b[0m")
    logs.append("\x1b[37;1mcolored\x1b[31;1m error\x1b[m")

    assert logs.snapshot()["lines"] == ["[INFO] ready", "colored error"]


def test_managed_launch_commands_are_fixed_ros_launch_invocations():
    assert build_managed_launch_command(
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_rviz:=false debug_relocalization:=false",
    ) == [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "localization_with_nonground.launch.py",
        "use_rviz:=false",
        "debug_relocalization:=false",
    ]
    assert build_managed_launch_command(
        "lio_sam_hesai",
        "mapping.launch.py",
        "use_rviz:=false",
    ) == [
        "ros2",
        "launch",
        "lio_sam_hesai",
        "mapping.launch.py",
        "use_rviz:=false",
    ]


def test_web_control_manages_mapping_launch_and_clears_mapping_runtime_state():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("mapping_launch_args", "use_rviz:=false")' in web_control_node
    assert '"mapping": TerminalLogBuffer(' in web_control_node
    assert "mapping_command = build_managed_launch_command(" in web_control_node
    assert '"mapping.launch.py"' in web_control_node
    assert 'self.managed_processes["mapping"] = ManagedLaunchProcess(' in web_control_node
    assert "def clear_mapping_runtime_state(self)" in web_control_node
    assert "for cache in self.mapping_layers.values()" in web_control_node
    assert "self.mapping_odom.clear()" in web_control_node
    assert "self.mapping_path.clear()" in web_control_node
    assert 'elif target == "mapping":' in web_control_node
    assert "self.clear_mapping_runtime_state()" in web_control_node


def test_managed_launch_stop_signals_saved_group_after_leader_exits(monkeypatch):
    from dddmr_web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class ExitedProcess:
        pid = 123
        returncode = 0

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

    calls = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: calls.append((pgid, sig)))

    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = ExitedProcess()
    managed.pgid = 456

    managed.stop()

    assert calls == [(456, signal.SIGINT)]


def test_managed_launch_stop_uses_short_shutdown_timeouts(monkeypatch):
    from dddmr_web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class StubbornProcess:
        pid = 123
        returncode = None

        def __init__(self):
            self.wait_timeouts = []
            self.killed = False

        def poll(self):
            return None

        def wait(self, timeout=None):
            self.wait_timeouts.append(timeout)
            raise subprocess.TimeoutExpired("managed", timeout)

        def kill(self):
            self.killed = True

        def send_signal(self, _sig):
            pass

    monkeypatch.setattr("os.killpg", lambda _pgid, _sig: None)
    process = StubbornProcess()
    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = process
    managed.pgid = 456

    managed.stop()

    assert process.wait_timeouts == [1.5, 0.75, 0.25]
    assert process.killed is True


def test_managed_launch_stop_signals_descendant_processes(monkeypatch):
    from dddmr_web_control.web_control_node import ManagedLaunchProcess

    class Logger:
        def error(self, _message):
            pass

    class ExitedProcess:
        pid = 123
        returncode = 0

        def poll(self):
            return 0

        def wait(self, timeout=None):
            return 0

    group_calls = []
    pid_calls = []
    monkeypatch.setattr("os.killpg", lambda pgid, sig: group_calls.append((pgid, sig)))
    monkeypatch.setattr("os.kill", lambda pid, sig: pid_calls.append((pid, sig)))

    managed = ManagedLaunchProcess(
        "navigation",
        ["ros2", "launch", "p2p_move_base", "p2p_move_base_localization.launch.py"],
        TerminalLogBuffer("navigation"),
        Logger(),
    )
    managed.process = ExitedProcess()
    managed.pgid = 456
    managed.descendant_pids = {777, 888}

    managed.stop()

    assert group_calls == [(456, signal.SIGINT)]
    assert pid_calls == [(777, signal.SIGINT), (888, signal.SIGINT)]


def test_web_control_shutdown_uses_destroy_node_for_managed_launch_cleanup():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "signal.signal(signal.SIGINT" not in web_control_node
    assert "signal.signal(signal.SIGTERM" not in web_control_node
    assert "def destroy_node(self) -> bool:" in web_control_node
    assert "self.stop_managed_processes()" in web_control_node
    assert "self.stop_http_server()" in web_control_node


def test_web_control_stops_managed_launches_concurrently_on_shutdown():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'name=f"managed_launch_stop_' in web_control_node
    assert "threading.Thread(" in web_control_node
    assert "thread.join(timeout=" in web_control_node


def test_managed_launch_exit_clears_runtime_map_sources():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "def clear_map_runtime_state(self)" in web_control_node
    assert "for cache in self.layers.values()" in web_control_node
    assert "self.odom.clear()" in web_control_node
    assert "self.action_status.clear()" in web_control_node
    assert "def handle_managed_exit(self, target: str)" in web_control_node
    assert 'if target == "localization":' in web_control_node
    assert "self.clear_map_runtime_state()" in web_control_node
    assert "on_exit=self.handle_managed_exit" in web_control_node


def test_parse_proc_stat_cpu_ticks_reads_total_and_cores():
    stat_text = "\n".join(
        [
            "cpu  100 0 50 850 0 0 0 0 0 0",
            "cpu0 40 0 10 450 0 0 0 0 0 0",
            "cpu1 60 0 40 400 0 0 0 0 0 0",
            "intr 1 2 3",
        ]
    )

    ticks = parse_proc_stat_cpu_ticks(stat_text)

    assert ticks["total"] == (100, 0, 50, 850, 0, 0, 0, 0, 0, 0)
    assert ticks["cores"] == [
        (40, 0, 10, 450, 0, 0, 0, 0, 0, 0),
        (60, 0, 40, 400, 0, 0, 0, 0, 0, 0),
    ]


def test_calculate_cpu_usage_returns_percent_from_tick_delta():
    previous = {
        "total": (100, 0, 50, 850, 0, 0, 0, 0, 0, 0),
        "cores": [
            (40, 0, 10, 450, 0, 0, 0, 0, 0, 0),
            (60, 0, 40, 400, 0, 0, 0, 0, 0, 0),
        ],
    }
    current = {
        "total": (130, 0, 70, 900, 0, 0, 0, 0, 0, 0),
        "cores": [
            (50, 0, 15, 485, 0, 0, 0, 0, 0, 0),
            (80, 0, 55, 415, 0, 0, 0, 0, 0, 0),
        ],
    }

    usage = calculate_cpu_usage(previous, current)

    assert usage == {
        "total_percent": 50.0,
        "cores": [
            {"id": 0, "percent": 30.0},
            {"id": 1, "percent": 70.0},
        ],
    }


def test_parse_proc_meminfo_reports_memory_and_swap_usage():
    meminfo = "\n".join(
        [
            "MemTotal:        8000000 kB",
            "MemFree:         1000000 kB",
            "MemAvailable:    2500000 kB",
            "SwapTotal:       2000000 kB",
            "SwapFree:        1500000 kB",
        ]
    )

    usage = parse_proc_meminfo(meminfo)

    assert usage["memory"] == {
        "total_kb": 8000000,
        "used_kb": 5500000,
        "percent": 68.75,
    }
    assert usage["swap"] == {
        "total_kb": 2000000,
        "used_kb": 500000,
        "percent": 25.0,
    }


def test_build_system_usage_payload_is_lightweight_and_handles_first_cpu_sample():
    previous = parse_proc_stat_cpu_ticks("cpu  100 0 0 900\ncpu0 100 0 0 900")
    current = parse_proc_stat_cpu_ticks("cpu  150 0 0 950\ncpu0 150 0 0 950")
    meminfo = "MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n"

    first_payload, first_ticks = build_system_usage_payload(
        "cpu  100 0 0 900\ncpu0 100 0 0 900",
        meminfo,
        previous_ticks=None,
        stamp=12.0,
    )
    second_payload, second_ticks = build_system_usage_payload(
        "cpu  150 0 0 950\ncpu0 150 0 0 950",
        meminfo,
        previous_ticks=previous,
        stamp=13.0,
    )

    assert first_payload["cpu"] == {
        "total_percent": None,
        "cores": [{"id": 0, "percent": None}],
    }
    assert first_ticks == previous
    assert second_payload["cpu"] == {
        "total_percent": 50.0,
        "cores": [{"id": 0, "percent": 50.0}],
    }
    assert second_ticks == current


def test_web_control_node_imports_system_usage_payload_builder():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "build_system_usage_payload" in web_control_node
    assert "self.system_cpu_ticks" in web_control_node


def test_web_control_get_router_exposes_system_endpoint():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'parsed.path == "/api/system"' in web_control_node
    assert "node.system_payload()" in web_control_node


def test_http_handler_ignores_client_disconnects_without_writing_error_response():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "CLIENT_DISCONNECT_ERRORS" in web_control_node
    assert "except CLIENT_DISCONNECT_ERRORS:" in web_control_node
    assert "return" in web_control_node
    assert "BrokenPipeError" in web_control_node
    assert "ConnectionResetError" in web_control_node


def test_frontend_exposes_system_tab_and_metric_targets():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")

    assert 'id="tabSystem"' in index_html
    assert 'id="viewSystem"' in index_html
    assert 'id="systemCpuTotal"' in index_html
    assert 'id="systemMemory"' in index_html
    assert 'id="systemSwap"' in index_html
    assert 'id="systemCpuCores"' in index_html


def test_frontend_exposes_current_viewer_count():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="viewerCount"' in index_html
    assert "function viewerSessionId()" in app_js
    assert 'postJson("/api/viewer", { viewer_id: viewerSessionId() })' in app_js
    assert "function updateViewerCount(count)" in app_js
    assert "updateViewerCount(data.viewers)" in app_js
    assert "setInterval(heartbeatViewer, 3000)" in app_js
    assert ".header-status" in styles_css


def test_web_control_exposes_viewer_heartbeat_endpoint_and_state_count():
    web_control_node = (
        Path(__file__).resolve().parents[1]
        / "dddmr_web_control"
        / "web_control_node.py"
    ).read_text(encoding="utf-8")

    assert "self.viewers = ViewerRegistry" in web_control_node
    assert '"viewers": self.viewers.count(now)' in web_control_node
    assert "def viewer_heartbeat(self, payload: dict[str, Any])" in web_control_node
    assert 'elif self.path == "/api/viewer":' in web_control_node


def test_left_drag_rotates_3d_view_in_natural_follow_direction():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "state.camera.yaw += dx * 0.006;" in app_js
    assert "state.camera.pitch = Math.max(0.18, Math.min(1.38, state.camera.pitch + dy * 0.006));" in app_js
    assert "state.camera.yaw -= dx * 0.006;" not in app_js
    assert "state.camera.pitch - dy * 0.006" not in app_js


def test_3d_map_supports_touch_pointer_interaction():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "touches: new Map()" in app_js
    assert "function touchGesture()" in app_js
    assert "function handleTouchPanZoom(event)" in app_js
    assert "state.pointer.touches.set(event.pointerId, canvasPoint(event));" in app_js
    assert "state.pointer.touches.delete(event.pointerId);" in app_js
    assert "if (event.pointerType === \"touch\" && state.pointer.touches.size >= 2)" in app_js
    assert "state.camera.distance = Math.max(5, Math.min(280, state.pointer.touchStart.cameraDistance * zoomRatio));" in app_js
    assert "state.camera.target.addScaledVector(right, -dx * scale);" in app_js
    assert "mapCanvas3d.addEventListener(\"pointerleave\", endPointer);" in app_js


def test_frontend_polls_system_api_only_for_system_tab():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert 'getJson("/api/system")' in app_js
    assert 'if (state.view !== "system") return;' in app_js
    assert 'setInterval(pollSystem, 1000)' in app_js
    assert 'setTab("system")' in app_js


def test_system_tab_hides_sidebar_and_expands_stage():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    set_tab_start = app_js.index("function setTab(tab)")
    set_tab_block = app_js[set_tab_start : app_js.index("function selectedPosePayload", set_tab_start)]

    assert 'document.querySelector(".app").classList.toggle("system-mode", tab === "system");' in set_tab_block
    assert 'const panelName = tab === "system" ? "" : (tab === "mapping" ? "mapping" : "navigation");' in set_tab_block
    assert "panel.dataset.panel === panelName" in set_tab_block
    assert ".app.system-mode" in styles_css
    assert ".app.system-mode .panel" in styles_css
    system_panel_start = styles_css.index(".app.system-mode .panel")
    system_panel_block = styles_css[system_panel_start : styles_css.index("}", system_panel_start)]
    assert "display: none;" in system_panel_block


def test_cpu_core_rows_are_reused_so_bars_slide_to_targets():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function updateCpuCores(cores)" in app_js
    assert 'list.querySelector(`[data-core-id="${core.id}"]`)' in app_js
    assert "row.dataset.coreId = String(core.id);" in app_js
    assert 'list.innerHTML = "";' not in app_js


def test_frontend_clears_3d_scene_when_map_sources_reset():
    app_js = (
        Path(__file__).resolve().parents[1] / "static" / "app.js"
    ).read_text(encoding="utf-8")

    assert "function clearMapSceneState()" in app_js
    assert "function mapSourcesAreWaiting(layers)" in app_js
    assert "if (mapSourcesAreWaiting(data.layers || {}))" in app_js
    assert "clearMapSceneState();" in app_js
    assert "Object.keys(LAYERS).forEach((name) =>" in app_js
    assert "state.paths = {};" in app_js
    assert "state.odom = null;" in app_js
    assert 'if (target === "all" || target === "localization")' in app_js


def test_frontend_styles_cpu_core_horizontal_bars():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".cpu-core-row" in styles_css
    assert ".cpu-core-fill" in styles_css
    assert "width: var(--cpu-percent" in styles_css


def test_system_tab_uses_refined_dashboard_elements():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert "system-card cpu-card" in index_html
    assert "system-card memory-card" in index_html
    assert "system-card swap-card" in index_html
    assert "system-meter-fill" in index_html
    assert "system-card-status" in index_html
    assert "usageLevel(" in app_js
    assert 'row.className = `cpu-core-row load-${usageLevel(core.percent)}`;' in app_js
    assert ".system-card::before" in styles_css
    assert "transition: width 420ms ease" in styles_css
    assert "@keyframes meterGlow" in styles_css


def test_frontend_uses_ios_inspired_visual_theme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-blue: #0a84ff" in styles_css
    assert "--glass:" in styles_css
    assert "--radius-lg: 22px" in styles_css
    assert "backdrop-filter: blur(22px) saturate(1.35)" in styles_css
    assert ".panel section" in styles_css
    assert "border-radius: var(--radius-md)" in styles_css
    assert ".tab.active" in styles_css
    assert ".hud" in styles_css


def test_mode_control_uses_ios_sliding_segmented_selector():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="modeSegment"' in index_html
    assert 'data-active="view"' in index_html
    assert 'document.getElementById("modeSegment").dataset.active = mode;' in app_js
    assert ".segmented::before" in styles_css
    assert '.segmented[data-active="initial"] {' in styles_css
    assert '.segmented[data-active="goal"] {' in styles_css
    assert "--mode-index: 2;" in styles_css


def test_frontend_uses_immersive_glass_light_environment():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "body::before" in styles_css
    assert "body::after" in styles_css
    assert "@keyframes ambientDrift" in styles_css
    assert "@keyframes sheenSweep" in styles_css
    assert "mix-blend-mode: screen" in styles_css
    assert "background-attachment: fixed" in styles_css
    assert ".panel section::before" in styles_css
    assert ".system-card:hover" in styles_css
    assert "transform: translateY(-2px)" in styles_css


def test_frontend_applies_apple_liquid_glass_guidance():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--liquid-glass-tint" in styles_css
    assert "--liquid-glass-highlight" in styles_css
    assert ".tabs::before" in styles_css
    assert ".panel section::after" in styles_css
    assert ".terminal::before" in styles_css
    assert ".source::before" in styles_css
    assert "transition: opacity 260ms ease, transform 420ms ease" in styles_css
    assert "cubic-bezier(0.22, 1, 0.36, 1)" in styles_css


def test_top_tabs_use_sliding_segmented_selector():
    static_dir = Path(__file__).resolve().parents[1] / "static"
    index_html = (static_dir / "index.html").read_text(encoding="utf-8")
    app_js = (static_dir / "app.js").read_text(encoding="utf-8")
    styles_css = (static_dir / "styles.css").read_text(encoding="utf-8")

    assert 'id="viewTabs"' in index_html
    assert 'class="tabs sliding-tabs"' in index_html
    assert 'data-active="3d"' in index_html
    assert 'document.getElementById("viewTabs").dataset.active = tab;' in app_js
    assert ".sliding-tabs::before" in styles_css
    assert '.sliding-tabs[data-active="system"] {' in styles_css
    assert "--tabs-index: 1;" in styles_css


def test_sliding_selectors_use_transform_based_ios_spring_motion():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-spring: cubic-bezier(0.2, 0.85, 0.25, 1.08)" in styles_css
    assert "will-change: transform, box-shadow, filter;" in styles_css
    assert "transform: translate3d(calc(var(--tabs-index, 0) * 100%), 0, 0)" in styles_css
    assert "transform: translate3d(calc(var(--mode-index, 0) * 100%), 0, 0)" in styles_css
    assert "transition: transform 430ms var(--ios-spring)" in styles_css
    assert '.sliding-tabs[data-active="system"] {' in styles_css
    assert '.segmented[data-active="goal"] {' in styles_css
    assert "transition: left 280ms" not in styles_css
    assert "transform: translateX(100%)" not in styles_css
    assert "transform: translateX(200%)" not in styles_css


def test_sliding_selectors_use_ios_26_liquid_glass_material():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--liquid-slider-fill" in styles_css
    assert "--liquid-slider-rim" in styles_css
    assert "--liquid-slider-caustic" in styles_css
    assert "--liquid-slider-noise" in styles_css
    assert "data:image/svg+xml" in styles_css
    assert "feTurbulence" in styles_css
    assert "backdrop-filter: blur(18px) saturate(1.9) brightness(1.08);" in styles_css
    assert "-webkit-backdrop-filter: blur(18px) saturate(1.9) brightness(1.08);" in styles_css
    assert "var(--liquid-slider-noise)" in styles_css
    assert "background-blend-mode: soft-light" in styles_css
    assert "background-size: 80px 80px" in styles_css
    assert "radial-gradient(circle at 18% 0%, var(--liquid-slider-caustic), transparent 42%)" in styles_css
    assert "inset 0 1px 0 rgba(255, 255, 255, 0.72)" in styles_css
    assert "filter: drop-shadow(0 10px 24px rgba(10, 132, 255, 0.18));" in styles_css


def test_top_tab_slider_uses_equal_width_segments():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "grid-template-columns: repeat(3, minmax(96px, 1fr))" in styles_css
    assert "grid-template-columns: repeat(3, max-content)" not in styles_css
    assert ".sliding-tabs .tab" in styles_css
    assert "justify-content: center" in styles_css


def test_top_tab_hover_does_not_move_or_draw_extra_blue_button():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".sliding-tabs .tab:hover" in styles_css
    assert "transform: none;" in styles_css
    assert ".sliding-tabs .tab.active" in styles_css
    assert "min-width: 0;" in styles_css
    assert ".tab::after" not in styles_css
    assert ".tab:hover::after" not in styles_css


def test_mode_segment_hover_does_not_draw_offset_blue_button():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert ".segmented .mode:hover," in styles_css
    assert ".segmented .mode.active" in styles_css
    assert "background: transparent;" in styles_css[
        styles_css.index(".segmented .mode:hover,") : styles_css.index("label {")
    ]
    assert "transform: none;" in styles_css[
        styles_css.index(".segmented .mode:hover,") : styles_css.index("label {")
    ]


def test_mode_track_is_visible_in_light_theme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--mode-track-bg: rgba(10, 132, 255, 0.1);" in styles_css
    assert "--mode-track-border: rgba(10, 132, 255, 0.22);" in styles_css
    assert "--mode-track-glow: rgba(10, 132, 255, 0.16);" in styles_css
    assert "border: 1px solid var(--mode-track-border);" in styles_css
    assert "background: linear-gradient(180deg, rgba(255, 255, 255, 0.48), var(--mode-track-bg));" in styles_css
    assert "0 0 0 1px var(--mode-track-glow)" in styles_css
    assert "--mode-track-bg: rgba(255, 255, 255, 0.06);" in styles_css
    assert "--mode-slider-fill" not in styles_css
    assert "border: 1px solid var(--liquid-slider-rim);" in styles_css


def test_mode_slider_uses_contained_spring_to_stay_inside_track():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "--ios-contained-spring: cubic-bezier(0.2, 0.85, 0.25, 1);" in styles_css
    segmented_before = styles_css[
        styles_css.index(".segmented::before") : styles_css.index(".segmented[data-active=\"initial\"]")
    ]
    assert "transition: transform 430ms var(--ios-contained-spring)" in segmented_before
    assert "var(--ios-spring)" not in segmented_before


def test_mode_slider_is_clipped_and_mode_text_is_centered():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    segmented_block = styles_css[
        styles_css.index(".segmented {") : styles_css.index(".segmented::before")
    ]
    mode_start = styles_css.index(".mode {")
    mode_block = styles_css[mode_start : styles_css.index(".mode.active", mode_start)]

    assert "overflow: hidden;" in segmented_block
    assert "display: flex;" in mode_block
    assert "align-items: center;" in mode_block
    assert "justify-content: center;" in mode_block
    assert "text-align: center;" in mode_block
    assert "line-height: 1.1;" in mode_block


def test_frontend_follows_system_light_and_dark_color_scheme():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "color-scheme: light dark" in styles_css
    assert "@media (prefers-color-scheme: dark)" in styles_css
    assert "--bg: #f5f7fb" in styles_css
    assert "--text: #111827" in styles_css
    assert "--glass: rgba(255, 255, 255, 0.62)" in styles_css
    assert "--bg: #07080d" in styles_css
    assert "body::before" in styles_css
    assert ".system-viewport" in styles_css


def test_mobile_layout_keeps_control_panel_as_side_panel():
    styles_css = (
        Path(__file__).resolve().parents[1] / "static" / "styles.css"
    ).read_text(encoding="utf-8")

    assert "min-width: 760px;" in styles_css
    assert "@media (max-width: 900px)" in styles_css
    assert "grid-template-columns: minmax(420px, 1fr) 340px;" in styles_css
    assert "grid-template-rows: 1fr;" in styles_css
    assert "overflow-x: auto;" in styles_css
    mobile_app_block_start = styles_css.index("@media (max-width: 900px)")
    mobile_app_block_end = styles_css.index(".panel", mobile_app_block_start)
    assert "grid-template-columns: 1fr;" not in styles_css[mobile_app_block_start:mobile_app_block_end]
    assert "grid-template-rows: 52vh 48vh;" not in styles_css
