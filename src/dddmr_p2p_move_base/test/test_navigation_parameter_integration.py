"""Isolated ROS integration checks for the shared public parameter names."""

import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent
TF_SCRIPT = WORKSPACE / 'dddmr_global_planner' / 'script' / 'static_transform_from_params.py'
LOG_DIR = Path('/tmp/dddmr_verify_ros_logs')


def ros_env():
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '187',
        'ROS_LOCALHOST_ONLY': '1',
        'PYTHONDONTWRITEBYTECODE': '1',
        'ROS_LOG_DIR': str(LOG_DIR),
    })
    return env


def executable(package, name):
    path = Path(get_package_prefix(package)) / 'lib' / package / name
    assert path.is_file(), f'missing built executable: {path}'
    return path


def start(cmd, env=None):
    return subprocess.Popen(
        [str(item) for item in cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env or ros_env(),
    )


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    if process.poll() not in (0, -15, None):
        _, stderr = process.communicate(timeout=5)
        print(stderr, file=sys.stderr)


class InertNode:
    def __init__(self, name):
        self.node = Node(name)
        self.messages = []

    def wait(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
            time.sleep(0.02)
        return False


@pytest.fixture
def inert_node():
    rclpy.init(args=[])
    node = InertNode(f'wildcard_receiver_probe_{uuid.uuid4().hex[:8]}')
    try:
        yield node
    finally:
        node.node.destroy_node()
        rclpy.shutdown()


def write_pcd(path, point):
    path.write_text(
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        'WIDTH 1\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        'POINTS 1\n'
        'DATA ascii\n'
        f'{point[0]} {point[1]} {point[2]}\n')


def test_static_tf_script_reads_common_frames():
    token = uuid.uuid4().hex[:8]
    frame = f'wildcard_map_{token}'
    child = f'wildcard_base_{token}'
    topic = '/tf_static'
    params = LOG_DIR / f'wildcard_tf_{token}.yaml'
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {
            'global_frame': frame,
            'robot_base_frame': child,
        }},
    }, sort_keys=False))

    process = start([
        sys.executable, TF_SCRIPT,
        '--x', '-3.73', '--y', '-3.18', '--z', '-0.5',
        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
        '--ros-args', '--params-file', params,
        '-r', f'__node:=wildcard_tf_{token}',
    ])
    # Join late: a transient-local static TF must survive the publisher's startup
    # and prove the script stays alive after its formatted log call.
    time.sleep(2.2)
    if process.poll() is not None:
        _, stderr = process.communicate(timeout=5)
        pytest.fail(f'TF script exited before the late subscriber joined: {stderr}')
    rclpy.init(args=[])
    node = Node(f'wildcard_tf_probe_{token}')
    received = []
    qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    node.create_subscription(TFMessage, topic, received.append, qos)
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                _, stderr = process.communicate(timeout=5)
                pytest.fail(f'TF script exited early: {stderr}')
        assert received, 'no static transform received from the parameter-reading publisher'
        transform = next(
            (
                item for message in received for item in message.transforms
                if item.header.frame_id == frame and item.child_frame_id == child
            ),
            None,
        )
        assert transform is not None, 'late subscriber did not receive the durable static TF'
        assert transform.header.frame_id == frame
        assert transform.child_frame_id == child
        assert transform.transform.translation.x == pytest.approx(-3.73)
        assert transform.transform.translation.y == pytest.approx(-3.18)
        assert transform.transform.translation.z == pytest.approx(-0.5)
        assert transform.transform.rotation.x == pytest.approx(0.0)
        assert transform.transform.rotation.y == pytest.approx(0.0)
        assert transform.transform.rotation.z == pytest.approx(0.0)
        assert transform.transform.rotation.w == pytest.approx(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop(process)
        params.unlink(missing_ok=True)


def test_pcl_publisher_uses_custom_common_topics_and_frame(inert_node, tmp_path):
    token = uuid.uuid4().hex[:8]
    frame = f'custom_map_frame_{token}'
    map_topic = f'/custom_map_{token}'
    ground_topic = f'/custom_ground_{token}'
    map_pcd = tmp_path / 'map.pcd'
    ground_pcd = tmp_path / 'ground.pcd'
    write_pcd(map_pcd, (1.0, 2.0, 3.0))
    write_pcd(ground_pcd, (4.0, 5.0, 0.0))
    params = tmp_path / 'custom_common.yaml'
    params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {
            'global_frame': frame,
            'map_topic': map_topic,
            'ground_topic': ground_topic,
        }},
    }, sort_keys=False))

    pcl_config = Path(get_package_share_directory('mcl_3dl')) / 'config' / 'pcl_publisher.yaml'
    process = start([
        executable('mcl_3dl', 'pcl_publisher'),
        '--ros-args',
        '--params-file', pcl_config,
        '--params-file', params,
        '-p', f'map_dir:={map_pcd}',
        '-p', f'ground_dir:={ground_pcd}',
    ])
    qos = QoSProfile(
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    clouds = {}
    inert_node.node.create_subscription(
        PointCloud2, map_topic, lambda msg: clouds.setdefault('map', msg), qos)
    inert_node.node.create_subscription(
        PointCloud2, ground_topic, lambda msg: clouds.setdefault('ground', msg), qos)
    try:
        assert inert_node.wait(
            lambda: 'map' in clouds and 'ground' in clouds, timeout=30.0), (
                'pcl_publisher did not publish on the custom common topics')
        assert clouds['map'].header.frame_id == frame
        assert clouds['ground'].header.frame_id == frame
        assert clouds['map'].width == 1
        assert clouds['ground'].width == 1
    finally:
        stop(process)


def test_native_param_resolution_lets_wildcard_override_pcl_publisher_default(tmp_path):
    """rcl resolves the same file order used by the launch files.

    The shared ``/**`` frame is applied after mcl_3dl's node-specific config.
    The C++ node keeps a ``map`` default for launches that pass no common file.
    """
    common = tmp_path / 'common.yaml'
    frame = f'native_custom_frame_{uuid.uuid4().hex[:8]}'
    common.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'global_frame': frame}},
    }, sort_keys=False))
    pcl_config = Path(get_package_share_directory('mcl_3dl')) / 'config' / 'pcl_publisher.yaml'

    rclpy.init(args=[])
    node = Node(
        'pcl_publisher',
        cli_args=[
            '--ros-args',
            '--params-file', str(pcl_config),
            '--params-file', str(common),
        ],
    )
    try:
        resolved = node.declare_parameter('global_frame', 'map').value
        assert resolved == frame
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_shared_cloud_preprocessor_uses_custom_common_topics_and_filters(
        inert_node, tmp_path):
    token = uuid.uuid4().hex[:8]
    node_name = f'shared_cloud_preprocessor_{token}'
    input_topic = f'/custom_raw_{token}'
    output_topic = f'/custom_preprocessed_{token}'
    global_frame = f'custom_global_{token}'
    robot_frame = f'custom_base_{token}'
    sensor_frame = f'custom_sensor_{token}'
    params = tmp_path / 'custom_common.yaml'
    params.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {
            'global_frame': global_frame,
            'robot_base_frame': robot_frame,
            'sensor_frame': sensor_frame,
            'input_cloud_topic': input_topic,
            'preprocessed_cloud_topic': output_topic,
            'perception_window_size': 4.0,
            'min_obstacle_height': -0.25,
            'marking_height': 1.5,
            'expected_sensor_time': 0.75,
        }},
    }, sort_keys=False))

    process = start([
        executable('perception_3d', 'shared_cloud_preprocessor_node'),
        '--ros-args',
        '--params-file', params,
        '-r', f'__node:={node_name}',
    ])
    try:
        assert inert_node.wait(
            lambda: (
                inert_node.node.count_subscribers(input_topic) >= 1 and
                inert_node.node.count_publishers(output_topic) >= 1
            ),
            timeout=20.0,
        ), 'preprocessor did not bind the custom common input/output topics'

        expected = {
            'perception_window_size': '4.0',
            'min_obstacle_height': '-0.25',
            'marking_height': '1.5',
            'expected_sensor_time': '0.75',
        }
        for key, value in expected.items():
            output = None
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                completed = subprocess.run(
                    ['ros2', 'param', 'get', f'/{node_name}', key],
                    capture_output=True, text=True, timeout=10, env=ros_env(),
                    check=False,
                )
                if completed.returncode == 0:
                    output = completed.stdout
                    break
                time.sleep(0.2)
            assert output is not None, f'could not read {key}'
            assert value in output, (key, output)
    finally:
        stop(process)
        params.unlink(missing_ok=True)
