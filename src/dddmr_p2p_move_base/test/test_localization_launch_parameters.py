"""Verify wildcard parameter ownership and launch consumption, without robot processes."""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import LogInfo, TimerAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent
CONFIG_PATH = PACKAGE / 'config' / 'p2p_move_base_localization.yaml'
LAUNCH_PATH = PACKAGE / 'launch' / 'p2p_move_base_localization.launch.py'
STANDALONE_LAUNCH_PATH = (
    WORKSPACE / 'dddmr_global_planner' / 'launch' /
    'path_planning_on_static_layer.launch.py')
PCL_CONFIG_PATH = WORKSPACE / 'dddmr_mcl_3dl' / 'config' / 'pcl_publisher.yaml'
SHARE_PATHS = {
    'p2p_move_base': PACKAGE,
    'mcl_3dl': WORKSPACE / 'dddmr_mcl_3dl',
    'global_planner': WORKSPACE / 'dddmr_global_planner',
}
COMMON_STRINGS = (
    'global_frame', 'robot_base_frame', 'sensor_frame', 'input_cloud_topic',
    'preprocessed_cloud_topic', 'map_topic', 'ground_topic', 'odom_topic',
)
SHARED_FILTERS = {
    'perception_window_size': 6.0,
    'min_obstacle_height': -0.12,
    'marking_height': 2.0,
    'expected_sensor_time': 0.5,
}
PROBE = r'''
import json
import sys
import rclpy
from rclpy.node import Node

requested_path = sys.argv[1]
node_name = sys.argv[2]
rclpy.init(args=sys.argv[3:])
node = Node(node_name, automatically_declare_parameters_from_overrides=True)
requested = json.loads(open(requested_path, encoding='utf-8').read())
result = {}
for key in requested:
    try:
        result[key] = node.get_parameter(key).value
    except Exception as exc:  # noqa: BLE001 - probe reports the exact failure
        result[key] = f'ERROR:{type(exc).__name__}:{exc}'
print(json.dumps(result, sort_keys=True))
rclpy.shutdown()
'''


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def capture_launch(path, module_name):
    module = load_module(module_name, path)
    nodes = []
    declarations = {}

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return LogInfo(msg='test: node captured without execution')

    def capture_declaration(name, **kwargs):
        declarations[name] = kwargs
        return LogInfo(msg='test: launch argument captured')

    def share(package):
        return str(SHARE_PATHS[package])

    with patch.object(module, 'Node', side_effect=capture_node), patch.object(
            module, 'DeclareLaunchArgument', side_effect=capture_declaration), patch.object(
            module, 'get_package_share_directory', side_effect=share):
        description = module.generate_launch_description()
    return module, description, nodes, declarations


def node(nodes, executable):
    return next(item for item in nodes if item['executable'] == executable)


def parameter_entries(action):
    return list(action.get('parameters') or [])


def parameter_paths(action):
    return [entry for entry in parameter_entries(action) if isinstance(entry, str)]


def parameter_dict(action):
    result = {}
    for entry in parameter_entries(action):
        if isinstance(entry, dict):
            result.update(entry)
    return result


def resolve_value(value, launch_configurations):
    if isinstance(value, str):
        return value
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations)
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def resolve_parameters(param_files, node_name, keys, tmp_path):
    requested = tmp_path / f'requested_{node_name}.json'
    requested.write_text(json.dumps(keys))
    cmd = [sys.executable, '-c', PROBE, str(requested), node_name, '--ros-args']
    for path in param_files:
        cmd.extend(['--params-file', str(path)])
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '187',
        'ROS_LOCALHOST_ONLY': '1',
        'PYTHONDONTWRITEBYTECODE': '1',
        'ROS_LOG_DIR': '/tmp/dddmr_verify_ros_logs',
    })
    completed = subprocess.run(
        cmd, capture_output=True, text=True, timeout=30, env=env, check=False)
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout.strip().splitlines()[-1])


def test_shared_names_and_filters_have_one_top_level_wildcard_source():
    config = load_yaml(CONFIG_PATH)
    wildcard = config['/**']['ros__parameters']

    assert 'cuboid' in wildcard
    for key in COMMON_STRINGS:
        assert isinstance(wildcard.get(key), str) and wildcard[key], f'missing common {key}'
    for key, value in SHARED_FILTERS.items():
        assert wildcard[key] == value, f'unexpected shared {key}'

    assert all(key not in config['p2p_move_base']['ros__parameters'] for key in COMMON_STRINGS)
    for layer in ('perception_3d_global', 'perception_3d_local'):
        lidar = config[layer]['ros__parameters']['lidar']
        assert all(key not in lidar for key in SHARED_FILTERS)

    # A node-specific copy would silently win over /** and reintroduce two sources.
    for section, parameters in config.items():
        if section == '/**':
            continue
        values = parameters['ros__parameters']
        for key in COMMON_STRINGS:
            assert key not in values, f'{section}.{key} would override the wildcard'
        for key in SHARED_FILTERS:
            assert key not in values, f'{section}.{key} would override the wildcard'

    pcl = load_yaml(PCL_CONFIG_PATH)['pcl_publisher']['ros__parameters']
    assert 'global_frame' not in pcl
    assert 'map_topic' not in pcl and 'ground_topic' not in pcl


def test_baseline_control_and_fusion_settings_are_preserved():
    config = load_yaml(CONFIG_PATH)
    p2p = config['p2p_move_base']['ros__parameters']
    global_perception = config['perception_3d_global']['ros__parameters']
    local_perception = config['perception_3d_local']['ros__parameters']
    local_planner = config['local_planner']['ros__parameters']

    assert p2p['local_obstacle_layer']['enabled'] is False
    assert p2p['local_obstacle_layer']['stale_timeout_sec'] == 0.6
    assert global_perception['sensors_collected_frequency'] == 10.0
    assert local_perception['sensors_collected_frequency'] == 10.0
    assert local_planner['controller_frequency'] == 15.0
    assert local_planner['mppi']['wheelbase'] == 0.65
    assert local_planner['mppi']['max_steer'] == 0.461
    assert local_planner['mppi']['vx_max'] == 1.0
    assert config['perception_3d_global']['ros__parameters']['lidar']['input_preprocessed'] is True
    assert config['perception_3d_local']['ros__parameters']['lidar']['input_preprocessed'] is True


def test_ros_yaml_applies_common_names_and_node_specific_values_win(tmp_path):
    config = load_yaml(CONFIG_PATH)
    common = config['/**']['ros__parameters']
    custom = {key: f'wildcard/{key}' for key in COMMON_STRINGS}
    common.update(custom)
    config['perception_3d_global'] = {
        'ros__parameters': {'map_topic': 'node_specific/map'},
    }
    custom_path = tmp_path / 'custom_localization.yaml'
    custom_path.write_text(yaml.safe_dump(config, sort_keys=False))

    keys = list(COMMON_STRINGS) + list(SHARED_FILTERS)
    for node_name in (
            'perception_3d_local', 'shared_cloud_preprocessor',
            'p2p_move_base', 'local_planner', 'pcl_publisher'):
        resolved = resolve_parameters([custom_path], node_name, keys, tmp_path)
        for key in COMMON_STRINGS:
            assert resolved[key] == custom[key], (node_name, key)
        for key, value in SHARED_FILTERS.items():
            assert resolved[key] == value, (node_name, key)

    resolved = resolve_parameters(
        [custom_path], 'perception_3d_global', keys, tmp_path)
    assert resolved['map_topic'] == 'node_specific/map'
    assert resolved['ground_topic'] == custom['ground_topic']


def test_pcl_common_config_after_pcl_defaults_overrides_shared_global_frame(tmp_path):
    config = load_yaml(CONFIG_PATH)
    common = config['/**']['ros__parameters']
    custom = {key: f'combined/{key}' for key in COMMON_STRINGS}
    common.update(custom)
    custom_path = tmp_path / 'combined_localization.yaml'
    custom_path.write_text(yaml.safe_dump(config, sort_keys=False))

    resolved = resolve_parameters(
        [PCL_CONFIG_PATH, custom_path], 'pcl_publisher',
        list(COMMON_STRINGS) + ['map_down_sample'], tmp_path)
    assert resolved['global_frame'] == custom['global_frame']
    assert resolved['map_topic'] == custom['map_topic']
    assert resolved['ground_topic'] == custom['ground_topic']
    assert resolved['map_down_sample'] == 0.3


def test_main_launch_passes_the_same_config_without_wiring_or_remaps():
    module, _, nodes, declarations = capture_launch(
        LAUNCH_PATH, 'main_localization_launch')
    assert not hasattr(module, 'build_navigation_wiring')
    assert not hasattr(module, 'yaml')

    config = str(CONFIG_PATH)
    for executable in (
            'shared_cloud_preprocessor_node', 'global_planner_node', 'p2p_move_base_node'):
        assert config in parameter_paths(node(nodes, executable)), executable

    publisher = node(nodes, 'pcl_publisher')
    paths = parameter_paths(publisher)
    pcl_config = str(PCL_CONFIG_PATH)
    assert paths.index(pcl_config) < paths.index(config)
    assert not publisher.get('remappings')
    override = parameter_dict(publisher)
    map_path = declarations['map_path']['default_value']
    assert resolve_value(override['map_dir'], {'map_path': map_path}) == (
        '/home/robot/map/hesai_outdoor_yifu/GroundMap.pcd')
    assert resolve_value(override['ground_dir'], {'map_path': map_path}) == (
        '/home/robot/map/hesai_outdoor_yifu/GroundMap.pcd')


def test_standalone_launch_uses_same_config_scripted_tf_and_original_defaults():
    module, description, nodes, declarations = capture_launch(
        STANDALONE_LAUNCH_PATH, 'standalone_global_launch')
    assert not hasattr(module, 'load_navigation_wiring')
    assert not hasattr(module, 'build_navigation_wiring')
    assert not hasattr(module, 'yaml')
    assert not hasattr(module, 'importlib')

    config = str(CONFIG_PATH)
    for executable in (
            'shared_cloud_preprocessor_node', 'global_planner_node', 'pcl_publisher'):
        assert config in parameter_paths(node(nodes, executable)), executable
    publisher = node(nodes, 'pcl_publisher')
    assert parameter_paths(publisher).index(str(PCL_CONFIG_PATH)) < parameter_paths(
        publisher).index(config)
    assert not publisher.get('remappings')
    override = parameter_dict(publisher)
    assert resolve_value(
        override['map_dir'], {'map_dir': declarations['map_dir']['default_value']}) == (
            '/home/robot/map/hesai_outdoor_cuiwen/GlobalMap.pcd')
    assert resolve_value(
        override['ground_dir'], {'ground_dir': declarations['ground_dir']['default_value']}) == (
            '/home/robot/map/hesai_outdoor_cuiwen/GroundMap.pcd')

    assert declarations['map_dir']['default_value'] == (
        '/home/robot/map/hesai_outdoor_cuiwen/GlobalMap.pcd')
    assert declarations['ground_dir']['default_value'] == (
        '/home/robot/map/hesai_outdoor_cuiwen/GroundMap.pcd')

    tf_node = node(nodes, 'static_transform_from_params.py')
    arguments = tf_node['arguments']
    assert config in parameter_paths(tf_node)
    assert arguments[arguments.index('--x') + 1] == '-3.73'
    assert arguments[arguments.index('--y') + 1] == '-3.18'
    assert arguments[arguments.index('--z') + 1] == '-0.5'
    assert arguments[arguments.index('--roll') + 1] == '0.0'
    assert arguments[arguments.index('--pitch') + 1] == '0.0'
    assert arguments[arguments.index('--yaw') + 1] == '0.0'

    timers = [entity for entity in description.entities if isinstance(entity, TimerAction)]
    assert len(timers) == 1
    assert timers[0].period == 2.0
