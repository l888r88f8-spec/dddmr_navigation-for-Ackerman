"""Check shared name wiring without starting ROS processes or robot control."""

import copy
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml
from launch.actions import LogInfo


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE / 'config' / 'p2p_move_base_localization.yaml'
LAUNCH_PATH = PACKAGE / 'launch' / 'p2p_move_base_localization.launch.py'
COMMON_NAMES = (
    'global_frame', 'robot_base_frame', 'sensor_frame', 'input_cloud_topic',
    'preprocessed_cloud_topic', 'map_topic', 'ground_topic', 'odom_topic',
)


def load_launch():
    spec = importlib.util.spec_from_file_location('localization_launch_under_test', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_config():
    return yaml.safe_load(CONFIG_PATH.read_text())


def build_wiring(config):
    build = getattr(load_launch(), 'build_navigation_wiring', None)
    assert callable(build), 'launch must distribute the centralized topic/frame settings'
    return build(config)


def test_common_names_have_one_authoritative_location():
    config = load_config()
    common = config['p2p_move_base']['ros__parameters']
    for key in COMMON_NAMES:
        assert isinstance(common.get(key), str) and common[key], f'missing common {key}'

    def check_no_names(value):
        if not isinstance(value, dict):
            return
        for key, child in value.items():
            assert key != 'topic' and not key.endswith(('_topic', '_frame')), key
            check_no_names(child)

    for section, parameters in config.items():
        if section != 'p2p_move_base':
            check_no_names(parameters)
    obstacle = common['local_obstacle_layer']
    assert 'input_cloud_topic' not in obstacle
    assert 'base_frame' not in obstacle


def test_common_rename_reaches_all_actual_launch_consumers(tmp_path):
    config = load_config()
    common = config['p2p_move_base']['ros__parameters']
    common.update({name: f'custom/{name}' for name in COMMON_NAMES})
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config' / CONFIG_PATH.name).write_text(yaml.safe_dump(config))
    module = load_launch()
    nodes = []

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return LogInfo(msg='test: node captured without execution')

    real_share = module.get_package_share_directory
    with patch.object(module, 'Node', side_effect=capture_node), patch.object(
            module, 'get_package_share_directory',
            side_effect=lambda package: str(tmp_path) if package == 'p2p_move_base'
            else real_share(package)):
        module.generate_launch_description()

    def node(executable):
        return next(item for item in nodes if item['executable'] == executable)

    def overrides(spec):
        result = {}
        for parameters in spec['parameters']:
            if isinstance(parameters, dict):
                result.update(parameters)
        return result

    for executable in ('global_planner_node', 'p2p_move_base_node'):
        parameters = overrides(node(executable))
        assert parameters['global_frame'] == common['global_frame']
        assert parameters['robot_base_frame'] == common['robot_base_frame']
        assert parameters['map.map_topic'] == common['map_topic']
        assert parameters['map.ground_topic'] == common['ground_topic']
        assert parameters['lidar.topic'] == common['preprocessed_cloud_topic']
        assert parameters['lidar.sensor_frame'] == common['sensor_frame']
    local = overrides(node('p2p_move_base_node'))
    assert local['odom_topic'] == common['odom_topic']
    assert local['local_obstacle_layer.input_cloud_topic'] == common['input_cloud_topic']
    assert local['local_obstacle_layer.base_frame'] == common['robot_base_frame']
    assert not node('p2p_move_base_node').get('remappings'), 'legacy odom remap must not override common input'
    producer = overrides(node('shared_cloud_preprocessor_node'))
    assert producer['input_topic'] == common['input_cloud_topic']
    assert producer['output_topic'] == common['preprocessed_cloud_topic']
    for key in ('global_frame', 'robot_base_frame', 'sensor_frame'):
        assert producer[key] == common[key]
    publisher = node('pcl_publisher')
    assert overrides(publisher)['global_frame'] == common['global_frame']
    assert ('mapcloud', common['map_topic']) in publisher['remappings']
    assert ('mapground', common['ground_topic']) in publisher['remappings']


@pytest.mark.parametrize('name', COMMON_NAMES)
def test_missing_common_name_fails_instead_of_using_a_silent_default(name):
    config = load_config()
    config['p2p_move_base']['ros__parameters'].pop(name, None)
    with pytest.raises(ValueError, match=name):
        build_wiring(config)


def test_empty_common_name_fails():
    config = load_config()
    config['p2p_move_base']['ros__parameters']['input_cloud_topic'] = ' '
    with pytest.raises(ValueError, match='input_cloud_topic'):
        build_wiring(config)


def test_shared_filter_bounds_still_require_matching_consumers():
    config = load_config()
    config['perception_3d_global']['ros__parameters']['lidar']['perception_window_size'] = 5.0
    with pytest.raises(ValueError, match='perception_window_size'):
        build_wiring(config)


def test_building_wiring_does_not_mutate_numeric_or_controller_configuration():
    config = load_config()
    original = copy.deepcopy(config)
    build_wiring(config)
    assert config == original


def test_standalone_global_launch_uses_the_same_common_configuration(tmp_path):
    config = load_config()
    common = config['p2p_move_base']['ros__parameters']
    common.update({name: f'custom/{name}' for name in COMMON_NAMES})
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config' / CONFIG_PATH.name).write_text(yaml.safe_dump(config))
    (tmp_path / 'launch').mkdir()
    (tmp_path / 'launch' / LAUNCH_PATH.name).write_text(LAUNCH_PATH.read_text())
    path = PACKAGE.parent / 'dddmr_global_planner' / 'launch' / 'path_planning_on_static_layer.launch.py'
    spec = importlib.util.spec_from_file_location('standalone_global_launch_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = []

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return LogInfo(msg='test: node captured without execution')

    real_share = module.get_package_share_directory
    with patch.object(module, 'Node', side_effect=capture_node), patch.object(
            module, 'get_package_share_directory',
            side_effect=lambda package: str(tmp_path) if package == 'p2p_move_base'
            else real_share(package)):
        module.generate_launch_description()

    def node(executable):
        return next(item for item in nodes if item['executable'] == executable)

    def overrides(spec):
        result = {}
        for parameters in spec['parameters']:
            if isinstance(parameters, dict):
                result.update(parameters)
        return result

    parameters = overrides(node('global_planner_node'))
    assert parameters['global_frame'] == common['global_frame']
    assert parameters['robot_base_frame'] == common['robot_base_frame']
    assert parameters['lidar.topic'] == common['preprocessed_cloud_topic']
    assert parameters['lidar.sensor_frame'] == common['sensor_frame']
    producer = overrides(node('shared_cloud_preprocessor_node'))
    assert producer['input_topic'] == common['input_cloud_topic']
    assert producer['output_topic'] == common['preprocessed_cloud_topic']
    assert producer['global_frame'] == common['global_frame']
    publisher = node('pcl_publisher')
    assert overrides(publisher)['global_frame'] == common['global_frame']
    assert ('mapcloud', common['map_topic']) in publisher['remappings']
    assert ('mapground', common['ground_topic']) in publisher['remappings']
    tf_arguments = node('static_transform_publisher')['arguments']
    assert tf_arguments[tf_arguments.index('--frame-id') + 1] == common['global_frame']
    assert tf_arguments[tf_arguments.index('--child-frame-id') + 1] == common['robot_base_frame']
