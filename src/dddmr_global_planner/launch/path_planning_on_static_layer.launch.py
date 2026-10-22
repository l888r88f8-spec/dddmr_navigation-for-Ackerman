import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_navigation_wiring(p2p_share_dir, localization_config):
    """Reuse the p2p localization launch's shared name distribution."""
    launch_path = os.path.join(
        p2p_share_dir,
        'launch',
        'p2p_move_base_localization.launch.py',
    )
    if not os.path.isfile(launch_path):
        raise RuntimeError(
            f'p2p_move_base localization launch not found: {launch_path}')
    if not os.path.isfile(localization_config):
        raise RuntimeError(
            f'p2p_move_base localization config not found: {localization_config}')

    spec = importlib.util.spec_from_file_location(
        'p2p_move_base_localization_wiring', launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(
            f'Cannot load p2p_move_base localization launch: {launch_path}')
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    build_navigation_wiring = getattr(
        launch_module, 'build_navigation_wiring', None)
    if not callable(build_navigation_wiring):
        raise RuntimeError(
            'p2p_move_base localization launch has no callable '
            f'build_navigation_wiring: {launch_path}')

    with open(localization_config, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError(
            f'p2p_move_base localization config is not a mapping: {localization_config}')
    return build_navigation_wiring(config)


def generate_launch_description():
    p2p_share_dir = get_package_share_directory('p2p_move_base')
    mcl_share_dir = get_package_share_directory('mcl_3dl')
    global_planner_share_dir = get_package_share_directory('global_planner')

    use_sim = LaunchConfiguration('use_sim')
    map_dir = LaunchConfiguration('map_dir')
    ground_dir = LaunchConfiguration('ground_dir')
    rviz_config = LaunchConfiguration('rviz_config')

    localization_config = os.path.join(
        p2p_share_dir,
        'config',
        'p2p_move_base_localization.yaml',
    )
    wiring = load_navigation_wiring(p2p_share_dir, localization_config)
    pcl_publisher_config = os.path.join(
        mcl_share_dir,
        'config',
        'pcl_publisher.yaml',
    )
    default_rviz_config = os.path.join(
        global_planner_share_dir,
        'rviz',
        'path_planning_on_static_layer.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Set use_sim_time for launched nodes. Use true only when replaying with /clock.',
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value='/home/robot/map/hesai_outdoor_cuiwen/GlobalMap.pcd',
            description='Path to the global map PCD file.',
        ),
        DeclareLaunchArgument(
            'ground_dir',
            default_value='/home/robot/map/hesai_outdoor_cuiwen/GroundMap.pcd',
            description='Path to the ground map PCD file.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='mcl_3dl',
            executable='pcl_publisher',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                pcl_publisher_config,
                wiring['map_publisher'],
                {
                    'map_dir': map_dir,
                    'ground_dir': ground_dir,
                    'use_sim_time': use_sim,
                },
            ],
            remappings=wiring['map_remappings'],
        ),
        Node(
            package='perception_3d',
            executable='shared_cloud_preprocessor_node',
            name='shared_cloud_preprocessor',
            output='screen',
            respawn=False,
            parameters=[
                wiring['preprocessor'],
                {'use_sim_time': use_sim},
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2baselink',
            arguments=[
                '--x', '-3.73',
                '--y', '-3.18',
                '--z', '-0.5',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', wiring['perception']['global_frame'],
                '--child-frame-id', wiring['perception']['robot_base_frame'],
            ],
            parameters=[
                {'use_sim_time': use_sim},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='global_planner',
                    executable='global_planner_node',
                    output='screen',
                    respawn=False,
                    emulate_tty=True,
                    parameters=[
                        localization_config,
                        wiring['perception'],
                        {'use_sim_time': use_sim},
                    ],
                ),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim},
            ],
        ),
    ])
