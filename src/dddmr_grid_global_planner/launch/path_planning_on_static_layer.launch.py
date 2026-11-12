import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _offline_pcd_paths(params_file):
    with open(params_file, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    try:
        offline = config['dddmr_grid_global_planner']['ros__parameters']['offline']
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f'{params_file}: missing '
            'dddmr_grid_global_planner.ros__parameters.offline') from exc

    paths = {}
    for key in ('global_pcd', 'ground_pcd'):
        value = offline.get(key)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(
                f'{params_file}: offline.{key} must be a non-empty path')
        paths[key] = value
    return paths


def generate_launch_description():

    grid_share = get_package_share_directory('dddmr_grid_global_planner')
    params_file = os.path.join(grid_share, 'config', 'dddmr_grid_global_planner.yaml')
    default_rviz_config = os.path.join(grid_share, 'rviz', 'dddmr_grid_global_planner.rviz')
    offline_pcd = _offline_pcd_paths(params_file)

    use_sim = LaunchConfiguration('use_sim')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = ParameterValue(use_sim, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description=(
                'Set use_sim_time for all five nodes. Use true only when '
                'replaying with /clock.'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # Display reuses the planner's offline PCD paths from the same YAML.
        # The launch overrides below never feed the offline planning map.
        Node(
            package='mcl_3dl',
            executable='pcl_publisher',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'map_dir': offline_pcd['global_pcd'],
                    'ground_dir': offline_pcd['ground_pcd'],
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        # Display-only offline pose. The bridge carries explicit start/goal
        # poses, so the planner does not need this transform for planning.
        Node(
            package='global_planner',
            executable='static_transform_from_params.py',
            name='map2baselink',
            output='screen',
            respawn=False,
            emulate_tty=True,
            arguments=[
                '--x', '3.0',
                '--y', '0.1',
                '--z', '-0.5',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
            ],
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        # The only /get_plan provider. All planner PCD paths come from YAML;
        # the display publisher has its own YAML section.
        Node(
            package='dddmr_grid_global_planner',
            executable='dddmr_grid_offline_planner_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        # RViz click -> /get_plan bridge. It never publishes /initialpose or a
        # vehicle command and only displays successful current-generation paths.
        Node(
            package='dddmr_grid_global_planner',
            executable='rviz_goal_bridge.py',
            name='dddmr_grid_rviz_bridge',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
