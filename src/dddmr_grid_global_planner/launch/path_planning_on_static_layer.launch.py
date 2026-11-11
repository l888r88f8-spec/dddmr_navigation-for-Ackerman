import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    grid_share = get_package_share_directory('dddmr_grid_global_planner')
    params_file = os.path.join(grid_share, 'config', 'dddmr_grid_global_planner.yaml')
    default_rviz_config = os.path.join(grid_share, 'rviz', 'dddmr_grid_global_planner.rviz')

    use_sim = LaunchConfiguration('use_sim')
    map_dir = LaunchConfiguration('map_dir')
    ground_dir = LaunchConfiguration('ground_dir')
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
            'map_dir',
            default_value='/home/robot/map/hesai_sim2/GroundMap.pcd',
            description=(
                'GroundMap input and mapcloud display. An explicit GlobalMap '
                'path is optional obstacle evidence only.'),
        ),
        DeclareLaunchArgument(
            'ground_dir',
            default_value='/home/robot/map/hesai_sim2/GroundMap.pcd',
            description=(
                'GroundMap used by the legacy ground-only build and the '
                'mapground display.'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # Single display-only publisher: mapcloud comes from map_dir and
        # mapground from ground_dir. The offline planner reads the PCDs
        # directly; this node never feeds the planning map.
        Node(
            package='mcl_3dl',
            executable='pcl_publisher',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'map_dir': map_dir,
                    'ground_dir': ground_dir,
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
        # The only /get_plan provider. The launch paths override the YAML
        # offline source paths; ground_dir drives the legacy ground build and
        # map_dir is optional same/independent obstacle evidence.
        Node(
            package='dddmr_grid_global_planner',
            executable='dddmr_grid_offline_planner_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'offline.global_pcd': map_dir,
                    'offline.ground_pcd': ground_dir,
                    'offline.obstacle_pcd': map_dir,
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
