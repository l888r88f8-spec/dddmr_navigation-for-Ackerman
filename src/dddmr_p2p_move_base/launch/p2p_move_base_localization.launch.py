import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def build_navigation_wiring(config):
    """Distribute the p2p section's common names to the actual ROS consumers."""
    common = config['p2p_move_base']['ros__parameters']
    for key in (
        'global_frame', 'robot_base_frame', 'sensor_frame', 'input_cloud_topic',
        'preprocessed_cloud_topic', 'map_topic', 'ground_topic', 'odom_topic',
    ):
        if not isinstance(common.get(key), str) or not common[key].strip():
            raise ValueError(f'p2p_move_base.ros__parameters.{key} must be a nonempty string')

    global_lidar = config['perception_3d_global']['ros__parameters']['lidar']
    local_lidar = config['perception_3d_local']['ros__parameters']['lidar']
    for key in ('perception_window_size', 'min_obstacle_height', 'marking_height'):
        if global_lidar[key] != local_lidar[key]:
            raise ValueError(f'Shared cloud requires matching lidar.{key} in both perception layers')

    return {
        'perception': {
            'global_frame': common['global_frame'],
            'robot_base_frame': common['robot_base_frame'],
            'map.map_topic': common['map_topic'],
            'map.ground_topic': common['ground_topic'],
            'lidar.topic': common['preprocessed_cloud_topic'],
            'lidar.sensor_frame': common['sensor_frame'],
        },
        'preprocessor': {
            **{key: common[key] for key in ('global_frame', 'robot_base_frame', 'sensor_frame')},
            'input_topic': common['input_cloud_topic'],
            'output_topic': common['preprocessed_cloud_topic'],
            **{key: local_lidar[key] for key in (
                'perception_window_size', 'min_obstacle_height', 'marking_height')},
            'expected_sensor_time': min(
                global_lidar['expected_sensor_time'], local_lidar['expected_sensor_time']),
        },
        'local': {
            'odom_topic': common['odom_topic'],
            'local_obstacle_layer.input_cloud_topic': common['input_cloud_topic'],
            'local_obstacle_layer.base_frame': common['robot_base_frame'],
        },
        'map_publisher': {'global_frame': common['global_frame']},
        'map_remappings': [('mapcloud', common['map_topic']), ('mapground', common['ground_topic'])],
    }


def generate_launch_description():
    p2p_share_dir = get_package_share_directory('p2p_move_base')
    mcl_share_dir = get_package_share_directory('mcl_3dl')

    use_sim = LaunchConfiguration('use_sim')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    global_planner_prefix = LaunchConfiguration('global_planner_prefix')
    map_path = LaunchConfiguration("map_path")

    localization_config = os.path.join(
        p2p_share_dir,
        'config',
        'p2p_move_base_localization.yaml',
    )
    pcl_publisher_config = os.path.join(
        mcl_share_dir,
        'config',
        'pcl_publisher.yaml',
    )
    default_rviz_config = os.path.join(
        p2p_share_dir,
        'rviz',
        'p2p_move_base_localization.rviz',
    )

    with open(localization_config, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    wiring = build_navigation_wiring(config)

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value=os.environ.get("USE_SIM_TIME") or "false",
            description=(
                "Use simulation clock if true. Defaults to USE_SIM_TIME from the "
                "launch process environment, or false when unset; an explicit "
                "use_sim:=... overrides it."
            ),
        ),
        DeclareLaunchArgument(
            "map_path",
            default_value="/home/robot/map/hesai_outdoor_yifu",
            description="Directory containing GlobalMap.pcd and GroundMap.pcd.",
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz.',
        ),
        DeclareLaunchArgument(
            'global_planner_prefix',
            default_value='',
            description='Optional command prefix for global_planner_node, e.g. gdb.',
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
                    'map_dir': PathJoinSubstitution([map_path, 'GroundMap.pcd']),
                    'ground_dir': PathJoinSubstitution([map_path, 'GroundMap.pcd']),
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
            parameters=[wiring['preprocessor'], {'use_sim_time': use_sim}],
        ),
        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            prefix=global_planner_prefix,
            parameters=[
                localization_config,
                wiring['perception'],
                {'use_sim_time': use_sim},
            ],
        ),
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                localization_config,
                wiring['perception'],
                wiring['local'],
                {'use_sim_time': use_sim},
            ],
        ),
        Node(
            package='p2p_move_base',
            executable='clicked2goal.py',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            emulate_tty=True,
            condition=IfCondition(use_rviz),
            parameters=[
                {'use_sim_time': use_sim},
            ],
        ),
    ])
