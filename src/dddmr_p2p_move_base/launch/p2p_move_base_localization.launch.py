import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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
                localization_config,
                {
                    'map_dir': PathJoinSubstitution([map_path, 'GroundMap.pcd']),
                    'ground_dir': PathJoinSubstitution([map_path, 'GroundMap.pcd']),
                    'use_sim_time': use_sim,
                },
            ],
        ),
        Node(
            package='perception_3d',
            executable='shared_cloud_preprocessor_node',
            name='shared_cloud_preprocessor',
            output='screen',
            respawn=False,
            parameters=[localization_config, {'use_sim_time': use_sim}],
        ),
        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            prefix=global_planner_prefix,
            parameters=[localization_config, {'use_sim_time': use_sim}],
        ),
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[localization_config, {'use_sim_time': use_sim}],
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
