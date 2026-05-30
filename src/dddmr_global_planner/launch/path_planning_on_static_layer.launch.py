import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            default_value='/home/clab/map/hesai_sim/GlobalMap.pcd',
            description='Path to the global map PCD file.',
        ),
        DeclareLaunchArgument(
            'ground_dir',
            default_value='/home/clab/map/hesai_sim/GroundMap.pcd',
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
                {
                    'map_dir': map_dir,
                    'ground_dir': ground_dir,
                    'use_sim_time': use_sim,
                },
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
                '--frame-id', 'map',
                '--child-frame-id', 'base_link',
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
