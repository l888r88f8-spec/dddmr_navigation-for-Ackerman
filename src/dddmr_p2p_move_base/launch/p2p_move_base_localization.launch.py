import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p2p_share_dir = get_package_share_directory('p2p_move_base')
    mcl_share_dir = get_package_share_directory('mcl_3dl')

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
        p2p_share_dir,
        'rviz',
        'p2p_move_base_localization.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true', # false if you want to use real robot
            description='Set use_sim_time for launched nodes.',
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value='/home/robot/map/hesai_sim/GlobalMap.pcd', # change this to your global map pcd file path
            description='Path to the global map PCD file.',
        ),
        DeclareLaunchArgument(
            'ground_dir',
            default_value='/home/robot/map/hesai_sim/GroundMap.pcd', # change this to your ground map pcd file path

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
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                localization_config,
                {'use_sim_time': use_sim},
            ],
            remappings=[
                ('/odom', '/lio_sam/localization/odometry'),
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
            parameters=[
                {'use_sim_time': use_sim},
            ],
        ),
    ])
