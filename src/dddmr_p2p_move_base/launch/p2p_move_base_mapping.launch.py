import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lego_loam_share_dir = get_package_share_directory('lego_loam_bor')
    p2p_share_dir = get_package_share_directory('p2p_move_base')

    rviz_config = LaunchConfiguration('rviz_config')

    lego_loam_config = os.path.join(
        lego_loam_share_dir,
        'config',
        'loam_c16_config.yaml',
    )
    mapping_config = os.path.join(
        p2p_share_dir,
        'config',
        'p2p_move_base_mapping.yaml',
    )
    default_rviz_config = os.path.join(
        p2p_share_dir,
        'rviz',
        'p2p_move_base_mapping.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='lego_loam_bor',
            executable='lego_loam',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[lego_loam_config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor2baselink',
            output='screen',
            respawn=False,
            emulate_tty=True,
            arguments=[
                '0.3',
                '0',
                '0.38',
                '-3.1415926535',
                '0.0',
                '0',
                'base_link',
                'laser_link',
            ],
        ),
        Node(
            package='global_planner',
            executable='global_planner_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[mapping_config],
        ),
        Node(
            package='p2p_move_base',
            executable='p2p_move_base_node',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[mapping_config],
        ),
        Node(
            package='p2p_move_base',
            executable='clicked2goal.py',
            output='screen',
            respawn=False,
            emulate_tty=True,
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            respawn=False,
            emulate_tty=True,
            arguments=['-d', rviz_config],
        ),
    ])
