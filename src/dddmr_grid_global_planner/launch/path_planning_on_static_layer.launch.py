import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    grid_share = get_package_share_directory('grid_global_planner')
    params_file = os.path.join(grid_share, 'config', 'grid_global_planner.yaml')
    visualization_file = os.path.join(
        grid_share, 'config', 'grid_visualization.yaml')
    with open(params_file, 'r', encoding='utf-8') as params_stream:
        common_params = yaml.safe_load(params_stream)['/**']['ros__parameters']
    global_frame = common_params['global_frame']
    robot_base_frame = common_params['robot_base_frame']
    default_rviz_config = os.path.join(grid_share, 'rviz', 'grid_global_planner.rviz')
    map_path = LaunchConfiguration('map_path')

    use_sim = LaunchConfiguration('use_sim')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = ParameterValue(use_sim, value_type=bool)

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
            'map_path',
            default_value='/home/robot/map/hesai_indoor',
            description='Directory containing GlobalMap.pcd and GroundMap.pcd.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to the RViz configuration file.',
        ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # One publisher loads the files; the planner consumes its latched clouds.
        Node(
            package='mcl_3dl',
            executable='pcl_publisher',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                visualization_file,
                {
                    'map_dir': PathJoinSubstitution([map_path, 'GlobalMap.pcd']),
                    'ground_dir': PathJoinSubstitution([map_path, 'GroundMap.pcd']),
                    'global_frame': global_frame,
                    'map_topic': common_params['map_topic'],
                    'ground_topic': common_params['ground_topic'],
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        # Display-only offline pose. The bridge carries explicit start/goal
        # poses, so the planner does not need this transform for planning.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
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
                '--frame-id', global_frame,
                '--child-frame-id', robot_base_frame,
            ],
            parameters=[
                visualization_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
        # The only /get_plan provider; static map input comes from ROS topics.
        Node(
            package='grid_global_planner',
            executable='grid_global_planner_node',
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
            package='grid_global_planner',
            executable='rviz_goal_bridge.py',
            name='grid_rviz_bridge',
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                visualization_file,
                {
                    'fixed_frame': global_frame,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config, '-f', global_frame],
            output='screen',
            respawn=False,
            emulate_tty=True,
            parameters=[
                visualization_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
