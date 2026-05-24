from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    map_frame = LaunchConfiguration("map_frame")
    global_map_topic = LaunchConfiguration("global_map_topic")
    mapcloud_topic = LaunchConfiguration("mapcloud_topic")
    mapground_topic = LaunchConfiguration("mapground_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")
    goal_topic = LaunchConfiguration("goal_topic")
    action_status_topic = LaunchConfiguration("action_status_topic")
    path_topics = LaunchConfiguration("path_topics")
    max_2d_points_per_layer = LaunchConfiguration("max_2d_points_per_layer")
    max_3d_points_per_layer = LaunchConfiguration("max_3d_points_per_layer")
    stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")
    use_sim = LaunchConfiguration("use_sim")
    localization_launch_args = LaunchConfiguration("localization_launch_args")
    navigation_launch_args = LaunchConfiguration("navigation_launch_args")
    mapping_launch_args = LaunchConfiguration("mapping_launch_args")
    managed_log_lines = LaunchConfiguration("managed_log_lines")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument(
                "global_map_topic",
                default_value="/lio_sam/localization/global_map",
            ),
            DeclareLaunchArgument("mapcloud_topic", default_value="mapcloud"),
            DeclareLaunchArgument("mapground_topic", default_value="/weighted_ground"),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/lio_sam/localization/odometry",
            ),
            DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),
            DeclareLaunchArgument("goal_topic", default_value="goal_pose_3d"),
            DeclareLaunchArgument(
                "action_status_topic",
                default_value="/p2p_move_base/_action/status",
            ),
            DeclareLaunchArgument(
                "path_topics",
                default_value="/global_path",
                description="Comma-separated nav_msgs/Path topics to visualize. Default subscribes to none.",
            ),
            DeclareLaunchArgument("max_2d_points_per_layer", default_value="60000"),
            DeclareLaunchArgument("max_3d_points_per_layer", default_value="40000"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument(
                "localization_launch_args",
                default_value="",
                description="Extra ros2 launch arguments for localization.",
            ),
            DeclareLaunchArgument(
                "navigation_launch_args",
                default_value="",
                description="Extra ros2 launch arguments for navigation.",
            ),
            DeclareLaunchArgument(
                "mapping_launch_args",
                default_value="",
                description="Extra ros2 launch arguments for mapping.",
            ),
            DeclareLaunchArgument("managed_log_lines", default_value="400"),
            Node(
                package="dddmr_web_control",
                executable="web_control_node",
                name="dddmr_web_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "map_frame": map_frame,
                        "global_map_topic": global_map_topic,
                        "mapcloud_topic": mapcloud_topic,
                        "mapground_topic": mapground_topic,
                        "odom_topic": odom_topic,
                        "initialpose_topic": initialpose_topic,
                        "goal_topic": goal_topic,
                        "action_status_topic": action_status_topic,
                        "path_topics": path_topics,
                        "max_2d_points_per_layer": max_2d_points_per_layer,
                        "max_3d_points_per_layer": max_3d_points_per_layer,
                        "stale_timeout_sec": stale_timeout_sec,
                        "use_sim": use_sim,
                        "localization_launch_args": localization_launch_args,
                        "navigation_launch_args": navigation_launch_args,
                        "mapping_launch_args": mapping_launch_args,
                        "managed_log_lines": managed_log_lines,
                    }
                ],
            ),
        ]
    )
