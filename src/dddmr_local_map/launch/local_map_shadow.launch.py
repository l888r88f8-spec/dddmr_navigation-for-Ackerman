"""Shadow local-map process.

Runs the reusable ROS adapter against its own TF listener and visualizes
occupancy/support clouds only when those topics have subscribers. It never
publishes a command or velocity.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("dddmr_local_map")
    default_config = os.path.join(package_share, "config", "local_map.yaml")

    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML file containing the local_map.* parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the /clock topic for ROS time",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace for the shadow node",
            ),
            Node(
                package="dddmr_local_map",
                executable="local_map_shadow",
                name="local_map_shadow",
                namespace=namespace,
                output="screen",
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
