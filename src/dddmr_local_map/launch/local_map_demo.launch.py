"""Self-contained synthetic local-map demo.

Starts the regular `local_map_node` (not a second mapping loop), a deterministic
synthetic point-cloud/TF source, and optionally RViz. No real LiDAR, bag,
navigation stack, or vehicle interface is required. The synthetic input is
published on its own topic and is never labeled as real sensor data.

The demo intentionally has no `namespace` argument: its RViz file uses absolute
topic names. Use `local_map.launch.py` for a namespaced real-input process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_demo(context):
    cloud_topic = LaunchConfiguration("demo_cloud_topic").perform(context)
    if not cloud_topic.startswith("/"):
        raise RuntimeError(
            "demo_cloud_topic must be an absolute topic name so RViz and "
            f"local_map_node agree; got '{cloud_topic}'"
        )

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    sensor_frame = LaunchConfiguration("sensor_frame").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    if not os.path.isfile(config_file):
        raise RuntimeError(
            f"local_map config_file does not exist or is not a file: {config_file}"
        )

    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    if use_rviz and not os.path.isfile(rviz_config):
        raise RuntimeError(f"RViz config does not exist: {rviz_config}")

    local_map_share = get_package_share_directory("dddmr_local_map")
    local_map_launch = os.path.join(
        local_map_share, "launch", "local_map.launch.py"
    )
    demo_node = Node(
        package="dddmr_local_map",
        executable="local_map_demo_scene",
        name="local_map_demo_scene",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "cloud_topic": cloud_topic,
                "map_frame": "odom",
                "base_frame": "demo_base_link",
                "sensor_frame": sensor_frame,
                "sensor_height": 1.4,
                "azimuth_samples": ParameterValue(
                    LaunchConfiguration("demo_azimuth_samples"), value_type=int
                ),
                "range_samples": ParameterValue(
                    LaunchConfiguration("demo_range_samples"), value_type=int
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("demo_rate_hz"), value_type=float
                ),
                "moving_obstacle": ParameterValue(
                    LaunchConfiguration("demo_moving_obstacle"),
                    value_type=bool,
                ),
                "moving_obstacle_period_sec": ParameterValue(
                    LaunchConfiguration("demo_moving_obstacle_period_sec"),
                    value_type=float,
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
        on_exit=Shutdown(reason="local_map_demo_scene exited"),
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_map_launch),
            launch_arguments={
                "config_file": config_file,
                "use_sim_time": use_sim_time,
                "cloud_topic": cloud_topic,
                "sensor_frame": sensor_frame,
                "base_frame": "demo_base_link",
                "map_frame": "odom",
                # The demo owns the only RViz window below. Disable the formal
                # entry's new default view so this launch never opens two.
                "use_rviz": "false",
            }.items(),
        ),
        demo_node,
    ]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                emulate_tty=True,
                arguments=["-d", rviz_config],
                # The bundled RViz file subscribes to the default synthetic
                # topic. Remap that subscription when demo_cloud_topic is
                # overridden so the displayed input matches local_map_node.
                remappings=[("/local_map_demo/points", cloud_topic)],
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        )
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("dddmr_local_map")
    default_config = os.path.join(package_share, "config", "local_map.yaml")
    default_rviz = os.path.join(package_share, "rviz", "local_map_demo.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "Base local-map YAML. Defaults to the unchanged 0.10 m "
                    "profile; config/local_map_005.yaml is the 0.05 m "
                    "diagnostic comparison."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use /clock for both nodes. An external clock is required "
                    "when this is true."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description=(
                    "Start RViz. Set false for headless/CI operation; the map "
                    "and synthetic source still run."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz configuration for the demo.",
            ),
            DeclareLaunchArgument(
                "demo_cloud_topic",
                default_value="/local_map_demo/points",
                description=(
                    "Absolute synthetic PointCloud2 topic. This is independent "
                    "of the real vehicle cloud."
                ),
            ),
            DeclareLaunchArgument(
                "sensor_frame",
                default_value="demo_lidar",
                description=(
                    "Synthetic sensor frame. The same value is used by the TF "
                    "source and local_map_node."
                ),
            ),
            DeclareLaunchArgument(
                "demo_rate_hz",
                default_value="5.0",
                description="Synthetic cloud publication rate.",
            ),
            DeclareLaunchArgument(
                "demo_azimuth_samples",
                default_value="180",
                description=(
                    "Synthetic azimuth sample count. The default 180 x 24 = "
                    "4,320 rays keeps the unchanged 0.05 s freshness threshold "
                    "at both the 0.10 m and 0.05 m demo profiles."
                ),
            ),
            DeclareLaunchArgument(
                "demo_range_samples",
                default_value="24",
                description=(
                    "Synthetic radial sample count. See demo_azimuth_samples."
                ),
            ),
            DeclareLaunchArgument(
                "demo_moving_obstacle",
                default_value="true",
                description=(
                    "Include a deterministic moving box so map updates are "
                    "visible. This is not a prediction or tracked object."
                ),
            ),
            DeclareLaunchArgument(
                "demo_moving_obstacle_period_sec",
                default_value="8.0",
                description="Period of the deterministic moving-box path.",
            ),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            OpaqueFunction(function=_launch_demo),
        ]
    )
