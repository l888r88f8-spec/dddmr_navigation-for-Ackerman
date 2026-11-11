"""Standalone local-map process with its real-input RViz view.

The process is the regular `local_map_node` entry point. It owns only its TF
listener and publishes diagnostic occupancy/surface clouds; it never publishes
a command or velocity. RViz is part of this formal entry and defaults to on, in
the same namespace as the map node. `local_map_shadow.launch.py` remains
available as the compatibility entry point.

The bundled `rviz/local_map.rviz` contains only TF and the two namespaced
diagnostics, so it never subscribes to the demo's synthetic input. RViz's Fixed
Frame follows the effective map frame: the `map_frame` launch override when
provided, otherwise `local_map.frames.map` from `config_file`.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_DEFAULT_MAP_FRAME = "odom"


def _optional_override(context, argument, parameter):
    value = LaunchConfiguration(argument).perform(context).strip()
    return {parameter: value} if value else {}


def _configured_map_frame(config_file):
    """Read local_map.frames.map from a ROS parameter file.

    The launch file is not a parameter server, so the map frame that will be
    effective inside the node must be read here to give RViz the same Fixed
    Frame. Missing or malformed values fall back to the documented default.
    """
    try:
        with open(config_file, encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(
            f"cannot read local_map config_file {config_file}: {error}"
        ) from error

    if not isinstance(document, dict):
        return _DEFAULT_MAP_FRAME
    for node_parameters in document.values():
        if not isinstance(node_parameters, dict):
            continue
        parameters = node_parameters.get("ros__parameters")
        if not isinstance(parameters, dict):
            continue
        local_map = parameters.get("local_map")
        if not isinstance(local_map, dict):
            continue
        frames = local_map.get("frames")
        if not isinstance(frames, dict):
            continue
        map_frame = frames.get("map")
        if isinstance(map_frame, str) and map_frame.strip():
            return map_frame.strip()
    return _DEFAULT_MAP_FRAME


def _launch_node(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    if not os.path.isfile(config_file):
        raise RuntimeError(
            f"local_map config_file does not exist or is not a file: {config_file}"
        )

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    map_frame_override = LaunchConfiguration("map_frame").perform(context).strip()

    overrides = {}
    overrides.update(
        _optional_override(context, "cloud_topic", "local_map.cloud_topic")
    )
    overrides.update(
        _optional_override(
            context, "sensor_frame", "local_map.frames.sensor"
        )
    )
    overrides.update(
        _optional_override(context, "base_frame", "local_map.frames.base")
    )
    overrides.update(
        _optional_override(context, "map_frame", "local_map.frames.map")
    )

    parameters = [config_file, {"use_sim_time": use_sim_time}]
    if overrides:
        parameters.append(overrides)

    actions = [
        Node(
            package="dddmr_local_map",
            executable="local_map_node",
            name="local_map_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            emulate_tty=True,
            parameters=parameters,
            on_exit=Shutdown(reason="local_map_node exited"),
        )
    ]

    use_rviz = LaunchConfiguration("use_rviz").perform(context).strip().lower()
    if use_rviz in ("1", "true", "yes"):
        rviz_config = LaunchConfiguration("rviz_config").perform(context).strip()
        if not rviz_config or not os.path.isfile(rviz_config):
            raise RuntimeError(
                f"RViz config does not exist or is not a file: {rviz_config}"
            )

        effective_map_frame = (
            map_frame_override or _configured_map_frame(config_file)
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                emulate_tty=True,
                # The file uses relative diagnostic topics, so the shared
                # namespace is enough. -f matches the frame local_map_node uses
                # without rewriting the installed RViz file.
                arguments=["-d", rviz_config, "-f", effective_map_frame],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return actions


def generate_launch_description():
    package_share = get_package_share_directory("dddmr_local_map")
    default_config = os.path.join(package_share, "config", "local_map.yaml")
    default_rviz = os.path.join(package_share, "rviz", "local_map.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "YAML file containing the local_map.* parameters. The "
                    "0.05 m comparison profile is config/local_map_005.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the /clock topic for ROS time",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description=(
                    "Optional ROS namespace. Diagnostics are then published "
                    "under <namespace>/local_map/* and RViz shares it."
                ),
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="",
                description=(
                    "Optional absolute cloud topic override. Empty keeps the "
                    "value from config_file."
                ),
            ),
            DeclareLaunchArgument(
                "sensor_frame",
                default_value="",
                description=(
                    "Optional sensor frame override. Empty keeps the value from "
                    "config_file."
                ),
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="",
                description=(
                    "Optional base frame override. Empty keeps the value from "
                    "config_file."
                ),
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="",
                description=(
                    "Optional map frame override. Empty keeps the value from "
                    "config_file."
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description=(
                    "Start the real-input RViz view. Set false for headless or "
                    "CI operation; local_map_node still runs."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description=(
                    "RViz configuration used by the real-input entry. The "
                    "default shows only TF and the namespaced local-map "
                    "diagnostics."
                ),
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )
