from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim = LaunchConfiguration("use_sim")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("dddmr_web_control"),
                        "config", "web_control.yaml",
                    ]
                ),
                description="YAML parameter file for dddmr_web_control.",
            ),
            Node(
                package="dddmr_web_control",
                executable="web_control_node",
                name="dddmr_web_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config_file,
                    {"use_sim": use_sim}
                ],
            ),
        ]
    )
