"""Launch a display-only Foxglove Bridge independently of FAST-LIVO2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the Foxglove Bridge launch description."""
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    log_level = LaunchConfiguration("log_level")

    # Restrict the bridge to visualization data used by this project.  This
    # prevents an accidental subscription to large raw LiDAR/IMU topics.
    topic_whitelist = [
        r"^/aft_mapped_to_init$",
        r"^/cloud_registered$",
        r"^/path$",
        r"^/camera/camera/color/image_raw$",
        r"^/tf$",
        r"^/tf_static$",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="Foxglove WebSocket TCP port.",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Local address on which Foxglove Bridge listens.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="ROS log level for the bridge node.",
            ),
            LogInfo(
                msg=[
                    "Starting Foxglove Bridge. Connect Foxglove Studio to ws://",
                    address,
                    ":",
                    port,
                ]
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "address": address,
                        "port": port,
                        "topic_whitelist": topic_whitelist,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
