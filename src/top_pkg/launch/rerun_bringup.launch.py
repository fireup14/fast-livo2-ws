"""Launch FAST-LIVO2 against sensor topics supplied by an external replayer.

This launch file intentionally does not start Livox, RealSense, or rosbag2.
Start it first, then start the external replay publisher so FAST-LIVO2 receives
the complete IMU initialization interval.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_rerun_config():
    """Load replay defaults installed with top_pkg."""
    config_path = os.path.join(
        get_package_share_directory("top_pkg"),
        "config",
        "rerun_bringup.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)["rerun"]


def generate_launch_description():
    """Start mapping only; all sensor messages must come from external topics."""
    config = load_rerun_config()
    top_pkg_share = get_package_share_directory("top_pkg")
    fast_livo_share = get_package_share_directory("fast_livo")

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value=str(config["enable_rviz"]).lower(),
        description="Whether to start RViz2 on this host.",
    )
    # Keep mapping startup identical to bringup.launch.py. The three input
    # topics remain defined in fast_livo/config/livo.yaml rather than here.
    fast_livo_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )

    # Use the project-level RViz layout, matching bringup.launch.py rather
    # than the RViz configuration embedded in fast_livo/mapping.launch.py.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="calibration_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=[
            "-d",
            os.path.join(top_pkg_share, "config", "bringup.rviz"),
        ],
    )

    return LaunchDescription(
        [
            enable_rviz_arg,
            LogInfo(
                msg=(
                    "Replay mapping launch requested. Start the external replay "
                    "publisher only after fastlivo_mapping has completed initialization."
                )
            ),
            fast_livo_mapping,
            rviz,
        ]
    )
