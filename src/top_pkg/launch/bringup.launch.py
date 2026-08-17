"""Launch sensor bringup and FAST-LIVO2 MID-360 mapping together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Start top_pkg sensors (without RViz) and FAST-LIVO2 mapping."""
    top_pkg_share = get_package_share_directory("top_pkg")
    fast_livo_share = get_package_share_directory("fast_livo")

    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(top_pkg_share, "launch", "bringup_sensor.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )

    fast_livo_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, "launch", "mapping.launch.py")
        )
    )

    return LaunchDescription([
        sensor_bringup,
        fast_livo_mapping,
    ])
