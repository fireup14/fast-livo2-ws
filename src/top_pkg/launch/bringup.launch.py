"""Launch sensor bringup and FAST-LIVO2 MID-360 mapping together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start sensors, FAST-LIVO2 mapping, and the top-level RViz view."""
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
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Whether to start the top-level bringup RViz.",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="bringup_rviz",
        arguments=["-d", os.path.join(top_pkg_share, "config", "bringup.rviz")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )

    # Start this node before including child launches.  Both child launch files
    # use an enable_rviz argument which is explicitly set to false above.
    return LaunchDescription([
        enable_rviz_arg,
        rviz,
        sensor_bringup,
        fast_livo_mapping,
    ])
