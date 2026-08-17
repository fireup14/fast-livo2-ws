"""Launch the LiDAR-to-D405 RGB projection validation node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Use the same FAST-LIVO2 calibration files as the mapping node."""
    fast_livo_share = get_package_share_directory("fast_livo")
    return LaunchDescription([
        Node(
            package="lidar_camera_projection_check",
            executable="projection_check_node",
            name="lidar_camera_projection_check",
            output="screen",
            parameters=[
                os.path.join(fast_livo_share, "config", "livo.yaml"),
                os.path.join(fast_livo_share, "config", "d405.yaml"),
            ],
        ),
    ])
