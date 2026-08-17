"""Start sensor drivers, optional image viewer, and projection checking."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the complete projection-validation bringup."""
    enable_rqt_arg = DeclareLaunchArgument(
        "enable_rqt",
        default_value="true",
        description="Start rqt_image_view for /lidar_projection/image.",
    )
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("top_pkg"),
                "launch",
                "bringup_sensor.launch.py",
            ),
        ]),
        launch_arguments={
            "enable_lidar": "true",
            "enable_camera": "true",
            "enable_rviz": "false",
        }.items(),
    )
    image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rqt")),
    )
    fast_livo_share = get_package_share_directory("fast_livo")
    projection_check = Node(
        package="lidar_camera_projection_check",
        executable="projection_check_node",
        name="lidar_camera_projection_check",
        output="screen",
        parameters=[
            os.path.join(fast_livo_share, "config", "livo.yaml"),
            os.path.join(fast_livo_share, "config", "d405.yaml"),
        ],
    )
    return LaunchDescription([
        enable_rqt_arg,
        sensor_bringup,
        image_view,
        projection_check,
    ])
