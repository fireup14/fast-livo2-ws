from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("radar_tilt_calibration")
    default_config = os.path.join(
        package_share, "config", "calibrate_tilt_param.yaml"
    )

    config_argument = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to calibrate_tilt_param.yaml",
    )

    transform_node = Node(
        package="radar_tilt_calibration",
        executable="transform_to_world_node",
        name="transform_to_world_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config")}],
    )

    return LaunchDescription([config_argument, transform_node])
