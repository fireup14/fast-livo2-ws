from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
        description="标定与转换节点共用的配置文件路径",
    )

    calibrate_node = Node(
        package="radar_tilt_calibration",
        executable="calibrate_tilt_node",
        name="calibrate_tilt_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config")}],
    )

    launch_message = LogInfo(
        msg=[
            "Starting tilt calibration; x/y/z angles will be written in degrees. Config: ",
            LaunchConfiguration("config"),
        ]
    )

    return LaunchDescription([config_argument, launch_message, calibrate_node])
