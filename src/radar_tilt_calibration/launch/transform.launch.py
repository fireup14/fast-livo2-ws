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

    transform_node = Node(
        package="radar_tilt_calibration",
        executable="transform_to_world_node",
        name="transform_to_world_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config")}],
    )

    tf_bridge_node = Node(
        package="radar_tilt_calibration",
        executable="world_tf_bridge_node",
        name="world_tf_bridge_node",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config")}],
    )

    launch_message = LogInfo(
        msg=[
            "Starting corrected odometry and connected TF tree; "
            "the largest absolute x/y/z angle is used. Config: ",
            LaunchConfiguration("config"),
        ]
    )

    return LaunchDescription(
        [config_argument, launch_message, transform_node, tf_bridge_node]
    )
