import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_bringup_config():
    config_path = os.path.join(
        get_package_share_directory("top_pkg"),
        "config",
        "bringup.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["bringup"]


def generate_launch_description():

    config = load_bringup_config()
    livox_cfg = config["livox"]
    livox_config_path = os.path.join(
        get_package_share_directory("livox_ros_driver2"),
        "config",
        livox_cfg["config_file"],
    )

    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar",
        default_value=str(config["enable_lidar"]).lower(),
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value=str(config["enable_camera"]).lower(),
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value=str(config["enable_rviz"]).lower(),
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": livox_cfg["publish_freq"]},
            {"output_data_type": 0},
            {"frame_id": livox_cfg["frame_id"]},
            {"user_config_path": livox_config_path},
            {"cmdline_input_bd_code": livox_cfg["cmdline_input_bd_code"]},
            {"lvx_file_path": livox_cfg["lvx_file_path"]},
        ],
    )

    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "device_type": "d405",
            "enable_depth": "false",
            "enable_color": "true",
            "enable_sync": "false",
            "align_depth.enable": "false",
            "pointcloud.enable": "false",
            "spatial_filter.enable": "false",
            "temporal_filter.enable": "false",
            "rgb_camera.color_profile": "1280,720,30",
            "depth_module.depth_profile": "1280,720,30",
            "log_level": "warn",

        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="calibration_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", os.path.join(
            get_package_share_directory("top_pkg"), "config", "bringup.rviz")],
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_rviz_arg,
        livox_driver,
        realsense_driver,
        rviz,
    ])
