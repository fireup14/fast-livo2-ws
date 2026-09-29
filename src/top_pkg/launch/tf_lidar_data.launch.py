"""Convert Livox PointXYZRTLT PointCloud2 messages to Livox CustomMsg."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the LiDAR message conversion node."""
    input_topic_arg = DeclareLaunchArgument(
        "input_topic",
        default_value="/livox/lidar_points",
        description="Input Livox PointXYZRTLT PointCloud2 topic.",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/livox/lidar",
        description="Output livox_ros_driver2/msg/CustomMsg topic.",
    )
    timestamp_mode_arg = DeclareLaunchArgument(
        "timestamp_mode",
        default_value="absolute_ns",
        description=(
            "Meaning of PointCloud2 timestamp: absolute_ns, relative_ns, "
            "absolute_sec, or relative_sec."
        ),
    )
    lidar_id_arg = DeclareLaunchArgument(
        "lidar_id",
        default_value="0",
        description="Livox lidar_id stored in the generated CustomMsg.",
    )

    converter = Node(
        package="top_pkg",
        executable="tf_lidar_data",
        name="tf_lidar_data",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "timestamp_mode": LaunchConfiguration("timestamp_mode"),
                "lidar_id": ParameterValue(
                    LaunchConfiguration("lidar_id"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            input_topic_arg,
            output_topic_arg,
            timestamp_mode_arg,
            lidar_id_arg,
            converter,
        ]
    )
