import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import (
    ComposableNodeContainer,
    Node,
)

from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    fast_livo_share = get_package_share_directory("fast_livo")

    livo_config = os.path.join(
        fast_livo_share,
        "config",
        "livo.yaml",
    )

    d405_config = os.path.join(
        fast_livo_share,
        "config",
        "d405.yaml",
    )


    # =====================================================
    # RealSense D405 Component
    # =====================================================

    realsense_component = ComposableNode(
        package="realsense2_camera",

        plugin="realsense2_camera::RealSenseNodeFactory",

        name="camera",
        namespace="camera",

        parameters=[{
            "camera_name": "camera",
            "camera_namespace": "camera",

            "device_type": "d405",

            "enable_color": True,
            "enable_depth": False,

            "pointcloud.enable": False,
            "align_depth.enable": False,

            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,

            "depth_module.color_profile":
                "1280,720,30",

            "color_qos": "SENSOR_DATA",
            "color_info_qos": "SENSOR_DATA",
        }],

        extra_arguments=[
            {
                "use_intra_process_comms": True
            }
        ],
    )


    # =====================================================
    # Projection Component
    # =====================================================

    projection_component = ComposableNode(
        package="lidar_camera_projection_check",

        plugin=(
            "lidar_camera_projection_check::"
            "LidarCameraProjectionCheck"
        ),

        name="lidar_camera_projection_check",

        parameters=[
            livo_config,
            d405_config,
        ],

        extra_arguments=[
            {
                "use_intra_process_comms": True
            }
        ],
    )


    # =====================================================
    # Component Container
    # =====================================================

    image_container = ComposableNodeContainer(
        name="image_container",

        namespace="",

        package="rclcpp_components",

        executable="component_container_mt",

        composable_node_descriptions=[
            realsense_component,
            projection_component,
        ],

        output="screen",
    )


    return LaunchDescription([
        image_container,
    ])
