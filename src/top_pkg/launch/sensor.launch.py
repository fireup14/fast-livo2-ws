"""Launch realsense camera and click measure node."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    realsense_share = get_package_share_directory('realsense2_camera')

    # Launch RealSense camera with color and depth only (no pointcloud)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'device_type': 'd405',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',  # Disable pointcloud
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            # Use standard resolution for better quality
            'rgb_camera.color_profile': '1280,720,30',
            'depth_module.depth_profile': '1280,720,30',
            'log_level': 'warn',
        }.items(),
    )

    # Launch click measure node with OpenCV window
    click_measure = Node(
        package='realsense_click_measure',
        executable='click_measure_node',
        name='click_measure_node',
        output='screen',
        parameters=[
            {'color_topic': '/camera/camera/color/image_raw'},
            {'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw'},
            {'camera_info_topic': '/camera/camera/color/camera_info'},
        ],
    )

    return LaunchDescription([camera, click_measure])