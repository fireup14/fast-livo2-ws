"""传感器驱动与 FAST-LIVO2 建图定位一键联合启动文件。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """同时启动传感器硬件驱动、FAST-LIVO2 建图定位算法及顶层 RViz 可视化。"""
    top_pkg_share = get_package_share_directory("top_pkg")
    fast_livo_share = get_package_share_directory("fast_livo")

    # 传感器启动文件统一管理三个模块开关及 RViz。
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(top_pkg_share, "launch", "bringup_sensor.launch.py")
        )
    )

    # 包含 FAST-LIVO2 建图定位算法启动文件（同样屏蔽其内部单独打开的 RViz）
    fast_livo_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )

    return LaunchDescription([
        sensor_bringup,
        fast_livo_mapping,
    ])
