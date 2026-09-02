"""传感器驱动与 FAST-LIVO2 建图定位一键联合启动文件。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """同时启动传感器硬件驱动、FAST-LIVO2 建图定位算法及顶层 RViz 可视化。"""
    top_pkg_share = get_package_share_directory("top_pkg")
    fast_livo_share = get_package_share_directory("fast_livo")

    # 包含传感器驱动启动文件（强制屏蔽其内部单独打开的 RViz，统一由顶层控制）
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(top_pkg_share, "launch", "bringup_sensor.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )

    # 包含 FAST-LIVO2 建图定位算法启动文件（同样屏蔽其内部单独打开的 RViz）
    fast_livo_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )

    # 声明顶层 RViz2 启动开关参数
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="是否启动顶层统一的 RViz2 可视化界面 (true/false)。",
    )

    # 顶层统一的 RViz2 节点（加载综合视图配置文件 bringup.rviz）
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="bringup_rviz",
        arguments=["-d", os.path.join(top_pkg_share, "config", "bringup.rviz")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
    )

    return LaunchDescription([
        enable_rviz_arg,
        rviz,
        sensor_bringup,
        fast_livo_mapping,
    ])
