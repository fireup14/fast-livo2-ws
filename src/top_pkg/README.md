# top_pkg

`top_pkg` 是工作区（FAST-LIVO2 Workspace）的顶层总控 ROS 2 功能包，用于集中管理传感器驱动（Livox MID-360 LiDAR、RealSense D405 Camera）、RViz 可视化及 FAST-LIVO2 建图定位系统的统一启动与参数配置。

---

## Quick Start

```bash
cd ~/Desktop/fast-livo2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to top_pkg --symlink-install
source install/setup.bash

# 实时传感器 + FAST-LIVO2
ros2 launch top_pkg bringup.launch.py

# 已有话题回放 + FAST-LIVO2（本文件不负责发布回放数据）
ros2 launch top_pkg rerun_bringup.launch.py

# Livox PointCloud2 → CustomMsg
ros2 launch top_pkg tf_lidar_data.launch.py
```

RViz 默认值来自对应 YAML，命令行具有最高优先级：

```bash
ros2 launch top_pkg bringup.launch.py enable_rviz:=true
ros2 launch top_pkg rerun_bringup.launch.py enable_rviz:=true
```

---

## 目录结构

```text
top_pkg/
├── CMakeLists.txt         # 编译及安装配置
├── package.xml            # 包依赖定义
├── config/
│   ├── bringup_sensor.yaml      # 实时模式默认开关
│   ├── rerun_bringup.yaml       # 回放模式默认开关
│   └── bringup.rviz             # 项目级 RViz 视图
├── include/top_pkg/
│   └── tf_lidar_data.hpp        # 点云消息转换节点声明
├── src/
│   └── tf_lidar_data.cpp        # PointCloud2 → CustomMsg 实现
└── launch/
    ├── bringup_sensor.launch.py # 仅传感器与可选 RViz
    ├── bringup.launch.py        # 传感器 + FAST-LIVO2
    ├── rerun_bringup.launch.py  # 外部回放话题 + FAST-LIVO2
    └── tf_lidar_data.launch.py  # 点云消息转换节点
```

---

## 参数配置

可以在 `config/bringup_sensor.yaml` 中配置各系统模块的默认使能开关：

```yaml
bringup:
  enable_lidar: true
  enable_camera: true
  enable_rviz: false
```

也可以通过 `ros2 launch` 命令行参数临时覆盖。

---

## 启动方式 Launch

雷达与相机均采用标准 `IncludeLaunchDescription` 方式分别调用官方 Launch 脚本：
- **雷达**：调用 `livox_ros_driver2/launch/msg_MID360_launch.py`
- **相机**：调用 `realsense2_camera/launch/rs_launch.py`

### 1. 单独启动传感器驱动 (Sensors Only)

```bash
# 启动所有已启用的传感器和 RViz
ros2 launch top_pkg bringup_sensor.launch.py

# 命令行覆盖参数（例如关闭 RViz 或单独关闭相机）
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false enable_camera:=false
```

### 2. 整体一键启动 (Sensors + FAST-LIVO2 Mapping)

同时启动传感器驱动、FAST-LIVO2 建图定位节点及 RViz：

```bash
ros2 launch top_pkg bringup.launch.py
```

### 3. 外部话题回放建图

先启动建图订阅端，再启动外部回放发布器：

```bash
ros2 launch top_pkg rerun_bringup.launch.py
```

回放工程应发布：

```text
/livox/lidar                         livox_ros_driver2/msg/CustomMsg
/livox/imu                           sensor_msgs/msg/Imu
/camera/camera/color/image_raw       sensor_msgs/msg/Image（LIVO 模式）
```

### 4. PointCloud2 转 CustomMsg

默认输入与输出：

```text
/livox/lidar_points   sensor_msgs/msg/PointCloud2
/livox/lidar          livox_ros_driver2/msg/CustomMsg
```

```bash
ros2 launch top_pkg tf_lidar_data.launch.py
```

输入必须是 Livox PointXYZRTLT 布局，包含 `x/y/z/intensity/tag/line/timestamp`。输入和输出必须使用不同话题名，避免同一话题同时存在两种 ROS 消息类型。
