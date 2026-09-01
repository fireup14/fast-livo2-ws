# FAST-LIVO2 ROS 2 集成工作区 (Livox MID-360 + RealSense D405)

本项目是一个集成了 **Livox MID-360 雷达** 与 **Intel RealSense D405 双目微距相机** 的 **FAST-LIVO2 (Fast, Direct LiDAR-Inertial-Visual Odometry)** ROS 2 实时建图与定位工作区。工作区已完成底层驱动集成、外参适配、算法调试与顶层一键启动封装。

---

## 🚀 快速启动指令 (Quick Start Launch)

编译并设置环境变量后即可直接调用以下统一 Launch 入口：

```bash
# 进入工作区并加载环境变量
cd ~/Desktop/fast-livo2-ws
source install/setup.bash
```

### 1. 一键启动：传感器 + FAST-LIVO2 建图与定位 (推荐)
同时启动 MID-360 雷达、D405 相机、FAST-LIVO2 实时建图定位算法节点及 RViz 可视化：
```bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```

### 2. 调试启动：仅启动传感器驱动 (Sensors Only)
仅启动 MID-360 雷达与 D405 相机驱动，适合传感器硬件检查、标定对准或数据采集：
```bash
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py
```

### 3. 常用命令行参数重载
```bash
# 关闭建图或调试时的 RViz 窗口
source install/setup.bash
ros2 launch top_pkg bringup.launch.py enable_rviz:=false

# 仅启动雷达，关闭相机
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py enable_camera:=false
```

---

## 目录指南

- [1. 工作区架构](#1-工作区架构)
- [2. 环境依赖](#2-环境依赖)
- [3. 编译指南](#3-编译指南)
- [4. 参数与外参配置](#4-参数与外参配置)
- [5. Topic 话题与 QoS 说明](#5-topic-话题与-qos-说明)
- [6. 常见问题 FAQ](#6-常见问题-faq)

---

## 1. 工作区架构

工作区内部功能包结构如下：

```text
fast-livo2-ws/
└── src/
    ├── top_pkg/            # [总控包] 统一传感器驱动与 FAST-LIVO2 建图定位的启动及配置
    ├── FAST-LIVO2/         # [算法核心] 直接法激光-惯性-视觉里程计功能包 (ROS 2 包名: fast_livo)
    ├── livox_ros_driver2/  # [传感器驱动] Livox MID-360 雷达 ROS 2 驱动
    ├── realsense-ros/      # [传感器驱动] Intel RealSense 相机 ROS 2 驱动
    └── rpg_vikit/          # [依赖项] 视觉运动学工具包 (Visual Kinematics Toolkit)
```

---

## 2. 环境依赖

环境依赖与基础库要求：

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS 2 Humble / Iron
- **核心依赖库**:
  - **PCL** >= 1.12
  - **Eigen** >= 3.3.4
  - **OpenCV** >= 4.5.4
  - **Sophus**: 非模板双精度版本
  - **Livox-SDK2**: 需提前安装 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)

---

## 3. 编译指南

在工作区根目录下执行编译：

```bash
# 1. 导入 ROS 2 环境
source /opt/ros/humble/setup.bash

# 2. 编译整个工作区
cd ~/Desktop/fast-livo2-ws
colcon build --symlink-install

# 3. 刷新环境变量
source install/setup.bash
```

---

## 4. 参数与外参配置

### 4.1 顶层启动配置 (`src/top_pkg/config/bringup.yaml`)

集中管理传感器硬件与 RViz 的使能开关：

```yaml
bringup:
  enable_lidar: true      # 雷达开关
  enable_camera: true     # 相机开关
  enable_rviz: true       # RViz 可视化开关

  livox:
    frame_id: livox_frame
    publish_freq: 10.0    # 雷达发布频率 (Hz)
    config_file: MID360_config.json # 雷达配置文件路径 (相对 livox_ros_driver2/config/)
```

### 4.2 算法与相机外参配置 (`src/FAST-LIVO2/config/`)

- `livo.yaml`: FAST-LIVO2 核心滤波器参数、特征提取及地图参数。
- `mid360.yaml`: MID-360 雷达与 IMU 噪声模型及内部外参。
- `d405.yaml`: RealSense D405 相机内参及与雷达/IMU 之间的外参矩阵。

---

## 5. Topic 话题与 QoS 说明

| 传感器 / 节点 | Topic 名称 | 消息类型 | 说明 |
| :--- | :--- | :--- | :--- |
| **Livox MID-360** | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 点云原始数据 |
| **Livox MID-360** | `/livox/imu` | `sensor_msgs/msg/Imu` | 高频 IMU 惯导数据 |
| **RealSense D405**| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB 图像数据 |
| **RealSense D405**| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机标定内参 |
| **FAST-LIVO2** | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 配准后的实时点云 |
| **FAST-LIVO2** | `/Odometry` | `nav_msgs/msg/Odometry` | 实时估计位姿里程计 |

> **QoS 策略提示**：RealSense 相机节点图像数据采用 `SENSOR_DATA` (Best Effort) 策略发布，确保在高分辨率下达到最实时流体验。

---

## 6. 常见问题 FAQ

1. **相机图像在某些查看工具中显示延迟或卡顿？**
   - 检查图像订阅端的 QoS 设置，请确保使用 `SENSOR_DATA` 或 `Best Effort` 兼容策略。
   - 使用 `ros2 topic hz /camera/camera/color/image_raw` 检查真实传输频率。

2. **如何修改雷达 IP 或网络广播？**
   - 修改 `src/livox_ros_driver2/config/MID360_config.json` 中的 `host_net_info` 与 `lidar_configs`。

3. **如何重新保存或导出地图点云？**
   - FAST-LIVO2 可以在 `livo.yaml` 中配置 `pcd_save_en: true` 自动保存 `.pcd` 点云文件。
