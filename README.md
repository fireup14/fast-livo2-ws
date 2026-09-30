# FAST-LIVO2 ROS 2 集成工作区 (Livox MID-360 + RealSense D405)

本项目是一个集成了 **Livox MID-360 雷达** 与 **Intel RealSense D405 双目微距相机** 的 **FAST-LIVO2 (Fast, Direct LiDAR-Inertial-Visual Odometry)** ROS 2 实时建图与定位工作区。工作区已完成底层驱动集成、外参适配、算法调试与顶层一键启动封装。

---

## Quick Start

RDK S100P（ROS 2 Humble）上首次构建并启动实时建图：

```bash
cd ~/Desktop/fast-livo2-ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```

常用入口：

```bash
# 仅启动 MID360、D405 及可选 RViz，不启动建图
ros2 launch top_pkg bringup_sensor.launch.py

# 订阅外部工程回放的 LiDAR/IMU/图像话题并在本地重新建图
ros2 launch top_pkg rerun_bringup.launch.py

# 临时打开项目级 RViz（命令行优先于 YAML 默认值）
ros2 launch top_pkg rerun_bringup.launch.py enable_rviz:=true

# 将 Livox PointXYZRTLT PointCloud2 转为 FAST-LIVO2 使用的 CustomMsg
ros2 launch top_pkg tf_lidar_data.launch.py

# 独立启动 Foxglove WebSocket Bridge
ros2 launch foxglove_bridge_bringup foxglove_bridge.launch.py
```

默认传感器输入：

```text
/livox/lidar                         livox_ros_driver2/msg/CustomMsg
/livox/imu                           sensor_msgs/msg/Imu
/camera/camera/color/image_raw       sensor_msgs/msg/Image
```

如果外部回放输出 Livox `PointCloud2`，请发布到 `/livox/lidar_points`，转换节点会在 `/livox/lidar` 输出 `CustomMsg`。不要让两种消息类型共用同一个话题名。

---

## 目录指南

- [Quick Start](#quick-start)
- [1. 工作区架构](#1-工作区架构)
- [2. 环境依赖](#2-环境依赖)
  - [2.1 操作系统与 ROS 2 平台](#21-操作系统与-ros-2-平台)
  - [2.2 ROS 2 系统依赖包 (APT 自适应安装)](#22-ros-2-系统依赖包-apt-自适应安装)
  - [2.3 第三方 C++ 基础与数学库](#23-第三方-c-基础与数学库)
  - [2.4 传感器底层硬件 SDK 驱动](#24-传感器底层硬件-sdk-驱动-sensor-sdks)

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
    ├── rpg_vikit/          # [依赖项] 视觉运动学工具包 (Visual Kinematics Toolkit)
    └── foxglove_bridge_bringup/ # [远程显示] Foxglove WebSocket Bridge 启动包
```

---

## 2. 环境依赖

为保证工作区成功编译与运行，系统需提前安装以下软件环境与依赖库：

### 2.1 操作系统与 ROS 2 平台

- **RDK S100P 部署端**：ROS 2 Humble
- **PC 开发端**：可使用 ROS 2 Jazzy；建议通过 Foxglove WebSocket 查看 RDK 数据

Humble 与 Jazzy 不保证直接 DDS 互操作。实时建图、话题频率检查及 `CustomMsg` 诊断优先在 RDK Humble 本机执行。

### 2.2 ROS 2 系统依赖包 (APT 自适应安装)
在编译工作区前，请先使用 APT 安装以下 ROS 2 扩展依赖包。以下命令会自动读取当前终端已加载的 `$ROS_DISTRO` 环境变量（自动适配 `jazzy`、`humble` 等不同 ROS 2 版本）：

```bash
# 自动检测当前 ROS 2 发行版并一键安装对应依赖包
sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-image-transport
```

> [!TIP]
> **版本适配说明**：
> - 若您已 `source /opt/ros/<distro>/setup.bash`（例如 `humble` 或 `jazzy`），该命令会自动匹配安装对应的 `ros-humble-*` 或 `ros-jazzy-*` 包。
> - 若尚未加载 ROS 环境变量，也可以手动在命令行先指定版本，例如：`ROS_DISTRO=humble` 再运行安装。

### 2.3 第三方 C++ 基础与数学库
- **PCL (Point Cloud Library)** >= 1.12.1（用于点云数据预处理与滤波，依赖 `pcl-ros` / `pcl-conversions`）

  **安装步骤**：

  ```bash
  sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libboost-all-dev \
    libflann-dev \
    libvtk9-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libopenni2-dev

  sudo add-apt-repository universe
  sudo apt update

  mkdir -p ~/third_party
  cd ~/third_party
  git clone --depth 1 --branch pcl-1.14.0 \
    https://github.com/PointCloudLibrary/pcl.git pcl-1.14.0

  cd ~/third_party/pcl-1.14.0
  mkdir build
  cd build
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/pcl-1.14.0 \
    -DBUILD_examples=OFF \
    -DBUILD_tools=OFF \
    -DBUILD_apps=OFF \
    -DBUILD_global_tests=OFF \
    -DBUILD_visualization=OFF \
    -DWITH_OPENGL=OFF \
    -DWITH_QT=OFF

  make -j2
  sudo make install
  find /opt/pcl-1.14.0 -name "PCLConfig.cmake"
  ```

- **Eigen** == 3.4.0（用于矩阵运算与线性代数求解）

  ```bash
  sudo apt install -y libeigen3-dev
  ```

- **OpenCV** >= 4.5.4（用于视觉图像处理与金字塔生成）

  ```bash
  sudo apt install libopencv-dev
  ```

- **Sophus** == 1.22.10（用于三维空间李群与李代数转换）

  **安装步骤**：

  ```bash
  git clone https://github.com/strasdat/Sophus.git -b 1.22.10
  cd Sophus && mkdir build && cd build
  cmake .. && make -j$(nproc)
  sudo make install
  ```

  > [!TIP]
  > **已知编译报错及修补补丁 (GCC 13/C++17 兼容修补)**：
  > 
  > 若编译 Sophus 时报 `so2.cpp:32:26: error: lvalue required as left operand of assignment` 错误，请修改 `so2.cpp`：
  > 
  > **`so2.cpp`**
  > ```diff
  > SO2::SO2()
  > {
  > -  unit_complex_.real() = 1.;
  > -  unit_complex_.imag() = 0.;
  > +  unit_complex_.real(1.);
  > +  unit_complex_.imag(0.);
  > }
  > ```

### 2.4 传感器底层硬件 SDK 驱动 (Sensor SDKs)
- **Livox-SDK2**: Livox MID-360 雷达底层硬件通信接口库，参考 [Livox-SDK2 官方仓库](https://github.com/Livox-SDK/Livox-SDK2)
- **librealsense2**: Intel RealSense 相机底层硬件 SDK 驱动库，参考 [librealsense 官方仓库](https://github.com/IntelRealSense/librealsense)（依赖 `diagnostic-updater` / `image-transport`）

---
