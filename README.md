# FAST-LIVO2 ROS 2 集成工作区 (Livox MID-360 + RealSense D405)

本项目是一个集成了 **Livox MID-360 雷达** 与 **Intel RealSense D405 双目微距相机** 的 **FAST-LIVO2 (Fast, Direct LiDAR-Inertial-Visual Odometry)** ROS 2 实时建图与定位工作区。工作区已完成底层驱动集成、外参适配、算法调试与顶层一键启动封装。

---

## 目录指南

- [1. 工作区架构](#1-工作区架构)
- [2. 环境依赖](#2-环境依赖)
- [3. 快速启动指令 (Quick Start Launch)](#3-快速启动指令-quick-start-launch)

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

为保证工作区成功编译与运行，系统需提前安装以下软件环境与依赖库：

### 2.1 操作系统与 ROS 2 平台
- **操作系统**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2 版本**: ROS 2 Jazzy (Jazzy Jalisco)

### 2.2 第三方 C++ 基础与数学库
- **PCL (Point Cloud Library)** == 1.14.0（用于点云数据预处理与滤波）
- **Eigen** == 3.4.0（用于矩阵运算与线性代数求解）
- **OpenCV** == 4.6.0（用于视觉图像处理与金字塔生成）
- **Sophus** == 1.22.10（非模板双精度版本，用于三维空间李群与李代数转换）

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

### 2.3 传感器底层硬件 SDK 驱动 (Sensor SDKs)
- **Livox-SDK2**: Livox MID-360 雷达底层硬件通信接口库，参考 [Livox-SDK2 官方仓库](https://github.com/Livox-SDK/Livox-SDK2)
- **librealsense2**: Intel RealSense 相机底层硬件 SDK 驱动库，参考 [librealsense 官方仓库](https://github.com/IntelRealSense/librealsense)

---

## 3. 快速启动指令 (Quick Start Launch)

编译并设置环境变量后，即可直接调用统一 Launch 入口：

```bash
# 1. 进入工作区并加载环境变量
cd ~/Desktop/fast-livo2-ws
source install/setup.bash
```

### 1. 一键启动：传感器 + FAST-LIVO2 建图与定位 (推荐)
同时启动 MID-360 雷达、D405 相机、FAST-LIVO2 实时建图定位算法节点及 RViz 可视化：
```bash
ros2 launch top_pkg bringup.launch.py
```

### 2. 调试启动：仅启动传感器驱动 (Sensors Only)
仅启动 MID-360 雷达与 D405 相机驱动，适合传感器硬件检查、标定对准或数据采集：
```bash
ros2 launch top_pkg bringup_sensor.launch.py
```

### 3. 常用命令行参数重载
```bash
# 关闭建图或调试时的 RViz 窗口 (后台运行/节省算力)
ros2 launch top_pkg bringup.launch.py enable_rviz:=false

# 仅启动雷达，关闭相机
ros2 launch top_pkg bringup_sensor.launch.py enable_camera:=false
```
