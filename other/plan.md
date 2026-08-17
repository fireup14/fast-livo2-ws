# FAST-LIVO2 ROS 2 移植与实时定位建图最终实施方案 (plan.md)

本方案根据用户的最新指令进行定制更新：**严禁修改 `download/` 目录中的任何代码**。方案将 `download/` 下的 3 个源码仓库拷贝至 `src/` 目录后再展开完整的 ROS 2 移植、依赖构建与建图验证。

---

## 一、 用户约束与工作区规划 (Workspace Setup & Rules)

### 1. 核心约束规则
- **`download/` 目录只读原则**：`/home/fire/Desktop/fast_livo2_ws/download` 目录作为软硬件原始备份，保持完全不修改。
- **`src/` 目录隔离重构**：将以下三个源码仓库完整复制到 `src/` 中，并在 `src/` 中进行所有 ROS 2 移植修改：
  - `src/FAST-LIVO2` (算法核心)
  - `src/livox_ros_driver2` (Livox 雷达驱动)
  - `src/realsense-ros` (RealSense 相机驱动)

### 2. 工作区文件结构
```
fast_livo2_ws/
├── download/                  <-- 原始文件目录 (只读, 不做任何修改)
│   ├── FAST-LIVO2
│   ├── livox_ros_driver2
│   └── realsense-ros
└── src/                       <-- 所有 ROS 2 代码移植与修改的执行目录
    ├── FAST-LIVO2             <-- 移植为 ROS 2 (ament_cmake + rclcpp)
    ├── livox_ros_driver2      <-- 配置为 ROS 2 模式 (build.sh ROS2)
    ├── realsense-ros          <-- ROS 2 版本相机驱动
    └── rpg_vikit              <-- [NEW] 重构适配为 ROS 2 的 vikit 依赖包
```

---

## 二、 README.md 依赖分析与需安装的软件包 (Dependencies & Setup)

根据 `FAST-LIVO2/README.md` 与 ROS 2 环境要求，需安装并验证以下系统依赖与 ROS 2 功能包：

### 1. 基础系统与构建工具
```bash
sudo apt-get update
sudo apt-get install -y colcon-common-extensions python3-rosdep2 git
```

### 2. C++ 数学与算法依赖库 (README Section 2)
- **PCL (Point Cloud Library >= 1.8)**: `sudo apt-get install -y libpcl-dev`
- **Eigen3 (>= 3.3.4)**: `sudo apt-get install -y libeigen3-dev`
- **OpenCV (>= 4.2)**: `sudo apt-get install -y libopencv-dev`
- **Sophus (双精度/非模板化版本)**：需从 GitHub 源码编译安装：
  ```bash
  git clone https://github.com/strasdat/Sophus.git
  cd Sophus && git checkout a621ff
  mkdir build && cd build && cmake .. && make -j$(nproc)
  sudo make install
  ```
- **Vikit (`rpg_vikit`)**:
  FAST-LIVO2 依赖 `vikit_common`（提供相机模型 `PinholeCamera`、图像投影/特征匹配 `vision.h`, `math_utils.h`）。需在 `src/rpg_vikit` 中将其包装为 ROS 2 `ament_cmake` 编译包。

### 3. ROS 2 官方依赖包
```bash
sudo apt-get install -y \
  ros-$ROS_DISTRO-rclcpp \
  ros-$ROS_DISTRO-rclcpp-components \
  ros-$ROS_DISTRO-std-msgs \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-visualization-msgs \
  ros-$ROS_DISTRO-tf2 \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-tf2-eigen \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  ros-$ROS_DISTRO-image-transport-plugins \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-msgs
```

---

## 三、 ROS 1 $\rightarrow$ ROS 2 代码移植技术细节 (ROS 2 Migration Strategy)

### 1. 构建规则与配置清单 (Build Configuration)
- **`src/FAST-LIVO2/CMakeLists.txt`**:
  - 改用 `ament_cmake` 编译工具链。
  - 使用 `find_package(ament_cmake REQUIRED)`, `find_package(rclcpp REQUIRED)` 等替换 ROS 1 `catkin` 依赖。
  - 声明 `ament_target_dependencies(fast_livo ...)` 及 `ament_package()`。
- **`src/FAST-LIVO2/package.xml`**:
  - 升级为 `<package format="3">`。
  - 添加 `<buildtool_depend>ament_cmake</buildtool_depend>` 及相关 ROS 2 依赖。

### 2. C++ 节点 API 替换
| 功能模块 | ROS 1 实现 | ROS 2 实现 |
| :--- | :--- | :--- |
| **节点句柄** | `ros::NodeHandle nh` | `rclcpp::Node::SharedPtr node` |
| **节点初始化** | `ros::init(argc, argv, "laserMapping")` | `rclcpp::init(argc, argv)` / `rclcpp::spin(node)` |
| **日志输出** | `ROS_INFO(...)` | `RCLCPP_INFO(node->get_logger(), ...)` |
| **雷达点云订阅** | `nh.subscribe("/livox/lidar", ...)` | `node->create_subscription<livox_ros_driver2::msg::CustomMsg>(...)` (QoS 动态适配) |
| **IMU 数据订阅** | `nh.subscribe("/livox/imu", ...)` | `node->create_subscription<sensor_msgs::msg::Imu>(...)` (QoS 动态适配) |
| **图像数据订阅** | `image_transport::ImageTransport` | ROS 2 `image_transport::create_subscription` |
| **位姿/轨迹发布** | `ros::Publisher` | `rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr` |
| **定时器** | `ros::Timer` | `rclcpp::TimerBase::SharedPtr` (`node->create_wall_timer`) |
| **时间与频率** | `ros::Time::now()`, `stamp.toSec()` | `node->now()`, `rclcpp::Time(stamp).seconds()` |
| **资源路径获取** | 硬编码 `#define ROOT_DIR` | `ament_index_cpp::get_package_share_directory("fast_livo")` |

---

## 四、 Proposed Changes (拟实施的文件变更清单)

### [Phase 1: Workspace Duplication (源码复制阶段)]
- **[COPY]** `download/FAST-LIVO2` $\rightarrow$ `src/FAST-LIVO2`
- **[COPY]** `download/livox_ros_driver2` $\rightarrow$ `src/livox_ros_driver2`
- **[COPY]** `download/realsense-ros` $\rightarrow$ `src/realsense-ros`

### [Phase 2: Code & Build Migration in `src/`]

#### [MODIFY] [CMakeLists.txt](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/CMakeLists.txt)
- 改写为 ROS 2 ament 编译配置，移除 `ROOT_DIR` 宏硬编码，链接 `rclcpp`, `pcl_conversions`, `livox_ros_driver2` 及 `vikit_common`；添加 ament install 规则。

#### [MODIFY] [package.xml](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/package.xml)
- 迁移至 format 3 格式并完善 ROS 2 依赖表。

#### [MODIFY] [main.cpp](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/src/main.cpp)
- 重构为 ROS 2 启动入口，使用 `rclcpp::executors::MultiThreadedExecutor` 并提升退出时的线程回收安全性。

#### [MODIFY] [LIVMapper.h](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/include/LIVMapper.h) & [LIVMapper.cpp](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/src/LIVMapper.cpp)
- 将成员变量与成员函数中的 ROS 1 API 全面替换为 ROS 2 `rclcpp` 接口；使用 `ament_index_cpp` 加载 YAML 参数文件。

#### [MODIFY] [preprocess.h](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/include/preprocess.h) & [preprocess.cpp](file:///home/fire/Desktop/fast_livo2_ws/src/FAST-LIVO2/src/preprocess.cpp)
- 适配 `livox_ros_driver2::msg::CustomMsg` ROS 2 结构。

#### [NEW] `src/FAST-LIVO2/launch/mapping_avia.launch.py`
- 编写 ROS 2 Python Launch 文件管理节点与 YAML 参数。

#### [NEW] `src/rpg_vikit`
- 引入并适配 ROS 2 `ament_cmake` 构建配置。

---

## 五、 Verification Plan (编译与实时测试验证)

### 1. 依赖安装与 `src/` 复制验证
- 验证 `download/` 目录保持未动。
- 验证 `src/` 目录下三个源码仓库完整到位。

### 2. colcon 增量编译验证
```bash
cd /home/fire/Desktop/fast_livo2_ws
colcon build --symlink-install --packages-select vikit_common livox_ros_driver2 fast_livo
```
- 确保零编译错误，生成 `install/` 目录与 `install/setup.bash`。

### 3. 运行与实时话题校验
- Source 环境：`source install/setup.bash`
- 启动 ROS 2 节点：`ros2 launch fast_livo mapping_avia.launch.py`
- 话题检查：
  - `ros2 topic hz /cloud_registered`
  - `ros2 topic hz /aft_mapped_to_init`
- RViz 2 可视化：观察密化点云与实时轨迹。

---

## 六、 补充实施方案：版本、接口与可靠性 (已优化增强)

### 1. 版本与原始备份
- 在复制前记录 Ubuntu、`ROS_DISTRO`、GCC、Livox Driver 2、RealSense、Vikit、Sophus 的版本或 commit，并将该组合写入运行 README；只支持已验证的组合。
- 复制前后及每个阶段结束时，对 `download/` 生成并比较文件清单与 SHA-256 校验和，作为不修改原始备份的证据。

### 2. 驱动、SDK 与 Vikit 的处理
- `livox_ros_driver2` 已有 ROS 2 构建分支：按上游说明配置 `ROS_EDITION=ROS2`，不重复移植驱动逻辑；验证其 ROS 2 `CustomMsg` 能被 FAST-LIVO2 链接。
- `realsense-ros` 是含多个 ament 子包的仓库。先确定所需子包（如 `realsense2_camera`、`realsense2_camera_msgs`、`realsense2_description`）、相机型号、固件与兼容的 `librealsense2` 运行时/开发包，再纳入全量构建。
- 固定 `rpg_vikit` 来源和 commit，使用源码盘点确认 `vikit_common` 与 `vikit_ros` 的实际引用；只移植所需子包，并安装/导出其 target、头文件与 package.xml 依赖。
- FAST-LIVO2 内置的 ROS 1 `livox_ros_driver/CustomMsg.h` 必须删除或隔离；所有 include、回调和构建依赖统一到 `livox_ros_driver2::msg::CustomMsg`，禁止两套消息定义并存。

### 3. ROS 2 迁移的关键实现细节
- **参数与资源定位**：将 `nh.param` 转为 `declare_parameter`/读取参数；**废弃编译硬编码 `ROOT_DIR`**，改用 `ament_index_cpp::get_package_share_directory("fast_livo")` 获取 YAML 和 RViz 配置路径；运行日志写入用户临时或 ROS 2 日志路径。
- **QoS 动态适配**：LiDAR、IMU、图像订阅默认使用 `rclcpp::SensorDataQoS()`，同时在 YAML/launch 中提供 QoS 覆盖配置，防止与传感器驱动的 `BestEffort` / `Reliable` 不匹配引发静默不收包问题。
- **TF 广播**：用 `tf2_ros::TransformBroadcaster` 和 tf2/geometry 消息替换 ROS 1 tf；保持 `camera_init -> aft_mapped` 语义。
- **并发与内存安全**：为算法核心类中包含固定尺寸 Eigen 矩阵的类显式添加 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`；采用多线程 Executor 配合互斥锁保证线程安全，并验证 SIGINT 下正常回收。
- **构建规范**：CMake 显式声明并安装全部实际依赖（含 `pcl_conversions`、`tf2`、`cv_bridge`、`image_transport`、OpenCV、PCL、Sophus、Boost、OpenMP、Vikit）；所有可执行程序和配置文件均通过 ament 安装部署。

### 4. 接口、标定与时间同步
实施前为每个支持的传感器组合建立接口表并随代码交付：

| 类别 | 要记录的内容 |
| :--- | :--- |
| 输入 | LiDAR、IMU、图像话题，消息类型、QoS、频率、`frame_id`、remap |
| 输出 | `/cloud_registered`、`/aft_mapped_to_init`、`/LIVO2/imu_propagate`、`/path` 的类型、频率、frame；`/Odometry` 的兼容规则 |
| TF | `camera_init -> aft_mapped` 和各传感器 frame 的父子关系、静态/动态发布者 |
| 标定与时间 | 相机内参/模型（畸变系数对应映射）、LiDAR–IMU 与 LiDAR–Camera 外参、时间戳来源、同步方式和允许误差 |

RealSense 接入时，先单独验证实际图像话题、编码（如 `bgr8`/`rgb8`/`mono8`）、ROS optical frame（如 `camera_color_optical_frame`）、IMU 时间戳与 FAST-LIVO2 的单目模型匹配；在 launch 显式设置 remap 与静态外参，不能直接假设 `/left_camera/image` 可用。

### 5. 分层验证与交付
1. 验证版本基线、Sophus CMake 导出接口、RealSense SDK/设备及 `download/` 校验和。
2. 依赖顺序增量构建 Vikit、Livox Driver 2、FAST-LIVO2；随后对声明范围执行全量 `colcon build`。
3. 以固定 rosbag2、参数文件和命令完成离线回归，记录处理时长、警告/错误、CPU、内存、输入接收率、延迟和丢帧。
4. 在线以 `ros2 topic hz`、`ros2 topic echo --once`、`ros2 node info` 和 TF 检查接口表；RViz 2 验证点云、轨迹和坐标系。
5. 在预先定义的场景、时长和性能阈值内验证轨迹连续、无持续 QoS/TF/时间同步错误，并完成停止/重启测试。

交付物包含 ROS 2 launch、参数和标定文件、RViz 配置、接口表、构建/运行 README、已验证设备/数据集与已知限制。
