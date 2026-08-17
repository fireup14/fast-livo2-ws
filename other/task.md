# FAST-LIVO2 ROS 2 移植与实时定位建图任务需求清单 (task.md)

本文档记录用户提出的所有任务需求与约束规则，作为后续开发与验收的标准清单。

---

## 1. 核心任务目标 (Core Objectives)
- [ ] **目标 1**：实现 FAST-LIVO2 算法的 ROS 2 实时定位与建图 (Real-Time SLAM & Odometry)。
- [ ] **目标 2**：将 `download` 中的 ROS 1 FAST-LIVO2 算法移植至 ROS 2（`ament_cmake` / `rclcpp`）；对已原生支持 ROS 2 的驱动仅进行 ROS 2 配置、构建和接口集成，不重复移植其驱动源码。

---

## 2. 目录与工作区约束 (Workspace Constraints)
- [ ] **约束 1（不可修改 download 目录）**：严禁直接修改 `/home/fire/Desktop/fast_livo2_ws/download` 目录下的任何文件，保持原始备份不变。
- [ ] **约束 2（src 源码复制与重构）**：将 `download` 目录下的 3 个**源码仓库**整套复制到 `/home/fire/Desktop/fast_livo2_ws/src/` 目录下：
  - `src/FAST-LIVO2`
  - `src/livox_ros_driver2`
  - `src/realsense-ros`
  所有的移植修改、头文件重构与编译配置调整**仅在 `src/` 目录中进行**。

---

## 3. 依赖项与环境需求 (Dependencies Requirements)
根据 `FAST-LIVO2/README.md` 及 ROS 2 运行要求完成依赖梳理与安装：
- [ ] **依赖 1 (编译工具)**：`colcon-common-extensions`, `python3-rosdep2`, `ament_cmake`
- [ ] **依赖 2 (基础 C++ 数学/图像库)**：
  - `PCL` (>= 1.8)
  - `Eigen` (>= 3.3.4)
  - `OpenCV` (>= 4.2)
  - `Sophus` (非模板/double-only 版本，需从 GitHub `a621ff` 编译安装)
- [ ] **依赖 3 (Vikit 工具库)**：移植/适配 `rpg_vikit` (包含 `vikit_common`, `pinhole_camera.h`, `math_utils.h`, `vision.h` 等) 为 ROS 2 兼容库。
- [ ] **依赖 4 (ROS 2 消息与算法包)**：
  - `rclcpp`, `rclcpp_components`
  - `sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
  - `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`
  - `cv_bridge`, `image_transport`, `image_transport_plugins`, `pcl_conversions`, `pcl_msgs`

---

## 4. 代码移植与驱动适配 (ROS 2 Migration Tasks)
- [ ] **任务 1 (构建工具适配)**：将 `src/FAST-LIVO2` 的 `CMakeLists.txt` 和 `package.xml` 改写为 `ament_cmake` 和 ROS 2 `package format="3"`。
- [ ] **任务 2 (Node & API 适配)**：
  - `ros::NodeHandle` $\rightarrow$ `rclcpp::Node`
  - `ros::init` $\rightarrow$ `rclcpp::init`
  - `ROS_INFO/WARN/ERROR` $\rightarrow$ `RCLCPP_INFO/WARN/ERROR`
- [ ] **任务 3 (通信与消息转换)**：
  - 改写激光点云订阅 (`livox_ros_driver2::msg::CustomMsg`)
  - 改写 IMU 数据订阅 (`sensor_msgs::msg::Imu`)
  - 改写图像订阅 (`image_transport`)
  - 改写点云/里程计/轨迹发布 (`/cloud_registered`, `/aft_mapped_to_init`, `/path`（如需 `/Odometry`，以兼容发布或 remap 提供）)
- [ ] **任务 4 (驱动支持)**：
  - 配置 `src/livox_ros_driver2` 为 ROS 2 模式 (`ROS_EDITION=ROS2`)
  - 确认 `src/realsense-ros` 在 ROS 2 下构建正常
- [ ] **任务 5 (Launch 脚本与可视化)**：
  - 编写 ROS 2 Python Launch 文件 (`mapping_avia.launch.py` / `realtime_mapping.launch.py`)
  - 提供 RViz 2 实时渲染配置文件 (`fast_livo2.rviz`)

---

## 5. 编译验证与运行测试 (Verification & Testing)
- [ ] **步骤 1**：在 `fast_livo2_ws` 下执行 `colcon build` 编译所有包无报错。
- [ ] **步骤 2**：启动传感器驱动与算法节点，检查 ROS 2 话题发布频率 (`ros2 topic hz`)。
- [ ] **步骤 3**：在 RViz 2 中实时验证配准点云与轨迹输出，确认定位建图平滑准确。

---

## 6. 补充的实施前置条件与验收要求 (已优化增强)

- [ ] **版本基线**：实施前记录 Ubuntu、`ROS_DISTRO`、编译器、Livox Driver 2、RealSense、Vikit、Sophus 的版本/commit；仅该组合完成全量构建与测试后才是受支持配置。
- [ ] **原始备份可验证**：复制前生成 `download/` 文件清单及 SHA-256 校验和；每阶段后重新比较，确认无新增、删除或内容变化。
- [ ] **驱动范围**：`livox_ros_driver2` 与 `realsense-ros` 已有 ROS 2 支持，工作内容是按其上游 ROS 2 流程配置、构建和集成。`realsense-ros` 是多包仓库，必须列明实际构建的子包及兼容的 `librealsense2` 版本。
- [ ] **Vikit 完整性**：固定 `rpg_vikit` 来源和 commit；盘点 FAST-LIVO2 对 `vikit_common`、`vikit_ros` 的实际引用，移植所需子包并导出 CMake target、头文件和 package.xml 依赖。
- [ ] **Livox 消息唯一性**：统一使用 `livox_ros_driver2::msg::CustomMsg`；移除或隔离内置 ROS 1 `livox_ros_driver/CustomMsg.h`，不得混用两套消息定义。
- [ ] **接口契约与 QoS 匹配**：每个设备组合提供输入/输出接口表，包含话题、消息类型、QoS、频率、`frame_id`、remap。原始输出至少包括 `/cloud_registered`、`/aft_mapped_to_init`、`/LIVO2/imu_propagate`、`/path` 和 `camera_init -> aft_mapped` TF；传感器订阅必须支持 `BestEffort` 与 `Reliable` QoS 动态适配，防止默认 QoS 不匹配导致静默丢包。
- [ ] **参数、TF 与并发**：将 `nh.param` 改为声明式 ROS 2 参数和 ROS 2 YAML；使用 `tf2_ros::TransformBroadcaster`；确定 executor 模型（多线程 Executor 配合互斥锁）并验证共享状态、回调队列和 SIGINT 退出流程线程安全。
- [ ] **标定与硬/软同步**：为每个支持组合提供并验证相机内参/模型（畸变参数顺序映射）、LiDAR–IMU 与 LiDAR–Camera 外参、时间戳来源及同步方法（硬件触发或 `img_time_offset` 参数）。RealSense 使用时须明确图像话题、编码（bgr8/rgb8/mono8）、光学坐标系（`camera_color_optical_frame`）和 remap。
- [ ] **运行时资源定位（移除 ROOT_DIR）**：移除 C++ 编译宏 `ROOT_DIR` 硬编码。配置、launch、YAML、RViz 和标定文件统一通过 `ament_cmake` 安装至 `share/fast_livo` 目录，并在代码中使用 `ament_index_cpp::get_package_share_directory()` 定位资源；运行日志存储于标准用户目录或 ROS 2 log 路径。
- [ ] **Eigen 内存对齐与 C++ 标准**：所有包含固定尺寸 Eigen 矩阵的类显式引入 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 宏；统一全库编译标杆为 C++17，防止多线程并行优化与内存对齐报 Segmentation Fault。
- [ ] **分层验证与交付**：依赖顺序构建 Vikit、Livox、FAST-LIVO2，再对声明范围全量构建；使用固定 rosbag2 做离线回归并记录性能/丢帧；在线检查话题、TF 和 RViz；最后完成停止与重启测试。
