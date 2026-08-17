# FAST-LIVO2：新增 LiDAR → D405 图像投影验证节点任务说明

## 0. 任务目标

在当前工作空间：

```text
~/Desktop/fast_livo2_ws
```

中新增一个**独立 ROS2 调试节点**，用于验证：

```text
Livox MID360 LiDAR 点
        ↓
LiDAR → Camera 外参 Rcl / Pcl
        ↓
RealSense D405 相机模型
        ↓
投影到 D405 RGB 图像
```

最终输出一张叠加了 LiDAR 投影点的 RGB 图像，用于人工判断：

1. LiDAR → D405 空间外参是否正确；
2. D405 相机内参是否正确；
3. 外参方向是否使用正确；
4. 静态验证通过后，进一步辅助判断 LiDAR / Camera 时间偏移是否存在明显问题。

---

# 1. 强约束：所有输入 topic 和核心参数均为已知值

本任务**禁止 Agent 自行猜测**：

- topic 名称；
- topic 消息类型；
- 相机内参；
- LiDAR-Camera 外参；
- 外参方向；
- 输入分辨率。

如果运行环境中的实际 topic 与本文不一致，Agent 不得自动改成“看起来差不多”的 topic，必须停止并报告。

---

# 2. 已知 LiDAR 输入

## 2.1 Topic

```text
/livox/lidar
```

## 2.2 消息类型

```text
livox_ros_driver2/msg/CustomMsg
```

当前 MID360 使用 Livox CustomMsg。

---

# 3. 已知 D405 RGB 输入

## 3.1 Topic

```text
/camera/camera/color/image_raw
```

## 3.2 消息类型

```text
sensor_msgs/msg/Image
```

---

# 4. 当前 FAST-LIVO2 输入配置

当前文件：

```text
src/FAST-LIVO2/config/livo.yaml
```

已经配置：

```yaml
common:
  img_topic: "/camera/camera/color/image_raw"
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  img_en: 0
  lidar_en: 1
```

本次验证期间：

```yaml
img_en: 0
```

**必须保持不变。**

本投影验证节点与 FAST-LIVO2 LIO 并行运行，但不得让图像进入状态估计。

---

# 5. 已知 LiDAR → D405 外参

当前 `livo.yaml`：

```yaml
extrin_calib:
  Rcl: [0.9999564388707759, 0.004759234073604679, -0.0080293244989188,
        -0.00933118126284288, 0.530211956525646, -0.8478137827456489,
        0.00022229960923624004, 0.8478517741022183, 0.5302332691699048]

  Pcl: [0.01160000247362164,
        -0.10451554532367027,
        0.0011290699703005549]
```

当前 FAST-LIVO2 将它传入：

```cpp
vio_manager->setLidarToCameraExtrinsic(
    cameraextrinR,
    cameraextrinT);
```

当前 VIO 内部读取：

```cpp
Rcl << MAT_FROM_ARRAY(R);
Pcl << VEC_FROM_ARRAY(P);
```

因此第一版验证节点必须严格按当前 FAST-LIVO2 约定使用：

```text
Pc = Rcl * Pl + Pcl
```

其中：

```text
Pl = LiDAR 坐标系下点
Pc = Camera 坐标系下点
```

第一版禁止：

- 自动转置 `Rcl`；
- 自动对 `Pcl` 取负；
- 自动求逆；
- 自动从 TF 覆盖这组外参；
- 自动修正外参。

本节点的目的就是验证当前 `Rcl/Pcl` 是否正确。

---

# 6. 已知 D405 相机参数

当前文件：

```text
src/FAST-LIVO2/config/d405.yaml
```

使用：

```yaml
camera:
  cam_model: Pinhole
  cam_width: 1280
  cam_height: 720

  scale: 0.5

  cam_fx: 665.1898237482282
  cam_fy: 665.8415744550642
  cam_cx: 637.6588421587555
  cam_cy: 355.47059576089157

  cam_d0: -0.046950622434026715
  cam_d1: 0.04769351060975733
  cam_d2: 0.0
  cam_d3: 0.0
```

---

# 7. 本验证节点使用原始 1280×720 RGB 图像

输入为：

```text
/camera/camera/color/image_raw
```

因此第一版验证直接使用原始：

```text
1280 × 720
```

投影时使用原始内参：

```text
fx = 665.1898237482282
fy = 665.8415744550642
cx = 637.6588421587555
cy = 355.47059576089157
```

**不要在该调试节点中乘 `scale=0.5`。**

`scale=0.5` 是 FAST-LIVO2 内部处理尺度；本节点直接在原始 RGB 图像上画点。

---

# 8. 推荐新增 package

推荐：

```text
src/lidar_camera_projection_check
```

节点名：

```text
lidar_camera_projection_check
```

可执行文件：

```text
projection_check_node
```

推荐使用 ROS2 C++ 实现，与当前工程风格一致。

---

# 9. 节点仅订阅两个已知 topic

```text
/livox/lidar
/camera/camera/color/image_raw
```

对应：

```cpp
livox_ros_driver2::msg::CustomMsg
sensor_msgs::msg::Image
```

第一版**不要订阅**：

```text
/camera/camera/color/camera_info
/tf
/tf_static
/cloud_registered
/path
/aft_mapped_to_init
```

原因：本次所需的相机内参和 LiDAR-Camera 外参已经由现有 YAML 明确提供。

---

# 10. 输出 topic

新增：

```text
/lidar_projection/image
```

类型：

```text
sensor_msgs/msg/Image
```

该 topic 为本调试节点新建输出，不是未知外部依赖。

---

# 11. 参数读取原则

节点应读取当前 FAST-LIVO2 已有参数名称：

```text
common.lid_topic
common.img_topic

extrin_calib.Rcl
extrin_calib.Pcl

camera.cam_width
camera.cam_height
camera.cam_fx
camera.cam_fy
camera.cam_cx
camera.cam_cy
camera.cam_d0
camera.cam_d1
camera.cam_d2
camera.cam_d3
```

启动时加载现有：

```text
fast_livo/config/livo.yaml
fast_livo/config/d405.yaml
```

目标是让投影验证节点和 FAST-LIVO2 使用**同一套**标定参数。

不要另建一套独立 calibration YAML 作为最终参数源。

---

# 12. Launch 文件

新增：

```text
src/lidar_camera_projection_check/launch/projection_check.launch.py
```

该 launch 需要通过：

```python
get_package_share_directory("fast_livo")
```

找到并加载：

```text
config/livo.yaml
config/d405.yaml
```

然后启动：

```text
projection_check_node
```

---

# 13. LiDAR 点字段

Livox CustomMsg 每个点至少读取：

```cpp
point.x
point.y
point.z
point.reflectivity
point.offset_time
```

第一版主要使用：

```cpp
x
y
z
reflectivity
```

`offset_time` 先保留，为后续更严格时间补偿预留，不在第一版复杂化实现。

---

# 14. 基础范围过滤

建议：

```text
min_range = 0.6 m
max_range = 30.0 m
```

因为当前 MID360 配置：

```yaml
blind: 0.6
```

调试节点建议和当前 LIO 的近距离盲区保持一致。

可新增调试参数：

```text
projection.min_range
projection.max_range
```

默认：

```text
0.6
30.0
```

---

# 15. LiDAR → Camera 坐标变换

对每个点：

```cpp
Eigen::Vector3d p_l(
    point.x,
    point.y,
    point.z);
```

执行：

```cpp
Eigen::Vector3d p_c = Rcl_ * p_l + Pcl_;
```

仅保留：

```cpp
if (p_c.z() <= 0.0)
{
    continue;
}
```

即相机前方点。

---

# 16. 相机投影模型

推荐使用：

```cpp
cv::projectPoints()
```

相机矩阵：

```cpp
cv::Mat K = (cv::Mat_<double>(3, 3) <<
    665.1898237482282, 0.0, 637.6588421587555,
    0.0, 665.8415744550642, 355.47059576089157,
    0.0, 0.0, 1.0);
```

畸变参数：

```cpp
cv::Mat D = (cv::Mat_<double>(1, 5) <<
    -0.046950622434026715,
     0.04769351060975733,
     0.0,
     0.0,
     0.0);
```

因为点已经位于 Camera frame：

```text
Pc
```

所以：

```text
rvec = 0
tvec = 0
```

即可。

不要在第一版忽略现有畸变参数。

---

# 17. 像素范围过滤

投影得到 `(u,v)` 后：

```cpp
if (u < 0 || u >= image.cols ||
    v < 0 || v >= image.rows)
{
    continue;
}
```

只有落在原始 D405 RGB 图像内的点才绘制。

---

# 18. 投影点颜色

推荐按 Camera frame 下的：

```text
Zc
```

进行深度着色。

例如：

```text
近距离 → 红/黄
中距离 → 绿
远距离 → 蓝
```

第一版如果实现成本更低，也允许先全部使用红色：

```cpp
cv::Scalar(0, 0, 255)
```

但推荐最终保留深度着色，便于看层次和遮挡。

---

# 19. 点大小

增加参数：

```text
projection.point_radius
```

默认：

```text
2
```

不要画过大的点，避免遮挡 RGB 边缘。

---

# 20. 点采样

增加：

```text
projection.point_step
```

默认：

```text
3
```

循环：

```cpp
for (size_t i = 0;
     i < msg->points.size();
     i += point_step_)
```

投影验证不需要把所有 MID360 点都画出来。

---

# 21. 第一版同步策略

第一版不要为了同步引入过复杂架构。

推荐：

```text
LiDAR callback
→ 保存 latest_lidar + lidar_stamp

Image callback
→ 读取 latest_lidar
→ 将最近一帧 LiDAR 投影到当前图像
```

必须显示：

```text
dt_raw = image_stamp - lidar_stamp
```

单位 ms。

例如：

```text
Image-LiDAR dt_raw = +32.4 ms
```

这样能直观看到两路输入的时间关系。

---

# 22. 第一版不要自动应用当前 `img_time_offset: 0.1`

当前 `livo.yaml`：

```yaml
img_time_offset: 0.1
```

这个值尚未完成当前实机 LIVO 验证。

因此投影节点新增：

```text
projection.img_time_offset
```

默认必须：

```text
0.0
```

第一阶段空间外参验证保持：

```text
0.0
```

后续动态验证再手动测试：

```text
-0.10
-0.05
0.00
+0.05
+0.10
```

---

# 23. offset 符号约定

投影调试节点必须与 FAST-LIVO2 当前定义保持一致：

```text
corrected_image_time
=
raw_image_time
+
projection.img_time_offset
```

避免后续启用 LIVO 时符号相反。

---

# 24. 输出图像叠加调试文字

`/lidar_projection/image` 左上角显示：

```text
LiDAR->D405 Projection Check
points_in: XXXXX
points_projected: XXXX
image_stamp: XXXXX.XXXX
lidar_stamp: XXXXX.XXXX
dt_raw: XX.X ms
offset_test: XX.X ms
```

建议再显示：

```text
Rcl/Pcl loaded: OK
camera params loaded: OK
```

便于截图留档。

---

# 25. 输出图像 Header

输出图像：

```cpp
output.header = image_msg->header;
```

不要改成：

```text
camera_init
```

这是二维相机调试图，应保留 D405 图像 header/frame。

---

# 26. 推荐节点类结构

```cpp
class LidarCameraProjectionCheck : public rclcpp::Node
{
public:
    LidarCameraProjectionCheck();

private:
    void lidarCallback(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg);

    void projectAndPublish(
        const sensor_msgs::msg::Image::SharedPtr &image_msg);

private:
    std::string lidar_topic_;
    std::string image_topic_;

    int width_;
    int height_;

    double fx_;
    double fy_;
    double cx_;
    double cy_;

    double d0_;
    double d1_;
    double d2_;
    double d3_;

    Eigen::Matrix3d Rcl_;
    Eigen::Vector3d Pcl_;

    int point_step_;
    int point_radius_;

    double min_range_;
    double max_range_;
    double test_img_time_offset_;

    livox_ros_driver2::msg::CustomMsg::SharedPtr latest_lidar_;
    rclcpp::Time latest_lidar_stamp_;

    std::mutex data_mutex_;

    ...
};
```

---

# 27. 并发安全

以下变量：

```text
latest_lidar_
latest_lidar_stamp_
```

必须使用：

```cpp
std::mutex
```

保护。

图像 callback 中建议快速复制 SharedPtr 后释放锁，再做投影，避免长时间阻塞 LiDAR callback。

---

# 28. CMake 依赖

至少：

```text
rclcpp
sensor_msgs
cv_bridge
OpenCV
Eigen3
livox_ros_driver2
```

如果使用 `image_transport`，加入对应依赖。

---

# 29. package.xml

至少：

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>livox_ros_driver2</depend>
```

OpenCV / Eigen3 按 ROS2 CMake 规范配置。

---

# 30. 禁止修改 FAST-LIVO2 核心

本任务禁止修改：

```text
src/FAST-LIVO2/src/LIVMapper.cpp
src/FAST-LIVO2/src/IMU_Processing.cpp
src/FAST-LIVO2/src/voxel_map.cpp
src/FAST-LIVO2/src/vio.cpp
```

可读取、参考，但不得为了投影验证改变主算法。

当前 LIO 已完成稳定验证，因此必须保护现有基线。

---

# 31. 不修改 `img_en`

必须保持：

```yaml
common:
  img_en: 0
```

目标系统：

```text
                    ┌───────────────────┐
/livox/lidar ──────→│ FAST-LIVO2 LIO    │
/livox/imu ────────→│ img_en = 0        │
                    └─────────┬─────────┘
                              ↓
                         /path
                         /cloud_registered
                         /planes


/livox/lidar ───────────────┐
                            │
/camera/camera/color/       │
image_raw ─────────────────→│ projection_check
                            │
                            ↓
                  /lidar_projection/image
```

投影检查节点和 LIO 互不影响。

---

# 32. 编译

```bash
cd ~/Desktop/fast_livo2_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select lidar_camera_projection_check \
  --symlink-install

source install/setup.bash
```

如 launch 需要读取 `fast_livo` share 目录，确保 `fast_livo` 已经 build 并 source。

---

# 33. 启动前严格确认输入

执行：

```bash
ros2 topic info /livox/lidar
```

期望：

```text
Type: livox_ros_driver2/msg/CustomMsg
Publisher count: 1
```

执行：

```bash
ros2 topic info /camera/camera/color/image_raw
```

期望：

```text
Type: sensor_msgs/msg/Image
Publisher count: 1
```

再测：

```bash
ros2 topic hz /livox/lidar
```

约：

```text
10 Hz
```

以及：

```bash
ros2 topic hz /camera/camera/color/image_raw
```

约：

```text
30 Hz
```

如果实际 topic 与本文不一致：

> 不允许自动换 topic；停止并报告实际 `ros2 topic list`。

---

# 34. 启动

```bash
ros2 launch lidar_camera_projection_check projection_check.launch.py
```

检查：

```bash
ros2 topic list | grep lidar_projection
```

应出现：

```text
/lidar_projection/image
```

---

# 35. 查看投影图

推荐：

```bash
rqt_image_view
```

选择：

```text
/lidar_projection/image
```

---

# 36. 第一阶段：静态空间外参验证

传感器完全静止。

场景必须有清晰几何结构：

```text
墙角
门框
桌边
柜子
纸箱
立柱
```

不要只对着一整面白墙。

---

# 37. 静态正确表现

观察：

```text
墙面 LiDAR 点
```

是否落在 RGB 墙面。

观察：

```text
门框 / 桌边 / 柱子
```

是否和 RGB 对应几何边缘基本一致。

允许有局部小偏差，但不能整体错方向。

---

# 38. 典型错误表现

以下现象优先怀疑 `Rcl/Pcl` 方向或矩阵排列：

```text
大量点 Zc < 0
绝大多数点投不进图像
墙面点投到地面
点云整体偏到图像另一侧
明显 90° / 180° 旋转
大面积镜像
```

---

# 39. 禁止“只转置 R”

如果发现外参方向疑似相反，不得直接：

```cpp
Rcl = Rcl.transpose();
```

同时保持 `Pcl` 不变。

真正求逆必须：

```text
R_inv = R^T
t_inv = -R^T t
```

但第一版验证节点不要自动求逆；先输出事实结果，再人工判断。

---

# 40. 第二阶段：不同距离验证

分别观察约：

```text
1 m
2 m
3 m
```

如果：

```text
近处对齐，远处越来越偏
```

优先检查：

```text
旋转外参
相机内参
畸变
```

如果：

```text
近处偏差明显，远处相对变小
```

优先检查：

```text
平移外参
```

---

# 41. 第三阶段：运动时间偏移验证

仅在静态基本正确后进行。

让 MID360 + D405 刚性组件左右缓慢旋转：

```text
约 ±20°
```

如果：

```text
静态投影好
运动时出现固定方向拖尾
```

优先怀疑：

```text
LiDAR / Camera 时间偏移
```

不要第一反应重新调空间外参。

---

# 42. 手动测试时间偏移

调试参数：

```text
projection.img_time_offset
```

依次测试：

```text
-0.10
-0.05
0.00
+0.05
+0.10
```

单位：

```text
second
```

观察哪一个 offset 下运动边缘投影最好。

---

# 43. 日志要求

节点启动时打印一次：

```text
LiDAR topic
Image topic
Camera width/height
fx fy cx cy
Distortion
Rcl
Pcl
projection.img_time_offset
```

运行中每约 1 秒打印：

```text
input lidar points
projected points
dt_raw
```

禁止每帧大量刷屏。

---

# 44. 参数安全检查

启动时必须验证：

```text
Rcl.size() == 9
Pcl.size() == 3
fx > 0
fy > 0
cam_width == 1280
cam_height == 720
```

任一不满足：

```text
RCLCPP_FATAL
```

并停止节点。

不要静默使用：

```text
Identity R
zero P
RealSense 默认内参
```

---

# 45. 成功判据

必须满足：

- [ ] `/livox/lidar` 输入正常；
- [ ] `/camera/camera/color/image_raw` 输入正常；
- [ ] `/lidar_projection/image` 持续输出；
- [ ] RGB 原图正常；
- [ ] 投影点数量合理；
- [ ] 大部分可见 LiDAR 点落在合理物体区域；
- [ ] 墙角、门框、桌边等几何结构基本对齐；
- [ ] 无整体 90° / 180° 错误；
- [ ] 无明显镜像；
- [ ] 无绝大多数点投影到图像外；
- [ ] FAST-LIVO2 `img_en` 仍为 0；
- [ ] 原有 LIO `/path`、`/cloud_registered`、`/planes` 行为不受影响。

---

# 46. 本任务明确不做

禁止：

```text
img_en = 1
真正进入 LIVO
自动修改 Rcl / Pcl
在线外参标定
自动估计时间偏移
修改 D405 驱动
修改 Livox 驱动
修改 IMU 参数
修改 LIO 参数
修改 voxel map 参数
生成 /Laser_map
```

本任务唯一目标：

> 使用当前仓库已经明确的 MID360 topic、D405 RGB topic、Rcl/Pcl 和 D405 内参，生成一个独立的 LiDAR→RGB 投影验证图像。

---

# 47. Agent 最终必须汇报

完成后必须给出：

```text
1. 新增 package 名称
2. 新增文件列表
3. 两个输入 topic
4. 两个输入消息类型
5. 输出 topic
6. 实际读取的 Rcl
7. 实际读取的 Pcl
8. 实际读取的 fx/fy/cx/cy/distortion
9. 编译结果
10. 启动命令
11. 验证命令
12. 是否修改过 FAST-LIVO2 核心文件
```

第 12 项期望：

```text
NO
```

---

# 48. 最终架构

```text
MID360
/livox/lidar
      │
      │ Rcl / Pcl
      ▼
LiDAR → Camera transform
      │
      │ D405 K + distortion
      ▼
Pixel projection
      │
      ├────────────────────┐
      │                    │
      │              D405 RGB image
      │       /camera/camera/color/image_raw
      │                    │
      └──────────┬─────────┘
                 ▼
               overlay
                 │
                 ▼
      /lidar_projection/image
                 │
                 ▼
          rqt_image_view
```

通过这个 Gate 后，才进入：

```text
时间偏移精调
     ↓
img_en = 1
     ↓
LIVO
```


  构建已通过：

  colcon build --packages-select lidar_camera_projection_check --symlink-install

  启动：

  source install/setup.bash
  ros2 launch lidar_camera_projection_check projection_check.launch.py

  验证：

  ros2 topic info /livox/lidar
  ros2 topic info /camera/camera/color/image_raw
  ros2 topic hz /livox/lidar
  ros2 topic hz /camera/camera/color/image_raw
  ros2 topic info /lidar_projection/image
  rqt_image_view

  在 rqt_image_view 中选择 /lidar_projection/image。common.img_en 仍为 0；FAST-LIVO2 核心文件修改：NO。