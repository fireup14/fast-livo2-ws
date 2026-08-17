# FAST-LIVO2 LiDAR→D405 投影验证节点性能与回调诊断修改方案

## 1. 任务背景

当前工作空间：

```text
~/Desktop/fast_livo2_ws
```

当前已有独立投影验证 package：

```text
src/lidar_camera_projection_check
```

当前主要节点：

```text
projection_check_node
```

当前联合启动文件：

```text
bringup_projection_check.launch.py
```

联合启动后会同时启动：

```text
Livox MID360
RealSense D405
projection_check_node
rqt_image_view
```

当前现象：

1. MID360 可以正常启动，LiDAR 约 10 Hz；
2. D405 配置为 1280×720@30 FPS；
3. 运行联合 launch 后，D405 实际观察频率明显下降；
4. `projection_check_node` 只打印初始化参数：
   - LiDAR topic
   - Image topic
   - Camera intrinsics
   - Distortion
   - Rcl
   - Pcl
5. 没有持续打印预期的：

```text
input lidar points=...
projected points=...
dt_raw=...
```

说明当前需要优先判断：

```text
LiDAR callback 是否正常
Image callback 是否正常
projectAndPublish() 是否耗时过大
```

本任务目标：

> 在不修改 FAST-LIVO2 主算法、不修改 Rcl/Pcl、不修改传感器驱动的前提下，增强投影检查节点的回调诊断，并优化投影计算性能，使 `/lidar_projection/image` 能稳定输出。

---

## 2. 已知输入与输出

### LiDAR

```text
/livox/lidar
```

类型：

```text
livox_ros_driver2/msg/CustomMsg
```

### Camera

```text
/camera/camera/color/image_raw
```

类型：

```text
sensor_msgs/msg/Image
```

### 输出

```text
/lidar_projection/image
```

类型：

```text
sensor_msgs/msg/Image
```

---

## 3. 外参与投影关系保持不变

保持当前 `livo.yaml` 和 `d405.yaml` 参数。

LiDAR → Camera：

```text
Pc = Rcl * Pl + Pcl
```

禁止：

```text
转置 Rcl
修改 Pcl
求逆外参
自动优化外参
在线标定
```

当前任务首先解决回调和性能问题，不通过“改外参”解决没有输出的问题。

---

## 4. 第一项修改：增加回调入口诊断日志

### 4.1 LiDAR callback

在 `lidarCallback()` 最前面增加：

```cpp
RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "LiDAR callback OK, points=%zu",
    msg->points.size());
```

预期每约 2 秒出现：

```text
LiDAR callback OK, points=xxxxx
```

### 4.2 Image callback

在 `imageCallback()` 最前面增加：

```cpp
RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Image callback OK, width=%u height=%u",
    msg->width,
    msg->height);
```

预期：

```text
Image callback OK, width=1280 height=720
```

---

## 5. 增加“等待 LiDAR”诊断

当前：

```cpp
if (!lidar)
{
    return;
}
```

改成：

```cpp
if (!lidar)
{
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Image received but no LiDAR frame has been received yet.");
    return;
}
```

用于直接判断：

```text
Image callback 正常
但 LiDAR 尚未进入节点
```

---

## 6. 回调诊断结果解释

### 情况 A

只有：

```text
Image callback OK
```

没有：

```text
LiDAR callback OK
```

说明 `/livox/lidar` 没进入节点。

继续检查：

```bash
ros2 topic info /livox/lidar -v
```

重点检查：

```text
类型
QoS
Subscriber
```

### 情况 B

只有：

```text
LiDAR callback OK
```

没有：

```text
Image callback OK
```

说明 D405 图像没进入节点。

检查：

```bash
ros2 topic info /camera/camera/color/image_raw -v
```

### 情况 C

两个都有，但没有：

```text
input lidar points=...
projected points=...
```

说明 `projectAndPublish()` 很可能过慢。

此时进入下面的性能优化。

---

## 7. 第二项修改：禁止逐点调用 cv::projectPoints

当前主要性能问题是：

```cpp
for (...)
{
    std::vector<cv::Point3d> object_points = { ... };
    std::vector<cv::Point2d> image_points;

    cv::projectPoints(
        object_points,
        zero_rotation,
        zero_translation,
        camera_matrix_,
        distortion_,
        image_points);
}
```

当前是：

```text
一个 LiDAR 点
→ 一次 cv::projectPoints()
```

如果一帧约 24000 点、`point_step=3`：

```text
约 8000 次 projectPoints / 图像
```

在 30 FPS 下完全没有必要。

---

## 8. 改成批量投影

### 8.1 收集 Camera-frame 点

```cpp
std::vector<cv::Point3d> camera_points;
std::vector<double> depths;

camera_points.reserve(points_in / point_step_);
depths.reserve(points_in / point_step_);

for (size_t index = 0;
     index < points_in;
     index += static_cast<size_t>(point_step_))
{
    const auto &point = lidar_msg->points[index];

    Eigen::Vector3d point_lidar(
        point.x,
        point.y,
        point.z);

    const double range = point_lidar.norm();

    if (range < min_range_ ||
        range > max_range_)
    {
        continue;
    }

    const Eigen::Vector3d point_camera =
        Rcl_ * point_lidar + Pcl_;

    if (point_camera.z() <= 0.0)
    {
        continue;
    }

    camera_points.emplace_back(
        point_camera.x(),
        point_camera.y(),
        point_camera.z());

    depths.push_back(point_camera.z());
}
```

### 8.2 只调用一次 projectPoints

```cpp
std::vector<cv::Point2d> image_points;

if (!camera_points.empty())
{
    cv::projectPoints(
        camera_points,
        zero_rotation,
        zero_translation,
        camera_matrix_,
        distortion_,
        image_points);
}
```

要求：

```text
每帧最多调用一次 cv::projectPoints()
```

### 8.3 再统一绘制

```cpp
for (size_t i = 0;
     i < image_points.size();
     ++i)
{
    const int u =
        static_cast<int>(
            std::lround(image_points[i].x));

    const int v =
        static_cast<int>(
            std::lround(image_points[i].y));

    if (u < 0 || u >= overlay.cols ||
        v < 0 || v >= overlay.rows)
    {
        continue;
    }

    // 根据 depths[i] 计算颜色
    // cv::circle(...)

    ++points_projected;
}
```

---

## 9. 第三项修改：降低投影处理频率

该节点只是外参验证工具，不需要按 30 FPS 全速投影。

新增参数：

```text
projection.image_step
```

默认：

```text
3
```

含义：

```text
每 3 帧图像处理 1 帧
```

D405 为 30 FPS 时：

```text
projection ≈ 10 FPS
```

实现示例：

```cpp
++image_count_;

if (image_count_ % image_step_ != 0)
{
    return;
}
```

成员变量：

```cpp
uint64_t image_count_ = 0;
int image_step_ = 3;
```

参数读取：

```cpp
image_step_ =
    declare_parameter<int>(
        "projection.image_step",
        3);
```

如果 CPU 仍高，可设：

```text
projection.image_step = 6
```

得到约：

```text
5 FPS
```

---

## 10. 第四项修改：增加关键数量统计

建议统计：

```text
points_in
range_valid
camera_front
points_projected
```

例如：

```text
input=24000
range_valid=18000
camera_front=7200
projected=3100
```

意义：

```text
camera_front = 0
→ 优先怀疑外参方向

camera_front 很多但 projected = 0
→ 优先检查相机内参/图像尺寸/投影实现

projected 数量合理
→ 投影链路基本正常
```

---

## 11. 第五项修改：增加处理耗时统计

在 `projectAndPublish()` 开头：

```cpp
const auto start =
    std::chrono::steady_clock::now();
```

结束：

```cpp
const auto end =
    std::chrono::steady_clock::now();

const double elapsed_ms =
    std::chrono::duration<double, std::milli>(
        end - start).count();
```

日志推荐：

```cpp
RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "input=%zu range_valid=%zu camera_front=%zu projected=%zu dt_raw=%.1f ms process=%.1f ms",
    points_in,
    range_valid,
    camera_front,
    points_projected,
    dt_raw_ms,
    elapsed_ms);
```

目标：

```text
process < 100 ms
```

最好：

```text
process < 50 ms
```

---

## 12. 第六项修改：联合 launch 增加 enable_rqt 参数

建议 `bringup_projection_check.launch.py` 增加：

```text
enable_rqt
```

默认：

```text
true
```

性能排查：

```bash
ros2 launch lidar_camera_projection_check   bringup_projection_check.launch.py   enable_rqt:=false
```

只运行：

```text
MID360
D405
projection_check_node
```

确认节点稳定后，再：

```bash
ros2 launch lidar_camera_projection_check   bringup_projection_check.launch.py   enable_rqt:=true
```

---

## 13. 不要长期使用 ros2 topic hz 测 raw Image

D405 当前：

```text
1280×720
RGB8
30 FPS
```

不要长期运行：

```bash
ros2 topic hz /camera/camera/color/image_raw
```

因为它会增加一个完整 raw image subscriber。

本次优先使用：

```text
Image callback OK
```

和：

```text
process=xx ms
```

判断图像是否持续进入节点。

---

## 14. QoS 暂不修改

当前订阅推荐继续使用：

```cpp
rclcpp::SensorDataQoS()
```

除非：

```bash
ros2 topic info ... -v
```

明确显示 QoS 不兼容，否则不要先改 QoS。

---

## 15. 编译

```bash
cd ~/Desktop/fast_livo2_ws

source /opt/ros/jazzy/setup.bash

colcon build   --packages-select lidar_camera_projection_check   --symlink-install

source install/setup.bash
```

---

## 16. 第一轮验证

先关闭 rqt：

```bash
ros2 launch lidar_camera_projection_check   bringup_projection_check.launch.py   enable_rqt:=false
```

预期：

```text
LiDAR callback OK, points=...
Image callback OK, width=1280 height=720
```

随后持续看到：

```text
input=...
range_valid=...
camera_front=...
projected=...
dt_raw=...
process=... ms
```

---

## 17. 第二轮验证

确认性能正常后：

```bash
ros2 launch lidar_camera_projection_check   bringup_projection_check.launch.py   enable_rqt:=true
```

在 `rqt_image_view` 中选择：

```text
/lidar_projection/image
```

此时再开始判断 LiDAR 点与 RGB 几何边缘是否对齐。

---

## 18. 禁止修改范围

本任务禁止修改：

```text
src/FAST-LIVO2/src/LIVMapper.cpp
src/FAST-LIVO2/src/vio.cpp
src/FAST-LIVO2/src/IMU_Processing.cpp
src/FAST-LIVO2/src/voxel_map.cpp
```

禁止修改：

```text
Rcl
Pcl
```

禁止为了“让图像看起来正确”而人为：

```text
翻转
镜像
转置 Rcl
改 Pcl 符号
```

---

## 19. 验收标准

- [ ] LiDAR callback 持续执行；
- [ ] Image callback 持续执行；
- [ ] 不再只有初始化日志；
- [ ] `projectAndPublish()` 能持续完成；
- [ ] `/lidar_projection/image` 持续输出；
- [ ] 改为批量 `cv::projectPoints()`；
- [ ] 默认投影频率约 10 Hz；
- [ ] 单帧处理时间建议 <100 ms；
- [ ] D405 不因投影节点明显卡死；
- [ ] 不修改 Rcl/Pcl；
- [ ] 不修改 FAST-LIVO2 LIO 主链。

---

## 20. Agent 最终需要汇报

完成后请输出：

```text
1. 修改文件列表
2. LiDAR callback 是否正常
3. Image callback 是否正常
4. 是否改为批量 cv::projectPoints
5. projection.image_step 默认值
6. 单帧 process 时间
7. /lidar_projection/image 实际输出频率
8. camera_front 数量
9. projected 数量
10. 是否修改 Rcl/Pcl
11. 是否修改 FAST-LIVO2 核心
```

第 10、11 项期望：

```text
NO
NO
```

---

## 21. 推荐执行顺序

```text
Step 1
增加 LiDAR/Image callback 日志
        ↓
Step 2
确认两个 callback 都触发
        ↓
Step 3
批量 cv::projectPoints
        ↓
Step 4
projection.image_step 降频
        ↓
Step 5
增加数量和耗时日志
        ↓
Step 6
关闭 rqt 做纯性能验证
        ↓
Step 7
重新打开 rqt
        ↓
Step 8
最后才判断 Rcl/Pcl 的几何对齐效果
```

最终链路：

```text
MID360 /livox/lidar
          │
          ▼
   latest LiDAR frame
          │
          │
D405 /camera/camera/color/image_raw
          │
          ▼
     Image callback
          │
     每3帧处理1帧
          ▼
Pc = Rcl * Pl + Pcl
          │
          ▼
批量 cv::projectPoints()
          │
          ▼
        overlay
          │
          ▼
/lidar_projection/image
          │
          ▼
    rqt_image_view
```

先解决“回调和性能”，再判断外参。
