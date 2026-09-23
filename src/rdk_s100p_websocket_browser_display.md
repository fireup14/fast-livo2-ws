# RDK S100P 通过局域网端口向 PC 浏览器显示图像与点云

## 1. 方案目标

RDK S100P 在本机运行 ROS 2 传感器节点和算法节点，并通过局域网开放一个 WebSocket 端口。PC 与 RDK 处于同一局域网时，直接使用浏览器打开 Foxglove 网页端，实时查看 RGB 图像、点云、TF 和路径。

RDK 不运行 RViz2 图形界面，避免使用 `llvmpipe` 软件渲染造成较高 CPU 占用。PC 浏览器使用 PC 的 GPU 负责图像和点云显示。

## 2. 系统架构

```text
RDK S100P
├── RealSense / MindVision 图像节点
├── Livox / FAST-LIVO2 点云与建图节点
├── ROS 2 图像、点云、TF、路径话题
└── foxglove_bridge :8765
          │ WebSocket
          │ 同一局域网
          ▼
PC 浏览器
└── https://app.foxglove.dev
    ├── Image 面板
    ├── 3D 面板
    ├── TF / Path
    └── 数据状态查看
```

## 3. 设备职责

### RDK S100P

- 运行相机和雷达驱动；
- 运行 FAST-LIVO2；
- 发布图像、点云、TF 和路径；
- 对图像进行 JPEG 压缩；
- 对点云进行降采样或限制发布频率；
- 运行 `foxglove_bridge`，提供 WebSocket 服务。

### PC 浏览器

- 通过 WebSocket 接收 ROS 2 数据；
- 使用 PC 的 GPU 进行图像和点云渲染；
- 显示 RGB、点云、路径和 TF；
- 回放或分析 MCAP 数据。

## 4. 网络要求

RDK 和 PC 必须位于同一局域网，并且 PC 能访问 RDK 的 WebSocket 端口。

查看 RDK IP：

```bash
hostname -I
```

例如：

```text
192.168.2.121
```

推荐使用独立网口或稳定的千兆网络连接图像和点云数据。

## 5. RDK 端启动 Foxglove Bridge

检查是否已经安装：

```bash
ros2 pkg prefix foxglove_bridge
```

启动 WebSocket 服务：

```bash
ros2 run foxglove_bridge foxglove_bridge
```

默认端口通常为：

```text
8765
```

确认端口已监听：

```bash
ss -lntp | grep 8765
```

如果看到类似结果，说明 Bridge 已启动：

```text
LISTEN 0  ... 0.0.0.0:8765 ... foxglove_bridge
```

## 6. PC 端浏览器连接

在 PC 浏览器打开：

```text
https://app.foxglove.dev
```

然后执行：

1. 选择 `Open connection`；
2. 选择 `Foxglove WebSocket`；
3. 输入 RDK 的地址：

```text
ws://192.168.2.121:8765
```

4. 点击连接；
5. 新建布局；
6. 添加 `Image` 和 `3D` 面板。

## 7. 推荐 ROS 2 话题

实际名称以当前系统为准：

```bash
ros2 topic list
```

### 7.1 RGB 图像

优先使用压缩图像：

```text
/camera/color/image_raw/compressed
```

检查话题类型：

```bash
ros2 topic type /camera/color/image_raw/compressed
```

应为：

```text
sensor_msgs/msg/CompressedImage
```

如果没有压缩图像，可以使用：

```text
/camera/color/image_raw
```

类型为：

```text
sensor_msgs/msg/Image
```

### 7.2 点云

FAST-LIVO2 点云示例：

```text
/fast_livo/cloud_registered
```

查找实际点云话题：

```bash
ros2 topic list | grep -Ei 'cloud|points|pointcloud'
```

检查类型：

```bash
ros2 topic type /fast_livo/cloud_registered
```

应为：

```text
sensor_msgs/msg/PointCloud2
```

### 7.3 TF

```text
/tf
/tf_static
```

### 7.4 路径

推荐使用：

```text
nav_msgs/msg/Path
```

可以显示：

- 机器人轨迹；
- 焊缝中心线；
- 规划路径；
- 缺陷位置；
- 检测区域边界。

## 8. Foxglove 网页布局

建议建立如下布局：

```text
┌─────────────────────┬─────────────────────┐
│ RGB Image           │ 3D PointCloud       │
│                     │ + TF                │
├─────────────────────┼─────────────────────┤
│ Robot Path          │ Diagnostics         │
│ Weld Centerline     │ Topic / Timestamp   │
└─────────────────────┴─────────────────────┘
```

### Image 面板

选择：

```text
/camera/color/image_raw/compressed
```

### 3D 面板

选择：

```text
/fast_livo/cloud_registered
```

设置：

- Fixed Frame 设置为 `map` 或实际地图坐标系；
- 添加 `/tf` 和 `/tf_static`；
- 调整点大小；
- 关闭不需要的历史点云；
- 只保留当前需要的点云话题。

## 9. 带宽与实时显示配置

### 9.1 RGB 图像

不建议实时传输未压缩的 1080P 图像。建议：

```text
分辨率：1920×1080
帧率：5～10 FPS
格式：JPEG 压缩
```

### 9.2 点云

建议对实时显示点云进行：

- 体素降采样；
- 降低发布频率；
- 限制空间范围；
- 关闭大范围历史点云；
- 只显示局部地图。

推荐初始配置：

```text
RGB：5～10 FPS
点云：5～10 FPS
路径：10～20 Hz
```

实时显示数据与原始采集数据分开：

```text
实时显示：JPEG 图像 + 降采样点云
数据存档：原始图像 + 原始点云 + 传感器数据写入 MCAP
```

## 10. 网络连通性检查

PC 上测试 RDK 网络：

```bash
ping 192.168.2.121
```

测试 WebSocket 端口：

```bash
nc -vz 192.168.2.121 8765
```

RDK 上检查端口：

```bash
ss -lntp | grep 8765
```

如果启用了防火墙：

```bash
sudo ufw allow 8765/tcp
```

## 11. 常见问题

### 11.1 浏览器无法连接

检查：

```bash
ping 192.168.2.121
ss -lntp | grep 8765
```

确认连接地址使用：

```text
ws://192.168.2.121:8765
```

不要写成 `http://`。

### 11.2 可以连接但看不到话题

检查 ROS 2 话题是否正在发布：

```bash
ros2 topic list
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /fast_livo/cloud_registered
```

检查 Bridge 启动时是否与传感器节点处于相同 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/east_eleair/physic_bev/install/setup.bash
ros2 run foxglove_bridge foxglove_bridge
```

### 11.3 点云没有显示

检查：

```bash
ros2 topic type /fast_livo/cloud_registered
ros2 topic echo /tf_static --once
```

常见原因：

- 话题类型不是 `sensor_msgs/msg/PointCloud2`；
- Fixed Frame 设置错误；
- 缺少 `/tf` 或 `/tf_static`；
- 点云坐标系和固定坐标系不一致；
- 点云频率或数据量过大。

## 12. 启动顺序

推荐启动顺序：

```bash
# 终端 1：加载 ROS 2 环境
source /opt/ros/humble/setup.bash
source ~/east_eleair/physic_bev/install/setup.bash

# 终端 2：启动传感器和 FAST-LIVO2
ros2 launch top_pkg bringup.launch.py

# 终端 3：启动 Foxglove Bridge
ros2 run foxglove_bridge foxglove_bridge
```

然后在 PC 浏览器连接：

```text
ws://RDK_IP:8765
```

如果 `bringup.launch.py` 中已经包含传感器和建图节点，也可以把 Bridge 加入该 launch 文件统一启动。

## 13. MCAP 数据回放

实时显示使用 WebSocket；离线分析直接使用 MCAP 文件。

MCAP 建议保存：

- 原始 RGB 图像；
- 深度图像；
- Livox 点云；
- FAST-LIVO2 位姿；
- `/tf` 和 `/tf_static`；
- 路径；
- 设备状态和时间戳。

这样可以做到：

```text
在线：RDK → WebSocket → PC 浏览器
离线：MCAP → PC 浏览器回放
```

## 14. 最终方案

```text
RDK S100P：
ROS 2 驱动 + FAST-LIVO2 + 数据压缩/降采样 + foxglove_bridge

局域网：
WebSocket 端口 8765

PC 浏览器：
Foxglove Web 端 + PC GPU 渲染 RGB、点云和路径
```

该方案不需要在 RDK 上运行 RViz2 来显示大规模 RGB 和点云数据。RDK 只负责采集和发布数据，PC 浏览器负责显示，适合后续接入焊缝巡检、三维建图和多模态数据回放。
