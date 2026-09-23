# RDK S100P 上 RGB 图像与点云的高效显示方案

## 1. 方案目标

RDK S100P 负责传感器驱动、ROS 2 数据处理、FAST-LIVO2 建图和 MCAP 采集；RGB 图像与点云交给 Ubuntu x86_64 桌面电脑显示。

这样可以避免在 S100P 上使用 RViz2 的 `llvmpipe` CPU 软件渲染，降低 CPU 负载。RViz2 后续只用于路径、焊缝中心线、机器人轨迹和缺陷标记等数据量较小的显示。

## 2. 总体架构

```text
RDK S100P
├── RealSense / MindVision / Livox
├── FAST-LIVO2
├── ROS 2 传感器与算法节点
├── MCAP 采集
└── foxglove_bridge
        │ Foxglove WebSocket
        │ 局域网
        ▼
Ubuntu x86_64 桌面电脑
└── Foxglove Studio
    ├── RGB 图像
    ├── 深度图像
    ├── 点云
    ├── TF
    ├── 路径与轨迹
    └── 检测结果
```

Foxglove 的 3D 面板支持 ROS 2 `sensor_msgs/msg/PointCloud2`，也支持 `sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CompressedImage`。

## 3. 设备分工

### 3.1 RDK S100P

- RealSense、Livox、MindVision 等设备驱动
- FAST-LIVO2 实时建图
- 点云、图像、TF 和路径话题发布
- 数据同步和时间戳处理
- MCAP 原始数据采集
- Foxglove Bridge 网络转发

### 3.2 Ubuntu 桌面电脑

- Foxglove Studio 图形显示
- 使用桌面电脑的 GPU 渲染点云和图像
- 录制数据回放
- 检查时间戳、TF 和传感器状态
- 辅助调试和结果查看

## 4. 推荐 ROS 2 话题

实际话题名称以当前系统为准，可以使用以下命令确认：

```bash
ros2 topic list
ros2 topic list | grep -Ei 'image|camera|cloud|point|path|tf'
```

### 4.1 RGB 图像

优先使用压缩图像：

```text
/camera/color/image_raw/compressed
```

如果系统没有压缩图像，再使用：

```text
/camera/color/image_raw
```

查看话题类型：

```bash
ros2 topic type /camera/color/image_raw/compressed
```

### 4.2 点云

FAST-LIVO2 点云示例：

```text
/fast_livo/cloud_registered
```

实际使用前检查：

```bash
ros2 topic list | grep -Ei 'cloud|points|pointcloud'
ros2 topic type /fast_livo/cloud_registered
```

类型应为：

```text
sensor_msgs/msg/PointCloud2
```

### 4.3 TF

```text
/tf
/tf_static
```

### 4.4 路径和轨迹

路径消息建议使用：

```text
nav_msgs/msg/Path
```

后续可以发布：

- 机器人运动轨迹
- 焊缝中心线
- 巡检规划路径
- 缺陷位置
- 检测区域边界

## 5. S100P 端部署 Foxglove Bridge

先检查是否已经安装：

```bash
ros2 pkg prefix foxglove_bridge
```

如果能返回安装路径，直接启动：

```bash
ros2 run foxglove_bridge foxglove_bridge
```

默认 WebSocket 端口通常为：

```text
8765
```

检查端口：

```bash
ss -lntp | grep 8765
```

查看 S100P 地址：

```bash
hostname -I
```

如果 S100P 地址为 `192.168.2.121`，桌面端连接地址为：

```text
ws://192.168.2.121:8765
```

### 5.1 将 Bridge 加入启动流程

可以在 ROS 2 launch 文件中加入：

```python
from launch_ros.actions import Node

foxglove_bridge = Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    name='foxglove_bridge',
    output='screen',
)
```

将 `foxglove_bridge` 放入已有的传感器和建图启动文件中，即可与系统一起启动。

## 6. 桌面端连接 Foxglove Studio

在 Ubuntu 桌面电脑上打开 Foxglove Studio：

1. 选择 `Open connection`；
2. 选择 `Foxglove WebSocket`；
3. 输入 S100P 的地址：

```text
ws://192.168.2.121:8765
```

4. 建立连接；
5. 新建布局；
6. 添加 `Image`、`3D`、`Raw Messages` 和 `Plot` 面板。

## 7. 推荐显示布局

```text
┌─────────────────────┬─────────────────────┐
│ RGB Image           │ 3D PointCloud       │
│                     │ + TF                │
├─────────────────────┼─────────────────────┤
│ Robot Path          │ Diagnostics / Topics│
│ Weld Centerline     │ Timestamp / Status  │
└─────────────────────┴─────────────────────┘
```

### 7.1 Image 面板

选择：

```text
/camera/color/image_raw/compressed
```

### 7.2 3D 面板

选择：

```text
/fast_livo/cloud_registered
```

设置：

- Fixed Frame：`map` 或实际地图坐标系；
- 添加 `/tf` 和 `/tf_static`；
- 调整点大小；
- 关闭不需要的历史点云；
- 仅显示当前需要的点云话题。

### 7.3 路径面板

选择 `nav_msgs/msg/Path` 话题，显示：

- 机器人轨迹；
- 焊缝中心线；
- 规划路径；
- 巡检覆盖范围。

## 8. 带宽和数据量建议

### 8.1 RGB 图像

不要在实时显示链路中长期传输未压缩的 1080P 图像。建议：

```text
分辨率：1920×1080
帧率：5～10 FPS
格式：JPEG 压缩
```

原始图像仍可单独写入 MCAP，用于后处理和最终数据留档。

### 8.2 点云

建议实时显示使用降采样点云：

- 体素滤波；
- 降低发布频率；
- 限制显示范围；
- 关闭大量历史点云；
- 只显示当前局部地图。

推荐初始参数：

```text
点云显示频率：5～10 FPS
RGB 显示频率：5～10 FPS
路径显示频率：10～20 Hz
```

实时显示数据和原始采集数据分开处理：

```text
实时显示：压缩图像 + 降采样点云
数据存档：原始图像 + 原始点云 + 传感器数据写入 MCAP
```

## 9. 网络检查

S100P 和桌面电脑应处于同一局域网。检查：

```bash
ping 192.168.2.121
```

检查 Bridge 端口是否可达：

```bash
nc -vz 192.168.2.121 8765
```

如果连接失败，检查：

```bash
ss -lntp | grep 8765
ip addr
ip route
```

如果启用了防火墙，开放端口：

```bash
sudo ufw allow 8765/tcp
```

## 10. MCAP 回放

Foxglove Studio 可以直接打开 MCAP 文件进行离线回放。建议采集时保存：

- RGB 图像；
- 深度图像；
- Livox 点云；
- FAST-LIVO2 位姿；
- `/tf` 和 `/tf_static`；
- 路径；
- 时间戳和设备状态。

离线回放时，桌面端直接打开 MCAP 文件，不需要让 S100P 重复运行传感器和建图节点。

## 11. 验证流程

### 11.1 验证 ROS 2 话题

```bash
ros2 topic list
ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /fast_livo/cloud_registered
ros2 topic echo /tf_static --once
```

### 11.2 验证 Bridge

```bash
ros2 run foxglove_bridge foxglove_bridge
ss -lntp | grep 8765
```

### 11.3 验证桌面端显示

在 Foxglove Studio 中确认：

- RGB 图像能够连续刷新；
- 点云坐标方向正确；
- Fixed Frame 设置正确；
- TF 链完整；
- 点云和图像时间戳持续更新；
- 桌面电脑 GPU 和 CPU 负载可接受。

## 12. 最终推荐

采用以下分工：

```text
RDK S100P：
传感器驱动 + FAST-LIVO2 + 数据同步 + MCAP 采集 + Foxglove Bridge

Ubuntu 桌面电脑：
Foxglove Studio + RGB/点云 GPU 渲染 + 数据回放与调试
```

RViz2 后续只保留路径、焊缝中心线、机器人轨迹和少量检测结果显示。RGB 图像和大规模点云统一由 Foxglove Studio 在桌面电脑上显示，以降低 S100P 的 CPU 和图形渲染负担。
