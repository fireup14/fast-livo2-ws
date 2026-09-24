# foxglove_bridge_bringup

这个包独立启动 `foxglove_bridge`，供 PC 上的 Foxglove Studio 通过 WebSocket
查看 RDK 上的 FAST-LIVO2 数据。它不启动或修改 FAST-LIVO2、Livox、RealSense。

默认仅暴露以下显示所需话题，避免误订阅原始雷达和 IMU 数据：

- `/aft_mapped_to_init`
- `/cloud_registered`
- `/path`
- `/camera/camera/color/image_raw`
- `/tf`、`/tf_static`

## 依赖安装（RDK）

先在 RDK 安装 Humble 的 Foxglove Bridge：

```bash
sudo apt update
sudo apt install ros-humble-foxglove-bridge
```

## 构建与启动

```bash
cd ~/Desktop/fast-livo2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select foxglove_bridge_bringup --symlink-install
source install/setup.bash
ros2 launch foxglove_bridge_bringup foxglove_bridge.launch.py
```

在 PC 的 Foxglove Studio 中添加连接：

```text
ws://<RDK_IP>:8765
```

如端口冲突，可改用其他端口：

```bash
ros2 launch foxglove_bridge_bringup foxglove_bridge.launch.py port:=8766
```
