# top_pkg

`top_pkg` 是工作区（FAST-LIVO2 Workspace）的顶层总控 ROS 2 功能包，用于集中管理传感器驱动（Livox MID-360 LiDAR、RealSense D405 Camera）、RViz 可视化及 FAST-LIVO2 建图定位系统的统一启动与参数配置。

---

## 目录结构

```text
top_pkg/
├── CMakeLists.txt         # 编译及安装配置
├── package.xml            # 包依赖定义
├── config/
│   ├── bringup.yaml       # 传感器及系统启动配置参数
│   └── bringup.rviz       # 默认 RViz 视图配置文件
└── launch/
    ├── bringup_sensor.launch.py # 传感器驱动单独启动脚本
    └── bringup.launch.py        # 传感器 + FAST-LIVO2 整体一键启动脚本
```

---

## 参数配置 (`config/bringup.yaml`)

可以在 `config/bringup.yaml` 中灵活配置启动开关及传感器参数：

```yaml
bringup:
  enable_lidar: true     # 是否启动 Livox MID-360 雷达
  enable_camera: true    # 是否启动 RealSense D405 相机
  enable_rviz: true      # 是否启动 RViz2 可视化界面

  livox:
    frame_id: livox_frame
    publish_freq: 10.0   # 雷达发布频率 (Hz)
    config_file: MID360_config.json # Livox 配置文件名（相对 livox_ros_driver2/config 目录）
    cmdline_input_bd_code: livox0000000001
    lvx_file_path: /home/livox/livox_test.lvx
```

---

## 启动方式 Launch

### 1. 单独启动传感器驱动 (Sensors Only)

用于硬件调试、外参对准或数据采集：

```bash
# 启动所有已启用的传感器和 RViz
ros2 launch top_pkg bringup_sensor.launch.py

# 命令行覆盖参数（例如关闭 RViz）
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false
```

### 2. 整体一键启动 (Sensors + FAST-LIVO2 Mapping)

同时启动传感器驱动、FAST-LIVO2 建图定位节点及 RViz：

```bash
ros2 launch top_pkg bringup.launch.py
```

---

## 注意事项

- 相机默认数据 QoS 策略为 `SENSOR_DATA` (Best Effort)，以降低高分辨率 RGB 传输延迟。
- `livox.config_file` 路径会在启动时自动解析为 `livox_ros_driver2/config/` 中的绝对路径，无需硬编码本机工作区路径。