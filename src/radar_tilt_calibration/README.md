# radar_tilt_calibration

## Quick start

```bash
cd ~/Desktop/fast-livo2-ws

source /opt/ros/humble/setup.bash
colcon build --packages-select radar_tilt_calibration
source install/setup.bash

# 1. 修改输入话题、消息类型、采集时间和输出坐标系
nano src/radar_tilt_calibration/config/calibrate_tilt_param.yaml

# 2. 重新构建，使源码配置复制到 install 目录
colcon build --packages-select radar_tilt_calibration
source install/setup.bash

# 3. 让机器人在水平地面上走出具有二维跨度的轨迹并执行标定
ros2 launch radar_tilt_calibration calibrate.launch.py

# 4. 标定节点写入角度并自动退出后，启动坐标转换
ros2 launch radar_tilt_calibration transform.launch.py
```

该包完成三项工作：

1. `calibrate_tilt_node` 采集雷达坐标系下的定位点，通过 SVD 拟合水平运动平面，
   计算并以度为单位保存 X、Y、Z 三个轴向角。
2. `transform_to_world_node` 读取三个角度，选择绝对值最大的角度作为单轴校正量，
   将位置和姿态从雷达坐标系旋转到世界坐标系，并发布为 `PoseStamped` 或
   `Odometry`。
3. `world_tf_bridge_node` 使用同一校正角发布静态 TF
   `camera_init_world -> camera_init`，并把 `/slam_result` 转换为动态 TF
   `camera_init_world -> aft_mapped_world`，从而连接原始与校正后的 TF 分支。

角度计算和“最大倾斜角选择”封装在 `tilt_angles.hpp`，角度 YAML 的统一读写封装在
`tilt_angle_io.hpp/.cpp`。标定、坐标转换和 TF 广播各自由独立模块负责。

## 两个 launch 的数据流

### `calibrate.launch.py`：生成倾斜角标定文件

当前默认数据流为：

```text
/aft_mapped_to_init (nav_msgs/msg/Odometry)
                    │
                    ▼
          提取 pose.pose.position
                    │
                    ▼
        在配置时长内累计三维轨迹点
                    │
                    ▼
     SVD 拟合运动平面并计算单位法向量
                    │
                    ▼
        计算 X/Y 倾斜角；Z 偏航固定为 0
                    │
                    ▼
config/tilt_angle.yaml (angles_deg: x/y/z)
```

该 launch 只生成标定文件，不发布 `/slam_result`。采集结束后，节点写入结果并自动
退出。

### `transform.launch.py`：在线转换定位结果

当前默认数据流为：

```text
config/tilt_angle.yaml
          │
          ▼
读取 X/Y/Z 三个角度并按绝对值选择最大值
          │
          ▼
构造 camera_init 到 camera_init_world 的旋转矩阵
          ▲
          │
/aft_mapped_to_init (nav_msgs/msg/Odometry)
          │
          ▼
旋转 position、orientation 和 pose covariance
保留时间戳，并按配置保留 Odometry twist
          │
          ▼
/slam_result
(nav_msgs/msg/Odometry)
frame_id=camera_init_world
child_frame_id=aft_mapped_world
          │
          ▼
world_tf_bridge_node
          ├── /tf_static: camera_init_world -> camera_init
          └── /tf: camera_init_world -> aft_mapped_world
```

转换节点持续运行，每收到一帧输入 Odometry 就发布一帧转换后的 Odometry。TF 桥接
节点负责广播新 TF；转换仍只包含旋转，不进行原点平移。

FAST-LIVO2 与该 launch 同时运行时，完整 TF 树为：

```text
camera_init_world
├── camera_init
│   └── aft_mapped
└── aft_mapped_world
```

其中 `camera_init -> aft_mapped` 继续由 FAST-LIVO2 发布。正确校正且时间戳一致时，
`aft_mapped` 与 `aft_mapped_world` 表示同一物理位姿，在公共 TF 树中应基本重合。

## 坐标与符号约定

标定结果表示从输入参考坐标系 B 到世界坐标系 A 的主动旋转：

- 绕 x 轴：`p_A = R_x(alpha) * p_B`，世界 z 轴在 B 中为
  `n=(0, sin(alpha), cos(alpha))`，所以 `alpha=atan2(n_y,n_z)`。
- 绕 y 轴：`p_A = R_y(beta) * p_B`，世界 z 轴在 B 中为
  `n=(-sin(beta), 0, cos(beta))`，所以 `beta=atan2(-n_x,n_z)`。

姿态使用左乘：`q_world = q_R * q_radar`。标定和转换节点使用完全相同的约定。

水平运动平面的法向量不能观测绕 z 轴的偏航，所以标定文件中的 `z` 固定为
`0.0 deg`。转换节点比较 `|x|`、`|y|`、`|z|`，选择绝对值最大者，但保留原角度的
正负号。该转换仍属于“主要倾斜轴的单轴校正”；若实际同时存在明显滚转和俯仰，应
改用完整旋转矩阵或四元数，而不是只使用最大角。

## 配置

全部业务参数位于
[`config/calibrate_tilt_param.yaml`](config/calibrate_tilt_param.yaml)。它是节点使用
`yaml-cpp` 手动读取的普通 YAML，不需要写成 ROS 参数文件的
`ros__parameters` 格式。

主要参数：

| 参数 | 说明 |
| --- | --- |
| `calibrate.input_topic` | 标定输入定位话题 |
| `calibrate.input_type` | `PoseStamped` 或 `Odometry` |
| `calibrate.duration` | 从第一条有效定位数据开始计算的采集时长，单位秒 |
| `calibrate.output_file` | 标定结果文件；相对路径按源码包根目录解析 |
| `calibrate.min_points` | 最少有效位置点数，至少为 3 |
| `transform.input_topic` | 转换节点输入话题 |
| `transform.input_type` | `PoseStamped` 或 `Odometry` |
| `transform.output_topic` | 世界坐标系结果话题 |
| `transform.output_type` | `PoseStamped` 或 `Odometry` |
| `transform.output_frame_id` | 输出消息的 `header.frame_id` |
| `transform.output_child_frame_id` | Odometry 子坐标系；空值时保留输入值 |
| `transform.angle_file` | 标定角度文件；相对路径按源码包根目录解析 |
| `transform.preserve_odometry_twist` | Odom 到 Odom 时是否保留原 twist |
| `tf_bridge.enable` | 是否发布静态校正 TF 和 `/slam_result` 动态 TF |
| `tf_bridge.original_root_frame` | FAST-LIVO2 原始根坐标系，默认 `camera_init` |
| `tf_bridge.corrected_root_frame` | 校正后的根坐标系，默认 `camera_init_world` |
| `tf_bridge.corrected_child_frame` | 校正后的移动坐标系，默认 `aft_mapped_world` |
| `tf_bridge.corrected_odom_topic` | 用于生成动态 TF 的 Odometry 话题 |
| `tf_bridge.angle_file` | 静态 TF 使用的角度文件，必须与转换节点一致 |

QoS 深度和可靠性也集中在该配置文件中。可靠性仅接受 `reliable` 或
`best_effort`。

### 配置文件覆盖

launch 文件提供 `config` 参数：

```bash
ros2 launch radar_tilt_calibration calibrate.launch.py \
  config:=/absolute/path/to/calibrate_tilt_param.yaml

ros2 launch radar_tilt_calibration transform.launch.py \
  config:=/absolute/path/to/calibrate_tilt_param.yaml
```

直接启动可执行文件时也支持自定义 `--config`：

```bash
ros2 run radar_tilt_calibration calibrate_tilt_node \
  --config /absolute/path/to/calibrate_tilt_param.yaml
```

## 编译

```bash
cd ~/Desktop/fast-livo2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radar_tilt_calibration
source install/setup.bash
```

如果使用 `--symlink-install`，修改源码目录中的配置后通常无需再次复制配置：

```bash
colcon build --symlink-install --packages-select radar_tilt_calibration
```

## 标定步骤

1. 确认输入定位消息的位置是雷达坐标系 B 下的轨迹。
2. 保持机器人在世界水平面上运动。
3. 轨迹必须具有二维跨度，例如矩形、圆形或往返加横向运动。只有一条直线无法唯一
   确定平面法向量。
4. 启动标定：

   ```bash
   ros2 launch radar_tilt_calibration calibrate.launch.py
   ```

5. 达到 `calibrate.duration` 后，节点输出点数、平面法向量及 X/Y/Z 三轴角度（deg），
   写入 `tilt_angle.yaml` 后自动退出。

节点收到第一条有效定位数据后才开始 `calibrate.duration` 计时。默认相对输出路径
解析到源码包根目录，因此标定结果为：

```text
~/Desktop/fast-livo2-ws/src/radar_tilt_calibration/config/tilt_angle.yaml
```

`calibrate.output_file`、`transform.angle_file` 和 `tf_bridge.angle_file` 均按同一规则
解析，所以标定结束后无需重新构建即可由转换节点读取。若运行环境中没有源码工作区，
应把三者设置为同一个可写的绝对路径。

## 转换步骤

标定成功后运行：

```bash
ros2 launch radar_tilt_calibration transform.launch.py
```

节点保留输入时间戳，并将输出 `frame_id` 设置为
`transform.output_frame_id`。当输入和输出都是 Odometry 时：

- 位姿协方差会使用 `diag(R,R)` 旋转到世界坐标系；
- twist 按 `nav_msgs/Odometry` 约定视为表达在 `child_frame_id` 中，默认原样保留；
- 设置 `preserve_odometry_twist: false` 可将输出 twist 置零；
- PoseStamped 转 Odometry 时没有输入 twist，因此输出为零。

默认配置会同时启动 TF 桥接节点。可用以下命令检查：

```bash
ros2 run tf2_ros tf2_echo camera_init_world camera_init
ros2 run tf2_ros tf2_echo camera_init_world aft_mapped_world
ros2 run tf2_ros tf2_echo aft_mapped aft_mapped_world
```

最后一组变换应接近单位变换。不要额外发布
`aft_mapped -> aft_mapped_world`，否则会让 `aft_mapped_world` 出现第二个父节点。

## ROS 参数覆盖

YAML 提供默认值，ROS 参数覆盖具有更高优先级。直接运行节点时，例如把采集时长临时
改为 60 秒：

```bash
ros2 run radar_tilt_calibration calibrate_tilt_node \
  --ros-args -p calibrate.duration:=60.0
```

也可以同时覆盖配置文件和单个参数：

```bash
ros2 run radar_tilt_calibration calibrate_tilt_node \
  --config /absolute/path/to/calibrate_tilt_param.yaml \
  --ros-args -p calibrate.duration:=60.0
```

launch 启动时推荐修改 YAML 或通过 `config:=...` 选择另一份完整配置，以继续保持所有
项目相关参数集中管理。

## 输出文件格式

```yaml
angles_deg:
  x: 7.047381
  y: -1.218390
  z: 0.0
```

三个角度的单位均为度。法向量在求解后会归一化，并强制选择 `n_z > 0` 的方向。
以上示例中转换节点会选择 X 轴的 `+7.047381 deg` 执行校正。

## 两个 Odometry 话题的含义

- `/aft_mapped_to_init`：`header.frame_id=camera_init`、
  `child_frame_id=aft_mapped`，表示 FAST-LIVO2 原始位姿
  `T_camera_init_aft_mapped`。
- `/slam_result`：`header.frame_id=camera_init_world`、
  `child_frame_id=aft_mapped_world`，表示同一物理位姿在校正后坐标系中的表达
  `T_camera_init_world_aft_mapped_world`。

二者满足：

```text
T_camera_init_world_aft_mapped_world
  = T_camera_init_world_camera_init × T_camera_init_aft_mapped
```

frame 名称不要以 `/` 开头；话题名称可以使用 `/slam_result` 这种绝对名称。
