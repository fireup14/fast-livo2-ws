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

该包完成两项工作：

1. `calibrate_tilt_node` 采集雷达坐标系下的定位点，通过 SVD 拟合水平运动平面，
   判断主要倾斜轴并保存角度。
2. `transform_to_world_node` 读取标定角度，将位置和姿态从雷达坐标系旋转到世界
   坐标系，并发布为 `PoseStamped` 或 `Odometry`。

## 坐标与符号约定

标定结果表示从雷达坐标系 B 到世界坐标系 A 的主动旋转：

- 绕 x 轴：`p_A = R_x(alpha) * p_B`，世界 z 轴在 B 中为
  `n=(0, sin(alpha), cos(alpha))`，所以 `alpha=atan2(n_y,n_z)`。
- 绕 y 轴：`p_A = R_y(beta) * p_B`，世界 z 轴在 B 中为
  `n=(sin(beta), 0, cos(beta))`，所以 `beta=atan2(n_x,n_z)`。

姿态使用左乘：`q_world = q_R * q_radar`。标定和转换节点使用完全相同的约定。

该方法假设安装误差主要只绕 x 或 y 中的一根轴。若实际同时存在明显的滚转和俯仰，
仅保存单轴角度会损失另一轴分量，应扩展结果格式为完整四元数或旋转矩阵。

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
| `calibrate.duration` | 墙上时间采集时长，单位秒 |
| `calibrate.output_file` | 标定结果文件；相对路径按包共享目录解析 |
| `calibrate.min_points` | 最少有效位置点数，至少为 3 |
| `transform.input_topic` | 转换节点输入话题 |
| `transform.input_type` | `PoseStamped` 或 `Odometry` |
| `transform.output_topic` | 世界坐标系结果话题 |
| `transform.output_type` | `PoseStamped` 或 `Odometry` |
| `transform.output_frame_id` | 输出消息的 `header.frame_id` |
| `transform.output_child_frame_id` | Odometry 子坐标系；空值时保留输入值 |
| `transform.angle_file` | 标定角度文件；相对路径按包共享目录解析 |
| `transform.preserve_odometry_twist` | Odom 到 Odom 时是否保留原 twist |

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

5. 达到 `calibrate.duration` 后，节点输出点数、平面法向量、倾斜轴和弧度角，写入
   `tilt_angle.yaml` 后自动退出。

默认相对输出路径解析到当前实际运行包的共享目录。普通 colcon 构建时通常是：

```text
~/Desktop/fast-livo2-ws/install/radar_tilt_calibration/share/
radar_tilt_calibration/config/tilt_angle.yaml
```

若包安装在系统只读目录，应在配置中把 `output_file` 和 `angle_file` 都设置为同一个
用户可写的绝对路径。

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
axis: "x"
angle: 0.123
```

`angle` 单位为弧度。法向量在求解后会归一化，并强制选择 `n_z > 0` 的方向。
