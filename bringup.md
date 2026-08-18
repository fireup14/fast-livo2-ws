## Launch

Bring up selected drivers:

### 确定雷达相机外参概况
启动相机加对准
source install/setup.bash
ros2 launch lidar_camera_projection_check bringup_projection_component.launch.py
启动雷达
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py \
  enable_lidar:=true \
  enable_camera:=false \
  enable_rviz:=false
启动rqt
source install/setup.bash
ros2 run rqt_image_view rqt_image_view


### 正常启动

```bash
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py
```

ros2 launch realsense2_camera rs_launch.py

```bash
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false
```

```bash
source install/setup.bash
ros2 launch fast_livo mapping.launch.py
```

```bash
source install/setup.bash
ros2 launch top_pkg bringup.launch.py
```


            "color_qos": "SENSOR_DATA",
            "color_info_qos": "SENSOR_DATA",











































