## Launch

Bring up selected drivers:

```bash
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py
```

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

  source install/setup.bash
  ros2 launch lidar_camera_projection_check projection_check.launch.py


  source install/setup.bash
  ros2 run rqt_image_view rqt_image_view