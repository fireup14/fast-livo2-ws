# top_pkg

Top-level ROS 2 bringup package for Livox, RealSense, and optional RViz.

## Config

Main bringup switches are in `config/bringup.yaml`:

- `enable_lidar`
- `enable_camera`
- `enable_rviz`

`livox.config_file` is resolved relative to the installed
`livox_ros_driver2/config/` directory, so it does not depend on an absolute
workspace path.

## Launch

Bring up selected drivers:

```bash
ros2 launch top_pkg bringup_sensor.launch.py
```

```bash
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false
```

Bring up the sensors and FAST-LIVO2 MID-360 mapping together:

```bash
ros2 launch top_pkg bringup.launch.py
```
