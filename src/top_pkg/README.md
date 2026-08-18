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








    # Start the camera node directly. Including rs_launch.py would expose this
    # launch file's enable_* switches to the driver's unsupported-param checker.
    # realsense_driver = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     namespace="camera",
    #     name="camera",
    #     output="screen",
    #     condition=IfCondition(LaunchConfiguration("enable_camera")),
    #     parameters=[{
    #         "camera_name": "camera",
    #         "camera_namespace": "camera",
    #         "device_type": "d405",
    #         # Keep parameter types native.  Quoted booleans are passed as
    #         # strings and rejected by realsense2_camera.
    #         "enable_color": True,
    #         # Match the real-time, latest-frame QoS policy used by image consumers.
    #         "color_qos": "SENSOR_DATA",
    #         "color_info_qos": "SENSOR_DATA",
    #         # This bringup only needs RGB images and their CameraInfo.
    #         "enable_depth": False,
    #         "pointcloud.enable": False,
    #         "align_depth.enable": False,
    #         "enable_infra": False,
    #         "enable_infra1": False,
    #         "enable_infra2": False,
    #         # D405 exposes its color stream through the depth module.
    #         "depth_module.color_profile": "1280,720,30",
    #     }],
    # )


    # realsense_driver = Node(
    #     package="realsense2_camera",
    #     executable="realsense2_camera_node",
    #     namespace="camera",
    #     name="camera",
    #     output="screen",
    #     emulate_tty=True,
    #     arguments=["--ros-args", "--log-level", "info"],
    #     parameters=[{
    #         # Device selection / input
    #         "camera_name": "camera",
    #         "camera_namespace": "camera",
    #         "serial_no": "",
    #         "usb_port_id": "",
    #         "device_type": "",
    #         "json_file_path": "",
    #         "initial_reset": False,
    #         "accelerate_gpu_with_glsl": False,
    #         "rosbag_filename": "",
    #         "rosbag_loop": False,

    #         # RGB camera
    #         "enable_color": True,
    #         "rgb_camera.color_profile": "1280,720,30", ### num2
    #         "rgb_camera.color_format": "RGB8",
    #         "rgb_camera.enable_auto_exposure": True,

    #         # Depth / infrared
    #         "enable_depth": False, ### num1
    #         "enable_infra": False,
    #         "enable_infra1": False,
    #         "enable_infra2": False,
    #         "depth_module.depth_profile": "0,0,0",
    #         "depth_module.depth_format": "Z16",
    #         "depth_module.infra_profile": "0,0,0",
    #         "depth_module.infra_format": "RGB8",
    #         "depth_module.infra1_format": "Y8",
    #         "depth_module.infra2_format": "Y8",

    #         # D405 color stream (from depth module)
    #         "depth_module.color_profile": "0,0,0",
    #         "depth_module.color_format": "RGB8",
    #         "depth_module.exposure": 8500,
    #         "depth_module.gain": 16,
    #         "depth_module.hdr_enabled": False,
    #         "depth_module.enable_auto_exposure": True,
    #         "depth_module.exposure.1": 7500,
    #         "depth_module.gain.1": 16,
    #         "depth_module.exposure.2": 1,
    #         "depth_module.gain.2": 16,

    #         # Synchronization / IMU
    #         "enable_sync": False,
    #         "depth_module.inter_cam_sync_mode": 0,
    #         "enable_rgbd": False,
    #         "enable_gyro": False,
    #         "enable_accel": False,
    #         "enable_motion": False,
    #         "gyro_fps": 0,
    #         "accel_fps": 0,
    #         "motion_fps": 0,
    #         "unite_imu_method": 0,
    #         "clip_distance": -2.0,
    #         "angular_velocity_cov": 0.01,
    #         "linear_accel_cov": 0.01,
    #         "diagnostics_period": 0.0,

    #         # TF
    #         "publish_tf": True,
    #         "tf_publish_rate": 0.0,
    #         "base_frame_id": "link",
    #         "tf_prefix": "",

    #         # Point cloud / processing filters
    #         "pointcloud.enable": False,
    #         "pointcloud.stream_filter": 2,
    #         "pointcloud.stream_index_filter": 0,
    #         "pointcloud.ordered_pc": False,
    #         "pointcloud.allow_no_texture_points": False,
    #         "align_depth.enable": False,
    #         "colorizer.enable": False,
    #         "decimation_filter.enable": False,
    #         "decimation_filter.filter_magnitude": 2,
    #         "rotation_filter.enable": False,
    #         "rotation_filter.rotation": 0.0,
    #         "spatial_filter.enable": False,
    #         "temporal_filter.enable": False,
    #         "disparity_filter.enable": False,
    #         "hole_filling_filter.enable": False,
    #         "hdr_merge.enable": False,

    #         # Reconnection
    #         "wait_for_device_timeout": -1.0,
    #         "reconnect_timeout": 6.0,

    #         # Safety / mapping cameras (supported device only)
    #         "enable_safety": False,
    #         "safety_camera.safety_mode": 0,
    #         "enable_labeled_point_cloud": False,
    #         "depth_mapping_camera.labeled_point_cloud_profile": "0,0,0",
    #         "enable_occupancy": False,
    #         "depth_mapping_camera.occupancy_profile": "0,0,0",
    #     }],
    # )

为什么我在一个终端进行ros2 launch top_pkg bringup_sensor.launch.py启动相机雷达加rviz显示之后
相机数据有明显迟钝
在另一个终端进行ros2 topic hz /camera/camera/color/image_raw之后
相机数据明显流畅，但这个终端卡住没有输出