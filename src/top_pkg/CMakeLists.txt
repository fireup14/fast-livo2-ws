cmake_minimum_required(VERSION 3.8)
project(top_pkg)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tf_lidar_data src/tf_lidar_data.cpp)
target_compile_features(tf_lidar_data PUBLIC cxx_std_17)
target_include_directories(tf_lidar_data PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tf_lidar_data
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(TARGETS tf_lidar_data
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
