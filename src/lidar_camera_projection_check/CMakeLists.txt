cmake_minimum_required(VERSION 3.8)

project(lidar_camera_projection_check)

find_package(ament_cmake REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  projection_check_component
  SHARED
  src/projection_check_node.cpp
)


target_compile_features(
  projection_check_component
  PUBLIC
  cxx_std_17
)


target_link_libraries(
  projection_check_component

  Eigen3::Eigen

  ${OpenCV_LIBS}

  cv_bridge::cv_bridge
)


ament_target_dependencies(
  projection_check_component

  rclcpp
  rclcpp_components
  sensor_msgs
  livox_ros_driver2
)


rclcpp_components_register_node(
  projection_check_component
  PLUGIN
  "lidar_camera_projection_check::LidarCameraProjectionCheck"
  EXECUTABLE projection_check_component_loader
)

add_executable(
  projection_check_node
  src/projection_check_node.cpp
)

target_compile_features(
  projection_check_node
  PUBLIC
  cxx_std_17
)

target_compile_definitions(
  projection_check_node
  PRIVATE
  PROJECTION_CHECK_STANDALONE
)

target_link_libraries(
  projection_check_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
)

ament_target_dependencies(
  projection_check_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

install(
  TARGETS projection_check_component projection_check_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY
  launch

  DESTINATION
  share/${PROJECT_NAME}
)


ament_package()
