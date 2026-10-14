cmake_minimum_required(VERSION 3.8)
project(radar_tilt_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(radar_tilt_core src/tilt_angle_io.cpp)
target_include_directories(radar_tilt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(radar_tilt_core PUBLIC
  ament_index_cpp::ament_index_cpp
  Eigen3::Eigen
  yaml-cpp
)

add_executable(calibrate_tilt_node src/calibrate_tilt_node.cpp)
target_include_directories(calibrate_tilt_node PRIVATE include)
ament_target_dependencies(calibrate_tilt_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(calibrate_tilt_node
  Eigen3::Eigen
  radar_tilt_core
  yaml-cpp
)

add_executable(transform_to_world_node src/transform_to_world_node.cpp)
target_include_directories(transform_to_world_node PRIVATE include)
ament_target_dependencies(transform_to_world_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(transform_to_world_node
  Eigen3::Eigen
  radar_tilt_core
  yaml-cpp
)

add_executable(world_tf_bridge_node src/world_tf_bridge_node.cpp)
target_include_directories(world_tf_bridge_node PRIVATE include)
ament_target_dependencies(world_tf_bridge_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)
target_link_libraries(world_tf_bridge_node
  Eigen3::Eigen
  radar_tilt_core
  yaml-cpp
)

install(TARGETS
  radar_tilt_core
  calibrate_tilt_node
  transform_to_world_node
  world_tf_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
