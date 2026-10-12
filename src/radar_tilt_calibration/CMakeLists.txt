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
find_package(yaml-cpp REQUIRED)

add_executable(calibrate_tilt_node src/calibrate_tilt_node.cpp)
ament_target_dependencies(calibrate_tilt_node
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)
target_link_libraries(calibrate_tilt_node
  Eigen3::Eigen
  yaml-cpp
)

add_executable(transform_to_world_node src/transform_to_world_node.cpp)
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
  yaml-cpp
)

install(TARGETS
  calibrate_tilt_node
  transform_to_world_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
