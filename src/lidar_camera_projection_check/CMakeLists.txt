cmake_minimum_required(VERSION 3.8)
project(lidar_camera_projection_check)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(projection_check_node src/projection_check_node.cpp)
target_compile_features(projection_check_node PUBLIC cxx_std_17)
target_link_libraries(projection_check_node Eigen3::Eigen ${OpenCV_LIBS})
ament_target_dependencies(projection_check_node
  cv_bridge
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(TARGETS projection_check_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
