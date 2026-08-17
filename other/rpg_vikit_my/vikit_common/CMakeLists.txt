cmake_minimum_required(VERSION 3.10)
project(vikit_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/abstract_camera.cpp
  src/pinhole_camera.cpp
  src/vision.cpp
  src/math_utils.cpp
  src/camera_loader.cpp
  src/homography.cpp
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
endif()

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp OpenCV Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
