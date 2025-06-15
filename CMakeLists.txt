cmake_minimum_required(VERSION 3.5)
project(ros2_camera_lidar_fusion_cpp)

# C++ standard
set(CMAKE_CXX_STANDARD 17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Node sources
add_executable(intrinsic_calibration_node src/intrinsic_calibration_node.cpp)
ament_target_dependencies(intrinsic_calibration_node rclcpp sensor_msgs OpenCV)

add_executable(sensor_data_saver_node src/sensor_data_saver_node.cpp)
ament_target_dependencies(sensor_data_saver_node rclcpp sensor_msgs PCL OpenCV)

add_executable(manual_selector_node src/manual_selector_node.cpp)
ament_target_dependencies(manual_selector_node rclcpp OpenCV PCL)

add_executable(extrinsic_calibration_node src/extrinsic_calibration_node.cpp)
ament_target_dependencies(extrinsic_calibration_node rclcpp PCL OpenCV)

add_executable(projection_node src/projection_node.cpp)
ament_target_dependencies(projection_node rclcpp PCL OpenCV sensor_msgs cv_bridge)

install(TARGETS
  intrinsic_calibration_node
  sensor_data_saver_node
  manual_selector_node
  extrinsic_calibration_node
  projection_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
