cmake_minimum_required(VERSION 3.8)
project(ros2_camera_lidar_fusion_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Executables
add_executable(intrinsic_calibration_node src/intrinsic_calibration_node.cpp)
add_executable(sensor_data_saver_node src/sensor_data_saver_node.cpp)
add_executable(manual_point_selector_node src/manual_point_selector_node.cpp)
add_executable(extrinsic_calibration_node src/extrinsic_calibration_node.cpp)
add_executable(projection_node src/projection_node.cpp)

# Link + dependencies
foreach(node
    intrinsic_calibration_node
    sensor_data_saver_node
    manual_point_selector_node
    extrinsic_calibration_node
    projection_node
)
  ament_target_dependencies(${node}
    rclcpp
    sensor_msgs
    OpenCV
    cv_bridge
    image_transport
    PCL
    pcl_conversions
    yaml-cpp
  )
endforeach()

# Install
install(TARGETS
  intrinsic_calibration_node
  sensor_data_saver_node
  manual_point_selector_node
  extrinsic_calibration_node
  projection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
