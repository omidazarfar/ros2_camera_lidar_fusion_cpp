cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Open3D REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_calibration_node src/camera_calibration_node.cpp)
add_executable(save_data_node src/save_data_node.cpp)
add_executable(image_cloud_correspondence_node src/image_cloud_correspondence_node.cpp)
add_executable(camera_lidar_extrinsic_node src/camera_lidar_extrinsic_node.cpp)
add_executable(lidar_camera_projection_node src/lidar_camera_projection_node.cpp)

ament_target_dependencies(camera_calibration_node
  rclcpp sensor_msgs cv_bridge yaml-cpp
)

ament_target_dependencies(save_data_node
  rclcpp sensor_msgs cv_bridge yaml-cpp message_filters pcl_conversions
)

ament_target_dependencies(image_cloud_correspondence_node
  rclcpp yaml-cpp Open3D
)

ament_target_dependencies(camera_lidar_extrinsic_node
  rclcpp yaml-cpp OpenCV
)

ament_target_dependencies(lidar_camera_projection_node
  rclcpp sensor_msgs cv_bridge yaml-cpp message_filters OpenCV
)

target_link_libraries(camera_calibration_node ${OpenCV_LIBRARIES})
target_link_libraries(save_data_node ${OpenCV_LIBRARIES} Open3D::Open3D)
target_link_libraries(image_cloud_correspondence_node ${OpenCV_LIBRARIES} Open3D::Open3D)
target_link_libraries(camera_lidar_extrinsic_node ${OpenCV_LIBRARIES})
target_link_libraries(lidar_camera_projection_node ${OpenCV_LIBRARIES})

install(TARGETS
  camera_calibration_node
  save_data_node
  image_cloud_correspondence_node
  camera_lidar_extrinsic_node
  lidar_camera_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
