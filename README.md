# 🔁 ROS2 Camera-LiDAR Fusion (C++)

This repository provides a complete **ROS 2 (C++) pipeline** for calibrating and fusing data from a camera and LiDAR sensor. It includes intrinsic and extrinsic calibration, manual frame selection, and LiDAR-to-image projection — designed for real-time robotic perception and autonomous systems.

---

## 📦 Features

- 🔍 **Camera Intrinsic Calibration** using a checkerboard
- 🌐 **Extrinsic Calibration** with point cloud alignment
- 🖼️ **Manual Pair Selection** of synchronized image and LiDAR frames
- 🎯 **Projection** of LiDAR points onto 2D image plane
- 📁 Full configuration via YAML file
- ✅ ROS 2 native, C++17-compliant, modular node architecture

---

🚀 How to Use

1. Build the Package

cd ~/ros2_ws
colcon build --packages-select ros2_camera_lidar_fusion_cpp
source install/setup.bash

2. Run the Nodes
Each node corresponds to a step in the calibration pipeline:

Step	Node	Description
1️⃣	intrinsic_calibration_node	Calibrates camera intrinsics using a checkerboard
2️⃣	sensor_data_saver_node	Saves incoming LiDAR point clouds and camera images
3️⃣	manual_selector_node	Allows manual selection of a matching image + point cloud pair
4️⃣	extrinsic_calibration_node	Computes transform between LiDAR and camera using ICP
5️⃣	projection_node	Projects LiDAR points into camera frame and overlays them on the image

Launch files (optional) can simplify each of these steps.

📂 Outputs

camera_intrinsics.yaml	Step 1	Stores camera matrix & distortion coefficients
images/*.png & lidar/*.pcd	Step 2	Raw synchronized data
selected/*.pcd & selected/*.png	Step 3	User-selected matching frames
extrinsics.yaml	Step 4	Transformation from LiDAR to camera
fused/fused_projection.png	Step 5	Visual projection of 3D points into image plane

🛠 Dependencies
Make sure the following are installed:

sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-pcl-conversions \
  libpcl-dev \
  libopencv-dev
  
Also install required ROS 2 packages (rclcpp, sensor_msgs, cv_bridge, pcl_conversions, etc.).

🤖 Application
This pipeline is ideal for:

Robotics and SLAM

Autonomous vehicles

Sensor fusion and 3D reconstruction

Object detection pipelines that combine 2D and 3D data

🧠 Notes
You can press n, s, and q during manual selection.

Be sure your camera is publishing clear images of a checkerboard for intrinsic calibration.

Intrinsic and extrinsic YAMLs must be generated before projection.

📬 Contact
Maintained by Omid Azarfar
📧 omid.azarfar95@gmail.com
🌐 github.com/omidazarfar

📜 License
This project is released under the MIT License.
