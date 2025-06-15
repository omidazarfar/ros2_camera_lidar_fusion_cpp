# 🔁 ros2_camera_lidar_fusion_cpp

A full-featured **C++ ROS 2** reproduction of the Python-based [CDonosoK/ros2_camera_lidar_fusion](https://github.com/CDonosoK/ros2_camera_lidar_fusion) package :contentReference[oaicite:1]{index=1}.  
This repository offers:
- Intrinsic and extrinsic calibration for camera + LiDAR.
- Manual point correspondence selection.
- Final LiDAR point projection onto camera images for precise sensor fusion.
  
*Originally designed for robotics, autonomous vehicles, and sensor integration.*

---

## 📁 Directory Structure

ros2_camera_lidar_fusion_cpp/
├── config/ # Parameter definitions and generated YAMLs
│ └── general.yaml # Topics, directories, and file paths
├── data/ # Saved frames and selected files
│ ├── images/
│ ├── lidar/
│ ├── selected/
│ └── fused/
├── src/ # 5 ROS 2 nodes in C++
├── launch/ # Corresponding launch files (*.launch.py)
├── include/ # Header files (optional)
├── CMakeLists.txt
├── package.xml
└── README.md

yaml
Copy
Edit

---

## ⚙️ Configuration – `config/general.yaml`

```yaml
camera_topic: "/camera/image_raw"
lidar_topic: "/lidar/points"
image_directory: "data/images"
lidar_directory: "data/lidar"
selected_image: "data/selected/image.png"
selected_lidar: "data/selected/lidar.pcd"
camera_intrinsics_file: "config/camera_intrinsics.yaml"
extrinsics_file: "config/extrinsics.yaml"
All ROS nodes automatically load these topics and paths—no changes needed in code for different setups.

🚀 Usage Instructions
1. Build the Package
bash
Copy
Edit
cd ~/ros2_ws
colcon build --packages-select ros2_camera_lidar_fusion_cpp
source install/setup.bash
2. Nodes Overview & Launch Commands
Step	Node & Launch Command
1. Intrinsic Calibration	Calibrates the camera using a checkerboard<br>ros2 launch ros2_camera_lidar_fusion_cpp intrinsic.launch.py
2. Data Recording	Capture synchronized camera and LiDAR data<br>ros2 launch ros2_camera_lidar_fusion_cpp data_saver.launch.py
3. Manual Frame Selection	Manually choose matching image + point cloud<br>ros2 launch ros2_camera_lidar_fusion_cpp point_selector.launch.py
4. Extrinsic Calibration	Calculates the transformation between LiDAR and camera<br>ros2 launch ros2_camera_lidar_fusion_cpp extrinsic.launch.py
5. Projection & Fusion	Projects LiDAR onto camera and saves result<br>ros2 launch ros2_camera_lidar_fusion_cpp projection.launch.py

🛠 Generate Intermediate Files
config/camera_intrinsics.yaml — created in step 1

data/images/*.png, data/lidar/*.pcd — Step 2

data/selected/* — Step 3

config/extrinsics.yaml — Step 4

data/fused/fused_projection.png — Step 5

🧩 Why This Matters
In robotics and autonomous systems, accurate calibration is essential for:

Merging 3D LiDAR point clouds with 2D camera images

Reliable depth estimation, object detection, and mapping

Seamless sensor fusion in perception pipelines

This pipeline offers a clean, modular implementation for developing real-time robotic systems.

📚 References & Inspiration
Adapted from the original Python implementation by Clemente Donoso Krauss:

“A ROS2 package for calculating intrinsic and extrinsic calibration … enabling precise projection of LiDAR points into the camera frame” 
github.com
+6
github.com
+6
github.com
+6
arxiv.org

🧾 Dependencies
ROS 2 Humble or later

OpenCV

Point Cloud Library (PCL)

cv_bridge, sensor_msgs, pcl_conversions

C++17 std::filesystem

Install with:

bash
Copy
Edit
sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-pcl-conversions \
  libpcl-dev \
  libopencv-dev
🙋 Tips & Troubleshooting
Ensure the checkerboard pattern aligns with the pattern_size and square_size code settings.

Use Ctrl+C while viewing windows (OpenCV) to close nodes cleanly.

Manual selection supports n = next, s = select, q = quit.

Modify general.yaml to adjust topic names or data directories.

🤝 Contributing
Pull requests are welcome—please follow these steps:

Fork the repo

Create a new branch (git checkout -b feature-name)

Make changes & commit

Submit a pull request

📝 License & Maintenance
MIT License

Maintained by Omid Azarfar

Contact: 📧 omid.azarfar95@gmail.com — 📎 LinkedIn / GitHub profile

⭐ Support
If you find this useful, star ⭐ the repo, share with the community, or drop a quick suggestion!
