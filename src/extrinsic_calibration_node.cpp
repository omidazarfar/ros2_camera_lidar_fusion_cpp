#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>

class ExtrinsicCalibrationNode : public rclcpp::Node {
public:
  ExtrinsicCalibrationNode() : Node("extrinsic_calibration_node") {
    declare_parameter("selected_image", "data/selected/image.png");
    declare_parameter("selected_lidar", "data/selected/lidar.pcd");
    get_parameter("selected_image", image_path_);
    get_parameter("selected_lidar", lidar_path_);

    RCLCPP_INFO(this->get_logger(), "📂 Loading data from: %s and %s", image_path_.c_str(), lidar_path_.c_str());

    run_icp_calibration();
  }

private:
  void run_icp_calibration() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(lidar_path_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "❌ Could not read LiDAR file.");
      return;
    }

    // Simulate camera cloud from image (mock point cloud)
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < 100; ++i) {
      camera_cloud->push_back(pcl::PointXYZ(i * 0.01, i * 0.01, 1.0));
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(camera_cloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.hasConverged()) {
      Eigen::Matrix4f transformation = icp.getFinalTransformation();
      save_extrinsics(transformation);
      RCLCPP_INFO(this->get_logger(), "✅ Extrinsic calibration done. Score: %.5f", icp.getFitnessScore());
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ ICP failed to converge.");
    }
  }

  void save_extrinsics(const Eigen::Matrix4f &mat) {
    std::filesystem::create_directories("config");
    cv::FileStorage fs("config/extrinsics.yaml", cv::FileStorage::WRITE);
    cv::Mat ext(4, 4, CV_32F, (void*)mat.data());
    fs << "extrinsic_matrix" << ext;
    fs.release();
  }

  std::string image_path_, lidar_path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtrinsicCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
