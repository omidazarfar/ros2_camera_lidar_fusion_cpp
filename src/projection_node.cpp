#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

class ProjectionNode : public rclcpp::Node {
public:
  ProjectionNode() : Node("projection_node") {
    declare_parameter("camera_intrinsics_file", "config/camera_intrinsics.yaml");
    declare_parameter("extrinsics_file", "config/extrinsics.yaml");
    declare_parameter("lidar_file", "data/selected/lidar.pcd");
    declare_parameter("image_file", "data/selected/image.png");

    get_parameter("camera_intrinsics_file", intrinsics_path_);
    get_parameter("extrinsics_file", extrinsics_path_);
    get_parameter("lidar_file", lidar_path_);
    get_parameter("image_file", image_path_);

    if (!load_parameters()) return;

    project_points();
  }

private:
  bool load_parameters() {
    cv::FileStorage fs1(intrinsics_path_, cv::FileStorage::READ);
    if (!fs1.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Cannot open camera intrinsics file.");
      return false;
    }
    fs1["camera_matrix"] >> K_;
    fs1["distortion_coefficients"] >> dist_;
    fs1.release();

    cv::FileStorage fs2(extrinsics_path_, cv::FileStorage::READ);
    if (!fs2.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Cannot open extrinsics file.");
      return false;
    }
    fs2["extrinsic_matrix"] >> extrinsics_;
    fs2.release();

    return true;
  }

  void project_points() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(lidar_path_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load LiDAR file.");
      return;
    }

    cv::Mat image = cv::imread(image_path_);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load image.");
      return;
    }

    for (const auto& point : cloud->points) {
      cv::Mat pt3D = (cv::Mat_<float>(4,1) << point.x, point.y, point.z, 1.0);
      cv::Mat pt_cam = extrinsics_ * pt3D;
      if (pt_cam.at<float>(2,0) <= 0) continue;

      float x = pt_cam.at<float>(0,0) / pt_cam.at<float>(2,0);
      float y = pt_cam.at<float>(1,0) / pt_cam.at<float>(2,0);

      cv::Mat pt2D;
      cv::projectPoints(cv::Mat(cv::Vec3f(x, y, 1.0)), cv::Mat::zeros(3,1,CV_64F),
                        cv::Mat::zeros(3,1,CV_64F), K_, dist_, pt2D);

      int u = static_cast<int>(pt2D.at<cv::Point2f>(0).x);
      int v = static_cast<int>(pt2D.at<cv::Point2f>(0).y);

      if (u >= 0 && v >= 0 && u < image.cols && v < image.rows) {
        cv::circle(image, cv::Point(u,v), 2, cv::Scalar(0,0,255), -1);
      }
    }

    cv::imwrite("data/fused/fused_projection.png", image);
    RCLCPP_INFO(this->get_logger(), "🎯 Fused image saved to data/fused/fused_projection.png");
  }

  std::string intrinsics_path_, extrinsics_path_, lidar_path_, image_path_;
  cv::Mat K_, dist_, extrinsics_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
