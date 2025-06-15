#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>

class ProjectionNode : public rclcpp::Node {
public:
  ProjectionNode() : Node("projection_node") {
    declare_parameter("intrinsic_yaml", "config/camera_intrinsics.yaml");
    declare_parameter("extrinsic_yaml", "config/extrinsic_calib.yaml");

    get_parameter("intrinsic_yaml", intrinsic_path_);
    get_parameter("extrinsic_yaml", extrinsic_path_);

    if (!load_calibration()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load calibration files.");
      rclcpp::shutdown();
    }

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&ProjectionNode::image_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10,
      std::bind(&ProjectionNode::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "📡 Projecting LiDAR onto camera stream...");
  }

private:
  bool load_calibration() {
    cv::FileStorage fs1(intrinsic_path_, cv::FileStorage::READ);
    cv::FileStorage fs2(extrinsic_path_, cv::FileStorage::READ);
    if (!fs1.isOpened() || !fs2.isOpened()) return false;

    fs1["camera_matrix"] >> camera_matrix_;
    fs1["distortion_coefficients"] >> dist_coeffs_;
    fs2["rotation_vector"] >> rvec_;
    fs2["translation_vector"] >> tvec_;

    cv::Rodrigues(rvec_, rmat_);
    fs1.release(); fs2.release();
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    image_ready_ = true;
    try_project();
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::fromROSMsg(*msg, cloud_);
    cloud_ready_ = true;
    try_project();
  }

  void try_project() {
    if (!image_ready_ || !cloud_ready_) return;

    cv::Mat image = latest_image_.clone();
    for (const auto& pt : cloud_.points) {
      cv::Mat pt_lidar = (cv::Mat_<double>(3,1) << pt.x, pt.y, pt.z);
      cv::Mat pt_cam = rmat_ * pt_lidar + tvec_;

      std::vector<cv::Point3f> object_point = {cv::Point3f(pt_cam)};
      std::vector<cv::Point2f> projected_point;
      cv::projectPoints(object_point, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0),
                        camera_matrix_, dist_coeffs_, projected_point);

      cv::Point2f p = projected_point[0];
      if (p.x > 0 && p.x < image.cols && p.y > 0 && p.y < image.rows)
        cv::circle(image, p, 2, cv::Scalar(0, 255, 0), -1);
    }

    cv::imshow("LiDAR Projection", image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  std::string intrinsic_path_, extrinsic_path_;
  cv::Mat camera_matrix_, dist_coeffs_, rvec_, tvec_, rmat_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  cv::Mat latest_image_;
  bool image_ready_ = false;
  bool cloud_ready_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
