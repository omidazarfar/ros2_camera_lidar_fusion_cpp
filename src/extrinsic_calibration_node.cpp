#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>

class ExtrinsicCalibrationNode : public rclcpp::Node {
public:
  ExtrinsicCalibrationNode() : Node("extrinsic_calibration_node") {
    declare_parameter("input_yaml", "config/point_pairs.yaml");
    declare_parameter("intrinsic_yaml", "config/camera_intrinsics.yaml");
    declare_parameter("output_yaml", "config/extrinsic_calib.yaml");

    get_parameter("input_yaml", input_path_);
    get_parameter("intrinsic_yaml", intrinsic_path_);
    get_parameter("output_yaml", output_path_);

    RCLCPP_INFO(this->get_logger(), "📥 Loading input from %s", input_path_.c_str());

    if (!load_point_pairs()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load point pairs.");
      rclcpp::shutdown();
      return;
    }

    if (!load_camera_intrinsics()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load intrinsics.");
      rclcpp::shutdown();
      return;
    }

    run_calibration();
    rclcpp::shutdown();
  }

private:
  bool load_point_pairs() {
    YAML::Node config = YAML::LoadFile(input_path_);
    if (!config["image_points"] || !config["lidar_points"])
      return false;

    for (const auto& pt : config["image_points"])
      image_points_.emplace_back(pt[0].as<float>(), pt[1].as<float>());

    for (const auto& pt : config["lidar_points"])
      lidar_points_.emplace_back(pt[0].as<float>(), pt[1].as<float>(), pt[2].as<float>());

    return image_points_.size() == lidar_points_.size() && !image_points_.empty();
  }

  bool load_camera_intrinsics() {
    cv::FileStorage fs(intrinsic_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    return true;
  }

  void run_calibration() {
    cv::Mat rvec, tvec;
    cv::solvePnP(lidar_points_, image_points_, camera_matrix_, dist_coeffs_, rvec, tvec);
    
    std::filesystem::create_directories("config");
    cv::FileStorage fs(output_path_, cv::FileStorage::WRITE);
    fs << "rotation_vector" << rvec;
    fs << "translation_vector" << tvec;
    fs.release();

    RCLCPP_INFO(this->get_logger(), "✅ Extrinsic calibration saved to %s", output_path_.c_str());
  }

  std::string input_path_, output_path_, intrinsic_path_;
  std::vector<cv::Point2f> image_points_;
  std::vector<cv::Point3f> lidar_points_;
  cv::Mat camera_matrix_, dist_coeffs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtrinsicCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
