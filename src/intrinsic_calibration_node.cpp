#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

class IntrinsicCalibrationNode : public rclcpp::Node {
public:
  IntrinsicCalibrationNode()
  : Node("intrinsic_calibration_node"),
    image_count_(0),
    collected_frames_(0),
    max_frames_(15),
    pattern_size_(9, 6),
    square_size_(0.025) {

    declare_parameter("image_topic", "/camera/image_raw");
    get_parameter("image_topic", image_topic_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&IntrinsicCalibrationNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "📸 Subscribed to: %s", image_topic_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(image, pattern_size_, corners);

    if (found) {
      cv::drawChessboardCorners(image, pattern_size_, corners, found);
      image_points_.push_back(corners);
      object_points_.push_back(create_object_points());
      collected_frames_++;
    }

    cv::imshow("Calibration", image);
    cv::waitKey(1);

    if (collected_frames_ >= max_frames_) {
      calibrate_camera(image.size());
      rclcpp::shutdown();
    }
  }

  std::vector<cv::Point3f> create_object_points() {
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < pattern_size_.height; ++i)
      for (int j = 0; j < pattern_size_.width; ++j)
        obj.emplace_back(j * square_size_, i * square_size_, 0);
    return obj;
  }

  void calibrate_camera(cv::Size image_size) {
    cv::Mat camera_matrix, dist_coeffs, rvecs, tvecs;
    double error = cv::calibrateCamera(object_points_, image_points_, image_size,
                                       camera_matrix, dist_coeffs, rvecs, tvecs);

    std::filesystem::create_directories("config");
    cv::FileStorage fs("config/camera_intrinsics.yaml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << dist_coeffs;
    fs << "reprojection_error" << error;
    fs.release();

    RCLCPP_INFO(this->get_logger(), "✅ Intrinsic calibration saved.");
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::string image_topic_;

  cv::Size pattern_size_;
  float square_size_;
  int image_count_;
  int collected_frames_;
  int max_frames_;
  std::vector<std::vector<cv::Point3f>> object_points_;
  std::vector<std::vector<cv::Point2f>> image_points_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntrinsicCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
