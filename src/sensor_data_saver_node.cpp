#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

class SensorDataSaverNode : public rclcpp::Node {
public:
  SensorDataSaverNode()
  : Node("sensor_data_saver_node"),
    image_saved_(false),
    lidar_saved_(false) {

    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("lidar_topic", "/lidar/points");
    get_parameter("image_topic", image_topic_);
    get_parameter("lidar_topic", lidar_topic_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&SensorDataSaverNode::image_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 10, std::bind(&SensorDataSaverNode::lidar_callback, this, std::placeholders::_1));

    std::filesystem::create_directories("data/images");
    std::filesystem::create_directories("data/lidar");

    RCLCPP_INFO(this->get_logger(), "📡 Listening to: %s and %s", image_topic_.c_str(), lidar_topic_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::string name = current_filename();
      cv::imwrite("data/images/" + name + ".png", img);
      image_saved_ = true;
      RCLCPP_INFO(this->get_logger(), "🖼️ Saved image: %s.png", name.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Image save error: %s", e.what());
    }
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      std::string name = current_filename();
      pcl::io::savePCDFileASCII("data/lidar/" + name + ".pcd", cloud);
      lidar_saved_ = true;
      RCLCPP_INFO(this->get_logger(), "📍 Saved LiDAR: %s.pcd", name.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ LiDAR save error: %s", e.what());
    }
  }

  std::string current_filename() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "frame_%Y%m%d_%H%M%S");
    return ss.str();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  std::string image_topic_, lidar_topic_;
  bool image_saved_, lidar_saved_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSaverNode>());
  rclcpp::shutdown();
  return 0;
}
