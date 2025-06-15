#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>

class SensorDataSaverNode : public rclcpp::Node {
public:
  SensorDataSaverNode()
  : Node("sensor_data_saver_node"), frame_count_(0) {
    declare_parameter("image_topic", "/camera/image_raw");
    declare_parameter("lidar_topic", "/lidar/points");
    declare_parameter("save_rate", 5);
    get_parameter("image_topic", image_topic_);
    get_parameter("lidar_topic", lidar_topic_);
    get_parameter("save_rate", save_rate_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&SensorDataSaverNode::image_callback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 10, std::bind(&SensorDataSaverNode::lidar_callback, this, std::placeholders::_1));

    std::filesystem::create_directories("data/images");
    std::filesystem::create_directories("data/lidar");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (frame_count_ % save_rate_ == 0) {
      std::string filename = "data/images/frame_" + std::to_string(frame_count_) + ".png";
      cv::imwrite(filename, cv_bridge::toCvShare(msg, "bgr8")->image);
      RCLCPP_INFO(this->get_logger(), "📷 Saved %s", filename.c_str());
    }
    frame_count_++;
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (frame_count_ % save_rate_ == 0) {
      pcl::PCLPointCloud2 cloud;
      pcl_conversions::toPCL(*msg, cloud);
      std::string filename = "data/lidar/frame_" + std::to_string(frame_count_) + ".pcd";
      pcl::io::savePCDFileBinary(filename, cloud);
      RCLCPP_INFO(this->get_logger(), "🧊 Saved %s", filename.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  std::string image_topic_, lidar_topic_;
  int save_rate_;
  int frame_count_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSaverNode>());
  rclcpp::shutdown();
  return 0;
}
