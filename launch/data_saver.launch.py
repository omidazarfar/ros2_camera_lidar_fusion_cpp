#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <chrono>

class SensorDataSaverNode : public rclcpp::Node {
public:
  SensorDataSaverNode()
  : Node("sensor_data_saver_node"),
    image_counter_(0)
  {
    declare_parameter("save_dir", "data");
    get_parameter("save_dir", save_dir_);

    std::filesystem::create_directories(save_dir_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&SensorDataSaverNode::image_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", 10,
      std::bind(&SensorDataSaverNode::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "📸 Saving data to: %s", save_dir_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    image_stamp_ = msg->header.stamp;
    try_save();
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_cloud_ = *msg;
    lidar_stamp_ = msg->header.stamp;
    try_save();
  }

  void try_save() {
    if (!last_cloud_.data.empty() && !last_image_.empty() &&
        lidar_stamp_.sec == image_stamp_.sec &&
        std::abs(lidar_stamp_.nanosec - image_stamp_.nanosec) < 1e7)
    {
      std::stringstream name;
      name << save_dir_ << "/frame_" << image_counter_;

      std::string image_path = name.str() + ".png";
      std::string cloud_path = name.str() + ".pcd";

      cv::imwrite(image_path, last_image_);
      pcl::PCLPointCloud2 cloud;
      pcl_conversions::toPCL(last_cloud_, cloud);
      pcl::io::savePCDFileBinary(cloud_path, cloud);

      RCLCPP_INFO(this->get_logger(), "💾 Saved %s.[png+pcd]", name.str().c_str());
      image_counter_++;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  cv::Mat last_image_;
  sensor_msgs::msg::PointCloud2 last_cloud_;
  rclcpp::Time image_stamp_;
  rclcpp::Time lidar_stamp_;
  std::string save_dir_;
  int image_counter_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSaverNode>());
  rclcpp::shutdown();
  return 0;
}
