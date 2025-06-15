#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>

class ManualSelectorNode : public rclcpp::Node {
public:
  ManualSelectorNode()
  : Node("manual_selector_node"),
    image_index_(0),
    lidar_index_(0) {

    declare_parameter("image_directory", "data/images");
    declare_parameter("lidar_directory", "data/lidar");
    get_parameter("image_directory", image_dir_);
    get_parameter("lidar_directory", lidar_dir_);

    load_file_lists();

    if (image_files_.empty() || lidar_files_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "❌ No data found to select from.");
      rclcpp::shutdown();
    }

    cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
    show_current();

    RCLCPP_INFO(this->get_logger(), "🧩 Manual selector initialized. Press [s] to save, [n] for next, [q] to quit.");
    run_loop();
  }

private:
  void load_file_lists() {
    for (const auto &entry : std::filesystem::directory_iterator(image_dir_))
      if (entry.path().extension() == ".png") image_files_.push_back(entry.path().string());

    for (const auto &entry : std::filesystem::directory_iterator(lidar_dir_))
      if (entry.path().extension() == ".pcd") lidar_files_.push_back(entry.path().string());

    std::sort(image_files_.begin(), image_files_.end());
    std::sort(lidar_files_.begin(), lidar_files_.end());
  }

  void show_current() {
    if (image_index_ < image_files_.size()) {
      cv::Mat img = cv::imread(image_files_[image_index_]);
      cv::imshow("Image Viewer", img);
    }
  }

  void run_loop() {
    char key;
    while (rclcpp::ok()) {
      key = static_cast<char>(cv::waitKey(0));

      if (key == 'q') {
        RCLCPP_INFO(this->get_logger(), "👋 Exiting manual selector.");
        break;
      } else if (key == 'n') {
        image_index_ = std::min(image_index_ + 1, static_cast<int>(image_files_.size() - 1));
        lidar_index_ = std::min(lidar_index_ + 1, static_cast<int>(lidar_files_.size() - 1));
        show_current();
      } else if (key == 's') {
        std::filesystem::copy(image_files_[image_index_], "data/selected/image.png", std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy(lidar_files_[lidar_index_], "data/selected/lidar.pcd", std::filesystem::copy_options::overwrite_existing);
        RCLCPP_INFO(this->get_logger(), "✅ Selected pair saved.");
      }
    }
  }

  std::string image_dir_, lidar_dir_;
  std::vector<std::string> image_files_, lidar_files_;
  int image_index_, lidar_index_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
