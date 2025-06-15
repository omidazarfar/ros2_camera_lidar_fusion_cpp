#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

class ManualPointSelectorNode : public rclcpp::Node {
public:
  ManualPointSelectorNode() : Node("manual_point_selector_node") {
    declare_parameter("image_path", "data/frame_0.png");
    declare_parameter("output_yaml", "config/point_pairs.yaml");

    get_parameter("image_path", image_path_);
    get_parameter("output_yaml", output_path_);

    RCLCPP_INFO(this->get_logger(), "📍 Loading image: %s", image_path_.c_str());
    image_ = cv::imread(image_path_);
    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load image");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "🖱️ Click points on the image. Press 'q' when done.");
    cv::namedWindow("Select Points");
    cv::setMouseCallback("Select Points", on_mouse, this);

    while (true) {
      cv::Mat temp = image_.clone();
      for (const auto& pt : clicked_points_)
        cv::circle(temp, pt, 5, cv::Scalar(0, 255, 0), -1);
      cv::imshow("Select Points", temp);

      char key = cv::waitKey(10);
      if (key == 'q' || key == 27)
        break;
    }

    collect_3d_points();
    save_points();
    rclcpp::shutdown();
  }

private:
  static void on_mouse(int event, int x, int y, int, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
      auto* self = static_cast<ManualPointSelectorNode*>(userdata);
      self->clicked_points_.emplace_back(x, y);
      RCLCPP_INFO(self->get_logger(), "🟢 Selected 2D point: (%d, %d)", x, y);
    }
  }

  void collect_3d_points() {
    std::cout << "🧠 Now enter matching 3D points for each 2D image point (format: x y z):" << std::endl;
    for (size_t i = 0; i < clicked_points_.size(); ++i) {
      float x, y, z;
      std::cout << "Point " << i + 1 << ": ";
      std::cin >> x >> y >> z;
      corresponding_3d_points_.emplace_back(x, y, z);
    }
  }

  void save_points() {
    std::filesystem::create_directories("config");
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image_points" << YAML::Value << YAML::BeginSeq;
    for (const auto& pt : clicked_points_)
      out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << YAML::EndSeq;
    out << YAML::EndSeq;

    out << YAML::Key << "lidar_points" << YAML::Value << YAML::BeginSeq;
    for (const auto& pt : corresponding_3d_points_)
      out << YAML::Flow << YAML::BeginSeq << pt.x << pt.y << pt.z << YAML::EndSeq;
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(output_path_);
    fout << out.c_str();
    fout.close();

    RCLCPP_INFO(this->get_logger(), "✅ Saved point pairs to %s", output_path_.c_str());
  }

  std::string image_path_;
  std::string output_path_;
  cv::Mat image_;
  std::vector<cv::Point2f> clicked_points_;
  std::vector<cv::Point3f> corresponding_3d_points_;
};
  
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManualPointSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
