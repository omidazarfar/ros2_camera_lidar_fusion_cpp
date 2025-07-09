#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

namespace fs = std::filesystem;

class ImageCloudCorrespondenceNode : public rclcpp::Node {
public:
    ImageCloudCorrespondenceNode() : Node("image_cloud_correspondence_node") {
        std::string config_path = declare_parameter<std::string>("config_file", "config/general.yaml");
        YAML::Node config = YAML::LoadFile(config_path);

        data_dir_ = config["general"]["data_folder"].as<std::string>();
        correspondence_file_ = config["general"]["correspondence_file"].as<std::string>();

        if (!fs::exists(data_dir_)) {
            RCLCPP_WARN(this->get_logger(), "Data directory does not exist. Creating it.");
            fs::create_directories(data_dir_);
        }

        RCLCPP_INFO(this->get_logger(), "Looking for .png and .pcd file pairs in '%s'", data_dir_.c_str());
        process_file_pairs();
    }

private:
    std::string data_dir_;
    std::string correspondence_file_;

    std::vector<std::tuple<std::string, std::string, std::string>> get_file_pairs(const std::string& directory) {
        std::map<std::string, std::pair<std::string, std::string>> file_map;

        for (const auto& entry : fs::directory_iterator(directory)) {
            if (!fs::is_regular_file(entry)) continue;
            auto path = entry.path();
            auto stem = path.stem().string();
            auto ext = path.extension().string();

            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".pcd") {
                if (ext == ".png")
                    file_map[stem].first = path.string();
                else if (ext == ".pcd")
                    file_map[stem].second = path.string();
            }
        }

        std::vector<std::tuple<std::string, std::string, std::string>> file_pairs;
        for (const auto& [prefix, paths] : file_map) {
            if (!paths.first.empty() && !paths.second.empty())
                file_pairs.emplace_back(prefix, paths.first, paths.second);
        }

        std::sort(file_pairs.begin(), file_pairs.end());
        return file_pairs;
    }

    std::vector<cv::Point2i> pick_image_points(const std::string& img_path) {
        cv::Mat img = cv::imread(img_path);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error loading image: %s", img_path.c_str());
            return {};
        }

        std::vector<cv::Point2i> points;
        std::string window_name = "Select image points (press q or ESC to finish)";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        cv::setMouseCallback(window_name, [](int event, int x, int y, int, void* userdata) {
            if (event == cv::EVENT_LBUTTONDOWN) {
                auto* pts = reinterpret_cast<std::vector<cv::Point2i>*>(userdata);
                pts->emplace_back(x, y);
                std::cout << "Clicked at: (" << x << ", " << y << ")\n";
            }
        }, &points);

        while (true) {
            cv::Mat display = img.clone();
            for (auto pt : points)
                cv::circle(display, pt, 5, cv::Scalar(0, 0, 255), -1);
            cv::imshow(window_name, display);
            int key = cv::waitKey(10);
            if (key == 27 || key == 'q') break;
        }

        cv::destroyWindow(window_name);
        return points;
    }

    std::vector<Eigen::Vector3d> pick_cloud_points(const std::string& pcd_path) {
        open3d::geometry::PointCloud cloud;
        if (!open3d::io::ReadPointCloud(pcd_path, cloud)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read PCD: %s", pcd_path.c_str());
            return {};
        }

        RCLCPP_INFO(this->get_logger(), "Shift+Click to select. Press q to finish.");

        open3d::visualization::VisualizerWithEditing vis;
        vis.CreateVisualizerWindow("Select cloud points", 1280, 720);
        vis.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(cloud));
        vis.Run();
        vis.DestroyVisualizerWindow();

        std::vector<Eigen::Vector3d> picked_points;
        auto picked_indices = vis.GetPickedPoints();
        for (auto idx : picked_indices) {
            picked_points.push_back(cloud.points_[idx]);
        }

        return picked_points;
    }

    void process_file_pairs() {
        auto pairs = get_file_pairs(data_dir_);
        if (pairs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No .png / .pcd pairs found.");
            return;
        }

        for (const auto& [prefix, png, pcd] : pairs) {
            RCLCPP_INFO(this->get_logger(), "Processing %s", prefix.c_str());
            auto img_pts = pick_image_points(png);
            auto cloud_pts = pick_cloud_points(pcd);

            std::string out_file = data_dir_ + "/" + correspondence_file_;
            std::ofstream out(out_file);
            out << "# u, v, x, y, z\n";
            int count = std::min(img_pts.size(), cloud_pts.size());
            for (int i = 0; i < count; ++i) {
                auto& uv = img_pts[i];
                auto& xyz = cloud_pts[i];
                out << uv.x << "," << uv.y << "," << xyz.x() << "," << xyz.y() << "," << xyz.z() << "\n";
            }
            RCLCPP_INFO(this->get_logger(), "Saved %d correspondences to %s", count, out_file.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Processing complete.");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCloudCorrespondenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
