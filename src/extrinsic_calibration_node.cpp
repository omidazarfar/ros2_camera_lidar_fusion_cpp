#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem;

class CameraLidarExtrinsicNode : public rclcpp::Node {
public:
    CameraLidarExtrinsicNode() : Node("camera_lidar_extrinsic_node") {
        std::string config_path = declare_parameter<std::string>("config_file", "config/general.yaml");
        YAML::Node config = YAML::LoadFile(config_path);

        corr_file_ = "/ros2_ws/src/ros2_camera_lidar_fusion/data/" + config["general"]["correspondence_file"].as<std::string>();
        camera_yaml_ = "/ros2_ws/src/ros2_camera_lidar_fusion/config/" + config["general"]["camera_intrinsic_calibration"].as<std::string>();
        output_dir_ = config["general"]["config_folder"].as<std::string>();
        output_file_ = config["general"]["camera_extrinsic_calibration"].as<std::string>();

        RCLCPP_INFO(this->get_logger(), "Starting extrinsic calibration...");
        solve_extrinsic_with_pnp();
    }

private:
    std::string corr_file_, camera_yaml_, output_dir_, output_file_;

    std::pair<cv::Mat, cv::Mat> load_camera_calibration(const std::string& yaml_path) {
        if (!fs::exists(yaml_path)) {
            throw std::runtime_error("Camera YAML file not found: " + yaml_path);
        }

        YAML::Node calib = YAML::LoadFile(yaml_path);
        auto cam_data = calib["camera_matrix"]["data"].as<std::vector<double>>();
        auto dist_data = calib["distortion_coefficients"]["data"].as<std::vector<double>>();

        cv::Mat camera_matrix = cv::Mat(cam_data).reshape(1, 3);
        cv::Mat dist_coeffs = cv::Mat(dist_data).reshape(1, 1);
        return {camera_matrix, dist_coeffs};
    }

    void solve_extrinsic_with_pnp() {
        auto [camera_matrix, dist_coeffs] = load_camera_calibration(camera_yaml_);
        RCLCPP_INFO(this->get_logger(), "Camera matrix:\n%s", cv::format(camera_matrix, cv::Formatter::FMT_DEFAULT).c_str());

        std::vector<cv::Point2d> pts_2d;
        std::vector<cv::Point3d> pts_3d;

        std::ifstream fin(corr_file_);
        if (!fin.is_open()) {
            throw std::runtime_error("Could not open correspondence file: " + corr_file_);
        }

        std::string line;
        while (std::getline(fin, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            std::vector<std::string> tokens;
            std::string item;
            while (std::getline(ss, item, ',')) {
                tokens.push_back(item);
            }

            if (tokens.size() != 5) continue;

            pts_2d.emplace_back(std::stod(tokens[0]), std::stod(tokens[1]));
            pts_3d.emplace_back(std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4]));
        }

        if (pts_2d.size() < 4) {
            throw std::runtime_error("At least 4 correspondences required for solvePnP.");
        }

        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(pts_3d, pts_2d, camera_matrix, dist_coeffs, rvec, tvec);

        if (!success) {
            throw std::runtime_error("solvePnP failed to compute a solution.");
        }

        RCLCPP_INFO(this->get_logger(), "solvePnP succeeded.");

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(extrinsic(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(extrinsic(cv::Rect(3, 0, 1, 3)));

        RCLCPP_INFO(this->get_logger(), "Transformation matrix (LiDAR -> Camera):\n%s", cv::format(extrinsic, cv::Formatter::FMT_DEFAULT).c_str());

        if (!fs::exists(output_dir_)) {
            fs::create_directories(output_dir_);
        }

        std::string out_path = output_dir_ + "/" + output_file_;
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "extrinsic_matrix" << YAML::Value << YAML::Flow << std::vector<double>(extrinsic.begin<double>(), extrinsic.end<double>());
        out << YAML::EndMap;

        std::ofstream fout(out_path);
        fout << out.c_str();
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Extrinsic matrix saved to: %s", out_path.c_str());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraLidarExtrinsicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
