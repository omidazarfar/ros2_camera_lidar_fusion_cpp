#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <chrono>

using std::placeholders::_1;

class CameraCalibrationNode : public rclcpp::Node {
public:
    CameraCalibrationNode()
    : Node("camera_calibration_node")
    {
        // Load config
        std::string config_file_path = declare_parameter<std::string>("config_file", "config/general.yaml");
        YAML::Node config = YAML::LoadFile(config_file_path);

        chessboard_rows_ = config["chessboard"]["pattern_size"]["rows"].as<int>();
        chessboard_cols_ = config["chessboard"]["pattern_size"]["columns"].as<int>();
        square_size_ = config["chessboard"]["square_size_meters"].as<float>();

        image_topic_ = config["camera"]["image_topic"].as<std::string>();
        image_width_ = config["camera"]["image_size"]["width"].as<int>();
        image_height_ = config["camera"]["image_size"]["height"].as<int>();

        output_path_ = config["general"]["config_folder"].as<std::string>();
        output_file_ = config["general"]["camera_intrinsic_calibration"].as<std::string>();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&CameraCalibrationNode::image_callback, this, _1));

        // Prepare object points grid
        for (int i = 0; i < chessboard_rows_; ++i)
            for (int j = 0; j < chessboard_cols_; ++j)
                objp_.emplace_back(j * square_size_, i * square_size_, 0.0f);

        RCLCPP_INFO(this->get_logger(), "Camera calibration node started. Waiting for images...");
    }

    ~CameraCalibrationNode() {
        save_calibration();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, cv::Size(chessboard_cols_, chessboard_rows_), corners);

            if (found) {
                cv::cornerSubPix(
                    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

                img_points_.push_back(corners);
                obj_points_.push_back(objp_);

                cv::drawChessboardCorners(cv_ptr->image, cv::Size(chessboard_cols_, chessboard_rows_), corners, found);
                RCLCPP_INFO(this->get_logger(), "Chessboard detected and points added.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Chessboard not detected in current image.");
            }

            cv::imshow("Camera View", cv_ptr->image);
            cv::waitKey(1);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
        }
    }

    void save_calibration() {
        if (obj_points_.size() < 10) {
            RCLCPP_ERROR(this->get_logger(), "Not enough samples for calibration.");
            return;
        }

        cv::Mat camera_matrix, dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;

        double error = cv::calibrateCamera(obj_points_, img_points_, cv::Size(image_width_, image_height_),
                                           camera_matrix, dist_coeffs, rvecs, tvecs);

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "calibration_date" << YAML::Value << get_timestamp();

        out << YAML::Key << "camera_matrix" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 3;
        out << YAML::Key << "columns" << YAML::Value << 3;
        out << YAML::Key << "data" << YAML::Value << std::vector<double>(camera_matrix.begin<double>(), camera_matrix.end<double>());
        out << YAML::EndMap;

        out << YAML::Key << "distortion_coefficients" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "rows" << YAML::Value << 1;
        out << YAML::Key << "columns" << YAML::Value << dist_coeffs.cols;
        out << YAML::Key << "data" << YAML::Value << std::vector<double>(dist_coeffs.begin<double>(), dist_coeffs.end<double>());
        out << YAML::EndMap;

        out << YAML::Key << "rms_reprojection_error" << YAML::Value << error;
        out << YAML::EndMap;

        std::ofstream fout(output_path_ + "/" + output_file_);
        fout << out.c_str();
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Calibration saved to: %s", (output_path_ + "/" + output_file_).c_str());
    }

    std::string get_timestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        char buffer[64];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time));
        return std::string(buffer);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    int chessboard_rows_, chessboard_cols_;
    float square_size_;
    int image_width_, image_height_;
    std::string image_topic_, output_path_, output_file_;

    std::vector<cv::Point3f> objp_;
    std::vector<std::vector<cv::Point3f>> obj_points_;
    std::vector<std::vector<cv::Point2f>> img_points_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
