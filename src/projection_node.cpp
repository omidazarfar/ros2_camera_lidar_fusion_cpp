#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <filesystem>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;
namespace fs = std::filesystem;
using namespace message_filters;

class LidarCameraProjectionNode : public rclcpp::Node {
public:
    LidarCameraProjectionNode() : Node("lidar_camera_projection_node") {
        std::string config_file = declare_parameter<std::string>("config_file", "config/general.yaml");
        YAML::Node config = YAML::LoadFile(config_file);

        // Load calibration files
        std::string config_folder = config["general"]["config_folder"].as<std::string>();
        std::string extrinsic_path = config_folder + "/" + config["general"]["camera_extrinsic_calibration"].as<std::string>();
        std::string intrinsics_path = config_folder + "/" + config["general"]["camera_intrinsic_calibration"].as<std::string>();

        T_lidar_to_cam_ = load_extrinsic_matrix(extrinsic_path);
        std::tie(camera_matrix_, dist_coeffs_) = load_camera_calibration(intrinsics_path);

        // Topics
        std::string image_topic = config["camera"]["image_topic"].as<std::string>();
        std::string lidar_topic = config["lidar"]["lidar_topic"].as<std::string>();
        std::string projected_topic = config["camera"]["projected_topic"].as<std::string>();

        image_sub_.subscribe(this, image_topic);
        cloud_sub_.subscribe(this, lidar_topic);

        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, cloud_sub_);
        sync_->registerCallback(std::bind(&LidarCameraProjectionNode::callback, this, _1, _2));

        pub_image_ = create_publisher<sensor_msgs::msg::Image>(projected_topic, 1);
        bridge_ = std::make_shared<cv_bridge::CvBridge>();

        RCLCPP_INFO(this->get_logger(), "Projection node started.");
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = sync_policies::ApproximateTime<ImageMsg, CloudMsg>;

    Eigen::Matrix4d load_extrinsic_matrix(const std::string& path) {
        YAML::Node file = YAML::LoadFile(path);
        auto flat = file["extrinsic_matrix"].as<std::vector<double>>();
        if (flat.size() != 16)
            throw std::runtime_error("Extrinsic matrix must be 4x4.");

        Eigen::Matrix4d T;
        for (int i = 0; i < 16; ++i)
            T(i / 4, i % 4) = flat[i];
        return T;
    }

    std::pair<cv::Mat, cv::Mat> load_camera_calibration(const std::string& path) {
        YAML::Node calib = YAML::LoadFile(path);
        auto cam_data = calib["camera_matrix"]["data"].as<std::vector<double>>();
        auto dist_data = calib["distortion_coefficients"]["data"].as<std::vector<double>>();
        return {
            cv::Mat(cam_data).reshape(1, 3),
            cv::Mat(dist_data).reshape(1, 1)
        };
    }

    std::shared_ptr<cv_bridge::CvBridge> bridge_;
    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<CloudMsg> cloud_sub_;
    std::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    rclcpp::Publisher<ImageMsg>::SharedPtr pub_image_;

    Eigen::Matrix4d T_lidar_to_cam_;
    cv::Mat camera_matrix_, dist_coeffs_;

    void callback(const ImageMsg::ConstSharedPtr image_msg, const CloudMsg::ConstSharedPtr cloud_msg) {
        auto cv_img = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
        std::vector<cv::Point3d> cam_points;

        for (size_t i = 0; i + 11 < cloud_msg->data.size(); i += cloud_msg->point_step) {
            float x = *reinterpret_cast<const float*>(&cloud_msg->data[i + 0]);
            float y = *reinterpret_cast<const float*>(&cloud_msg->data[i + 4]);
            float z = *reinterpret_cast<const float*>(&cloud_msg->data[i + 8]);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            Eigen::Vector4d pt_lidar(x, y, z, 1.0);
            Eigen::Vector4d pt_cam = T_lidar_to_cam_ * pt_lidar;
            if (pt_cam.z() <= 0) continue;
            cam_points.emplace_back(pt_cam.x(), pt_cam.y(), pt_cam.z());
        }

        if (cam_points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No valid points to project.");
            pub_image_->publish(*image_msg);
            return;
        }

        std::vector<cv::Point2d> projected_pts;
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

        cv::projectPoints(cam_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_pts);

        for (const auto& pt : projected_pts) {
            int u = static_cast<int>(pt.x + 0.5);
            int v = static_cast<int>(pt.y + 0.5);
            if (u >= 0 && v >= 0 && u < cv_img.cols && v < cv_img.rows) {
                cv::circle(cv_img, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
            }
        }

        auto msg_out = cv_bridge::CvImage(image_msg->header, "bgr8", cv_img).toImageMsg();
        pub_image_->publish(*msg_out);
    }
};
