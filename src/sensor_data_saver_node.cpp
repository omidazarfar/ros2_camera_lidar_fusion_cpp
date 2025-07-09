#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
using namespace message_filters;

class SaveDataNode : public rclcpp::Node {
public:
    SaveDataNode() : Node("save_data_node") {
        RCLCPP_INFO(this->get_logger(), "Save data node started.");

        std::string config_path = this->declare_parameter<std::string>("config_file", "config/general.yaml");
        YAML::Node config = YAML::LoadFile(config_path);

        max_file_saved_ = config["general"]["max_file_saved"].as<int>();
        storage_path_ = config["general"]["data_folder"].as<std::string>();
        image_topic_ = config["camera"]["image_topic"].as<std::string>();
        lidar_topic_ = config["lidar"]["lidar_topic"].as<std::string>();
        keyboard_listener_enabled_ = config["general"]["keyboard_listener"].as<bool>();
        slop_ = config["general"]["slop"].as<double>();

        if (!fs::exists(storage_path_)) {
            fs::create_directories(storage_path_);
        }

        RCLCPP_WARN(this->get_logger(), "Data will be saved to: %s", storage_path_.c_str());

        image_sub_.subscribe(this, image_topic_);
        cloud_sub_.subscribe(this, lidar_topic_);

        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, cloud_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_));
        sync_->registerCallback(std::bind(&SaveDataNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        save_data_flag_ = !keyboard_listener_enabled_;
        if (keyboard_listener_enabled_) {
            std::thread(&SaveDataNode::keyboard_listener, this).detach();
        }
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using CloudMsg = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = sync_policies::ApproximateTime<ImageMsg, CloudMsg>;

    void keyboard_listener() {
        std::string input;
        while (rclcpp::ok()) {
            std::cout << "Press Enter to save data: ";
            std::getline(std::cin, input);
            if (input.empty()) {
                save_data_flag_ = true;
                RCLCPP_INFO(this->get_logger(), "User pressed Enter, will save next synced data.");
            }
        }
    }

    void callback(const ImageMsg::ConstSharedPtr image_msg, const CloudMsg::ConstSharedPtr cloud_msg) {
        if (!save_data_flag_) return;

        std::string timestamp = get_timestamp();
        if (std::distance(fs::directory_iterator(storage_path_), fs::directory_iterator{}) >= max_file_saved_) {
            RCLCPP_WARN(this->get_logger(), "Max number of saved files reached.");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
        cv::imwrite(storage_path_ + "/" + timestamp + ".png", cv_ptr->image);

        auto pcd = pointcloud2_to_open3d(cloud_msg);
        open3d::io::WritePointCloud(storage_path_ + "/" + timestamp + ".pcd", *pcd);

        RCLCPP_INFO(this->get_logger(), "Saved %s.[png, pcd]", timestamp.c_str());

        if (keyboard_listener_enabled_) {
            save_data_flag_ = false;
        }
    }

    std::shared_ptr<open3d::geometry::PointCloud> pointcloud2_to_open3d(const CloudMsg::ConstSharedPtr& cloud_msg) {
        std::shared_ptr<open3d::geometry::PointCloud> cloud(new open3d::geometry::PointCloud);
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
                cloud->points_.emplace_back(*iter_x, *iter_y, *iter_z);
            }
        }

        return cloud;
    }

    std::string get_timestamp() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        char buf[64];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now_time));
        return std::string(buf);
    }

    std::string image_topic_, lidar_topic_, storage_path_;
    int max_file_saved_;
    bool keyboard_listener_enabled_;
    double slop_;
    bool save_data_flag_;

    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<CloudMsg> cloud_sub_;
    std::shared_ptr<Synchronizer<SyncPolicy>> sync_;
};
