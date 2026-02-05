#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace Modelec
{
    class ColorDetector : public rclcpp::Node
    {
    public:
        ColorDetector();
    private:
        void onRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        std::vector<std::string> classifyROIs(const cv::Mat& hsv) const;

        std::string classify(const cv::Vec3d& hsv) const;

        std::string generateImagePath() const;

        cv::VideoCapture cap_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ask_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub_;

        std::string link_;
        bool save_to_file_ = true;
        std::string save_directory_ = "./";
        bool enable_ = false;
    };
}
