#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace Modelec
{
    struct ColorSetting
    {
        std::string name;
        double h_min;
        double h_max;
    };

    class ColorDetector : public rclcpp::Node
    {
    public:
        ColorDetector();

        ~ColorDetector() override;
    private:
        bool processSnapshot(std::vector<std::string>& colors, std::string& error);

        void onRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        std::vector<std::string> classifyROIs(const cv::Mat& hsv, cv::Mat& debug_img) const;

        std::string classify(const cv::Vec3d& hsv_roi) const;

        std::string generateImagePath() const;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ask_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_pub_;

        std::vector<cv::Rect> rois_;

        std::string link_;
        bool save_to_file_ = true;
        std::string save_directory_ = "./";
        bool enable_ = false;
        bool headless_ = true;

        std::vector<ColorSetting> color_configs_;
    };
}
