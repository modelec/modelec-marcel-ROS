#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <modelec_utils/config.hpp>

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


    template<>
    inline std::vector<ColorSetting>
    Config::get<std::vector<ColorSetting>>(
        const std::string& prefix,
        const std::vector<ColorSetting>& default_value, bool)
    {
        auto result = Config::getArray<ColorSetting>(
            prefix,
            [](const std::string& base)
            {
                return ColorSetting{
                    Config::get<std::string>(base + "@name"),
                    Config::get<double>(base + "@hue_min"),
                    Config::get<double>(base + "@hue_max")
                };
            });

        return result.empty() ? default_value : result;
    }

    template<>
    inline std::vector<cv::Rect>
    Config::get<std::vector<cv::Rect>>(
        const std::string& prefix,
        const std::vector<cv::Rect>& default_value, bool)
    {
        auto result = Config::getArray<cv::Rect>(
            prefix,
            [](const std::string& base)
            {
                return cv::Rect(
                    Config::get<int>(base + "@x", 0),
                    Config::get<int>(base + "@y", 0),
                    Config::get<int>(base + "@w", 100),
                    Config::get<int>(base + "@h", 100)
                );
            });

        return result.empty() ? default_value : result;
    }
}
