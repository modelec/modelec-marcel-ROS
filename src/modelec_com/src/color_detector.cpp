#include <modelec_com/color_detector.hpp>

#include "modelec_utils/utils.hpp"

namespace Modelec
{
    ColorDetector::ColorDetector()
    : Node("color_detector")
    {
        cap_.open("http://192.168.1.21:8080/video");
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "Camera not detected");
            rclcpp::shutdown();
            return;
        }

        service_ = create_service<std_srvs::srv::Trigger>(
            "action/detect_color",
            std::bind(&ColorDetector::onRequest, this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();

        ask_sub_ = create_subscription<std_msgs::msg::Empty>(
            "action/detect_color/ask", qos,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                auto res = std_msgs::msg::String();

                if (!cap_.isOpened())
                {
                    res.data = "0;Camera not opened";
                    return;
                }

                cv::Mat frame;
                cap_ >> frame;

                if (frame.empty()) {
                    res.data = "0;Empty frame";
                    return;
                }

                if (save_to_file_)
                {
                    std::string path = generateImagePath();
                    cv::imwrite(path, frame);
                    RCLCPP_INFO(get_logger(), "Saved snapshot to %s", path.c_str());
                }

                cv::Mat hsv;
                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

                auto colors = classifyROIs(hsv);

                res.data = "1;" + join(colors, ";");

                color_pub_->publish(res);
            });

        color_pub_ = create_publisher<std_msgs::msg::String>("action/detect_color/res", qos);

        RCLCPP_INFO(get_logger(), "Color detector service ready");
    }

    void ColorDetector::onRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            response->success = false;
            response->message = "Empty frame";
            return;
        }

        if (save_to_file_)
        {
            std::string path = generateImagePath();
            cv::imwrite(path, frame);
            RCLCPP_INFO(get_logger(), "Saved snapshot to %s", path.c_str());
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        auto colors = classifyROIs(hsv);

        response->success = true;
        response->message = join(colors, ";");
    }

    // 4 independent ROIs
    std::vector<std::string> ColorDetector::classifyROIs(const cv::Mat& hsv) const
    {
        std::vector<cv::Rect> rois = {
            { 98,  98, 5, 5},
            {198,  98, 5, 5},
            { 98, 198, 5, 5},
            {198, 198, 5, 5}
        };

        std::vector<std::string> results;
        for (const auto& roi : rois) {
            cv::Rect safe_roi = roi & cv::Rect(0, 0, hsv.cols, hsv.rows);
            cv::Scalar mean = cv::mean(hsv(safe_roi));
            results.push_back(classify(cv::Vec3d(mean[0], mean[1], mean[2])));
        }
        return results;
    }

    std::string ColorDetector::classify(const cv::Vec3d& hsv) const
    {
        int h = static_cast<int>(hsv[0]);
        int s = static_cast<int>(hsv[1]);
        int v = static_cast<int>(hsv[2]);

        RCLCPP_DEBUG(get_logger(), "Classifying color with HSV: (%d, %d, %d)", h, s, v);

        if (s < 50 || v < 50)
            return "unknown";

        if (h >= 60)
            return "blue";
        return "yellow";
    }

    std::string ColorDetector::generateImagePath() const
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "snapshot_"
           << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
           << ".png"; // or .jpg
        return ss.str();
    }

}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::ColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
