#include <modelec_com/color_detector.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>
#include <modelec_utils/utils.hpp>

namespace Modelec
{
    ColorDetector::ColorDetector()
    : Node("color_detector")
    {
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        link_ = Config::get<std::string>("config.cam.link", "/dev/video0");
        save_to_file_ = Config::get<bool>("config.cam.save_to_file.enabled", false);
        save_directory_ = Config::get<std::string>("config.cam.save_to_file.path", "./");
        enable_ = Config::get<bool>("config.cam.enabled", false);

        if (!enable_)
        {
            RCLCPP_INFO(get_logger(), "Camera disabled by config");
            rclcpp::shutdown();
            return;
        }

        cap_.open(link_);
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
                std_msgs::msg::String res;
                std::vector<std::string> colors;
                std::string error;

                if (!processSnapshot(colors, error))
                {
                    res.data = "0;" + error;
                }
                else
                {
                    res.data = "1;" + join(colors, ";");
                }

                color_pub_->publish(res);
            });

        color_pub_ = create_publisher<std_msgs::msg::String>("action/detect_color/res", qos);

        RCLCPP_INFO(get_logger(), "Color detector service ready");
    }

    bool ColorDetector::processSnapshot(std::vector<std::string>& colors, std::string& error)
    {
        if (!cap_.isOpened())
        {
            error = "Camera not opened";
            return false;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            error = "Empty frame";
            return false;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        colors = classifyROIs(hsv, frame);

        if (save_to_file_)
        {
            std::string path = save_directory_ + generateImagePath();
            cv::imwrite(path, frame);
            RCLCPP_INFO(get_logger(), "Saved annotated snapshot: %s", path.c_str());
        }

        return true;
    }

    void ColorDetector::onRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::vector<std::string> colors;
        std::string error;

        if (!processSnapshot(colors, error))
        {
            response->success = false;
            response->message = error;
            return;
        }

        response->success = true;
        response->message = join(colors, ";");
    }

    // 4 independent ROIs
    std::vector<std::string> ColorDetector::classifyROIs(const cv::Mat& hsv, cv::Mat& debug_img) const
    {
        std::vector<cv::Rect> rois = {
            { 98,  98, 5, 5},
            {198,  98, 5, 5},
            { 98, 198, 5, 5},
            {198, 198, 5, 5}
        };

        std::vector<std::string> results;

        for (auto r : rois)
        {
            cv::Rect roi = r & cv::Rect(0, 0, hsv.cols, hsv.rows);
            cv::Scalar mean = cv::mean(hsv(roi));

            std::string color = classify(cv::Vec3d(mean[0], mean[1], mean[2]));
            results.push_back(color);

            if (save_to_file_)
            {
                cv::rectangle(debug_img, roi, {0, 255, 0}, 1);
                cv::putText(
                    debug_img,
                    color,
                    roi.tl() + cv::Point(0, -5),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.4,
                    {0, 255, 0},
                    1);
            }
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
           << ".png";
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
