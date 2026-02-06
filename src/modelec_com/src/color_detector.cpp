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
        headless_ = Config::get<bool>("config.cam.headless", true);

        if (!enable_)
        {
            RCLCPP_INFO(get_logger(), "Camera disabled by config");
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

        SetupRois();

        if (!headless_)
        {
            cv::namedWindow("color_detector", cv::WINDOW_NORMAL);
        }

        RCLCPP_INFO(get_logger(), "Color detector service ready");
    }

    ColorDetector::~ColorDetector()
    {
        if (!headless_)
        {
            cv::destroyAllWindows();
        }
    }

    bool ColorDetector::processSnapshot(std::vector<std::string>& colors, std::string& error)
    {
        cv::VideoCapture cap(link_);

        if (!cap.isOpened())
        {
            error = "Failed to open camera";
            return false;
        }

        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

        cv::Mat frame;

        for (int i = 0; i < 3; ++i)
            cap >> frame;

        if (frame.empty())
        {
            error = "Empty frame";
            return false;
        }

        cv::Mat hsv;
        cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0);
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        colors = classifyROIs(hsv, frame);

        if (!headless_)
        {
            cv::imshow("color_detector", frame);
            cv::waitKey(1);
        }

        if (save_to_file_)
        {
            std::string path = save_directory_ + generateImagePath();
            cv::imwrite(path, frame);
        }

        cap.release();

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
        std::vector<std::string> results;

        for (auto r : rois_)
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

    std::string ColorDetector::classify(const cv::Vec3d& hsv_roi) const
    {
        cv::Scalar yellow_low(20, 100, 100), yellow_high(40, 255, 255);
        cv::Scalar blue_low(100, 100, 100), blue_high(130, 255, 255);

        cv::Mat yellow_mask, blue_mask;
        cv::inRange(hsv_roi, yellow_low, yellow_high, yellow_mask);
        cv::inRange(hsv_roi, blue_low, blue_high, blue_mask);

        int yellow_count = cv::countNonZero(yellow_mask);
        int blue_count = cv::countNonZero(blue_mask);
        int total_pixels = hsv_roi.rows * hsv_roi.cols;

        double yellow_ratio = static_cast<double>(yellow_count) / total_pixels;
        double blue_ratio = static_cast<double>(blue_count) / total_pixels;

        if (yellow_ratio > blue_ratio && yellow_ratio > 0.3) return "yellow";
        if (blue_ratio > yellow_ratio && blue_ratio > 0.3) return "blue";

        return "unknown";
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

    void ColorDetector::SetupRois()
    {
        rois_.resize(4);

        rois_[0] = {
            Config::get<int>("config.cam.points.first@x", 0),
            Config::get<int>("config.cam.points.first@y", 0),
            Config::get<int>("config.cam.points.first@w", 0),
            Config::get<int>("config.cam.points.first@h", 0)
        };

        rois_[1] = {
            Config::get<int>("config.cam.points.second@x", 0),
            Config::get<int>("config.cam.points.second@y", 0),
            Config::get<int>("config.cam.points.second@w", 0),
            Config::get<int>("config.cam.points.second@h", 0)
        };

        rois_[2] = {
            Config::get<int>("config.cam.points.third@x", 0),
            Config::get<int>("config.cam.points.third@y", 0),
            Config::get<int>("config.cam.points.third@w", 0),
            Config::get<int>("config.cam.points.third@h", 0)
        };

        rois_[3] = {
            Config::get<int>("config.cam.points.fourth@x", 0),
            Config::get<int>("config.cam.points.fourth@y", 0),
            Config::get<int>("config.cam.points.fourth@w", 0),
            Config::get<int>("config.cam.points.fourth@h", 0)
        };
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
