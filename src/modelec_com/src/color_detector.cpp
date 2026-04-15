#include <modelec_com/color_detector.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/utils.hpp>
#include <mutex>

#ifdef RPI_BUILD
#include <libcam2opencv.h>
#endif

namespace Modelec
{
#ifdef RPI_BUILD
    struct CamCallback : Libcam2OpenCV::Callback {
        cv::Mat* target_frame;
        std::mutex* frame_mutex;
        CamCallback(cv::Mat& frame, std::mutex& mtx) : target_frame(&frame), frame_mutex(&mtx) {}
        void hasFrame(const cv::Mat &frame, const libcamera::ControlList &) override {
            std::lock_guard<std::mutex> lock(*frame_mutex);
            frame.copyTo(*target_frame);
        }
    };
#endif
    ColorDetector::ColorDetector()
    : Node("color_detector")
    {
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
         if (!Config::load(config_path))
         {
             RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
         }

        enable_ = Config::get<bool>("config.cam.enabled", false);
        link_ = Config::get<std::string>("config.cam.link", "/dev/video0");
        headless_ = Config::get<bool>("config.cam.headless", true);
        save_to_file_ = Config::get<bool>("config.cam.save_to_file.enabled", false);
        save_directory_ = Config::get<std::string>("config.cam.save_to_file.directory", "./");

        rois_ = Config::get<bool>("config.strat.two-sided")
            ? Config::get<std::vector<cv::Rect>>("config.cam.rois.two-sided-roi", {})
            : Config::get<std::vector<cv::Rect>>("config.cam.rois.one-sided.roi", {});

        color_configs_ = Config::get<std::vector<ColorSetting>>("config.cam.colors.color", {});

        if (!enable_)
        {
            RCLCPP_INFO(get_logger(), "Camera disabled by config");
            rclcpp::shutdown();
            return;
        }

#ifdef RPI_BUILD
        RCLCPP_INFO(get_logger(), "Starting RPi Libcamera (Full FOV mode)...");
        Libcam2OpenCVSettings settings;
        settings.width = 2304;
        settings.height = 1296;
        settings.framerate = 30;
        settings.cameraIndex = 0;

        my_callback_ = std::make_unique<CamCallback>(latest_frame_, frame_mutex_);
        camera_.registerCallback(my_callback_.get());
        camera_.start(settings);
#else
        RCLCPP_INFO(get_logger(), "Starting PC Webcam (OpenCV)...");
        pc_cap_.open(link_);
#endif

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
                    res.data = "0|" + error;
                }
                else
                {
                    res.data = "1|" + join(colors, ";");
                }

                color_pub_->publish(res);
            });

        color_pub_ = create_publisher<std_msgs::msg::String>("action/detect_color/res", qos);

        if (!headless_)
        {
            cv::namedWindow("color_detector", cv::WINDOW_NORMAL);

            cv::setWindowProperty("color_detector", cv::WND_PROP_AUTOSIZE, cv::WINDOW_GUI_EXPANDED);
        }

        RCLCPP_INFO(get_logger(), "Color detector service ready");
    }

    ColorDetector::~ColorDetector()
    {
#ifdef RPI_BUILD
        camera_.stop();
#else
        if(pc_cap_.isOpened()) pc_cap_.release();
#endif

        if (!headless_)
        {
            cv::destroyAllWindows();
        }
    }

    bool ColorDetector::processSnapshot(std::vector<std::string>& colors, std::string& error)
    {
        cv::Mat frame;
#ifdef RPI_BUILD
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (latest_frame_.empty()) {
                RCLCPP_WARN(get_logger(), "No frame received from libcamera yet");
                error = "Empty frame";
                return false;
            }
            latest_frame_.copyTo(frame);
        }
#else
if(!pc_cap_.isOpened()) { error = "PC Cam not open"; return false; }
pc_cap_ >> frame;
#endif

        if (frame.empty())
        {
            RCLCPP_WARN(get_logger(), "Captured empty frame from camera");
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
            RCLCPP_INFO(get_logger(), "Saved snapshot to %s", path.c_str());
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

    std::vector<std::string> ColorDetector::classifyROIs(const cv::Mat& hsv, cv::Mat& debug_img) const
    {
        std::vector<std::string> results;

        for (auto r : rois_)
        {
            cv::Rect roi = r & cv::Rect(0, 0, hsv.cols, hsv.rows);
            cv::Scalar mean = cv::mean(hsv(roi));

            std::string color = classify(cv::Vec3d(mean[0], mean[1], mean[2]));

            RCLCPP_DEBUG(get_logger(), "ROI at (%d, %d, %d, %d) has mean HSV (%.2f, %.2f, %.2f) classified as %s",
                         roi.x, roi.y, roi.width, roi.height,
                         mean[0], mean[1], mean[2],
                         color.c_str());

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
        double h = hsv_roi[0];

        for (const auto& color_config : color_configs_)
        {
            if (h >= color_config.h_min && h <= color_config.h_max)
            {
                return color_config.name;
            }
        }

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
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::ColorDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
