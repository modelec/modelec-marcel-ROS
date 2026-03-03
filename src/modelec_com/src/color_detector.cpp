#include <modelec_com/color_detector.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/utils.hpp>

#include <libcamera/libcamera.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>

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

        enable_ = Config::get<bool>("config.cam.enabled", false);
        link_ = Config::get<std::string>("config.cam.link", "/dev/video0");
        headless_ = Config::get<bool>("config.cam.headless", true);
        save_to_file_ = Config::get<bool>("config.cam.save_to_file.enabled", false);
        save_directory_ = Config::get<std::string>("config.cam.save_to_file.directory", "./");

        rois_ = Config::get<std::vector<cv::Rect>>("config.cam.rois.roi", {});
        color_configs_ = Config::get<std::vector<ColorSetting>>("config.cam.colors.color", {});

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
        libcamera::CameraManager cm;
        cm.start();

        if (cm.cameras().empty()) {
            error = "No cameras detected";
            return false;
        }

        auto camera = cm.cameras()[0]; // shared_ptr<Camera>
        if (!camera->acquire()) {
            error = "Failed to acquire camera";
            return false;
        }

        // Camera configuration
        std::unique_ptr<libcamera::CameraConfiguration> config =
            camera->generateConfiguration({libcamera::StreamRole::VideoRecording});

        config->at(0).size.width = 2304;
        config->at(0).size.height = 1296;
        config->at(0).pixelFormat = libcamera::formats::RGB888;

        if (camera->configure(config.get()) < 0) {
            error = "Camera configuration failed";
            camera->release();
            return false;
        }

        libcamera::FrameBufferAllocator allocator(camera);
        if (allocator.allocate(config->at(0).stream()) < 0) {
            error = "Buffer allocation failed";
            camera->release();
            return false;
        }

        // Create request
        auto request = camera->createRequest();
        if (!request) {
            error = "Request creation failed";
            camera->release();
            return false;
        }

        request->addBuffer(config->at(0).stream(), allocator.buffers(config->at(0).stream())[0].get());

        // Start camera
        if (camera->start() < 0) {
            error = "Camera start failed";
            camera->release();
            return false;
        }

        if (camera->queueRequest(request.get()) < 0) {
            error = "Queue request failed";
            camera->stop();
            camera->release();
            return false;
        }

        // Wait for request completion
        while (request->status() != libcamera::Request::RequestComplete) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        const libcamera::FrameBuffer &fb = *request->buffers().begin()->second;
        cv::Mat frame(config->at(0).size.height, config->at(0).size.width, CV_8UC3, fb.planes()[0].data());

        if (frame.empty()) {
            error = "Empty frame";
            camera->stop();
            camera->release();
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
            RCLCPP_DEBUG(get_logger(), "Saved snapshot to %s", path.c_str());
        }

        camera->stop();
        camera->release();
        cm.stop();

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
