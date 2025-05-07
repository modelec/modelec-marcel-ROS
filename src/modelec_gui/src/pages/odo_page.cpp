#include <modelec_gui/pages/odo_page.hpp>
#include <utility>
#include <boost/asio/connect.hpp>

namespace ModelecGUI
{

    OdoPage::OdoPage(rclcpp::Node::SharedPtr node, QWidget* parent) : QWidget(parent), node_(node)
    {
        startButton_ = new QPushButton("Start");
        connect(startButton_, &QPushButton::clicked, this, [this]() {
            // Create a request for the speed service
            RCLCPP_INFO(node_->get_logger(), "Start button clicked.");
            auto request = std::make_shared<modelec_interfaces::srv::OdometryStart::Request>();
            request->start = true;
            client_start_->async_send_request(request, [this](rclcpp::Client<modelec_interfaces::srv::OdometryStart>::SharedFuture response) {
                if (response.get()->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Start command sent successfully.");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to send start command.");
                }
            });
        });

        // Initialize the UI components
        xBox_ = new QLineEdit("x: ?");
        yBox_ = new QLineEdit("y: ?");
        thetaBox_ = new QLineEdit("theta: ?");
        xBox_->setReadOnly(true);
        yBox_->setReadOnly(true);
        thetaBox_->setReadOnly(true);

        posLayout_ = new QHBoxLayout;
        posLayout_->addWidget(xBox_);
        posLayout_->addWidget(yBox_);
        posLayout_->addWidget(thetaBox_);


        askPID_ = new QPushButton("Ask PID");
        connect(askPID_, &QPushButton::clicked, this, [this]() {
            RCLCPP_INFO(node_->get_logger(), "Ask PID button clicked.");
            // Create a request for the PID service
            auto request = std::make_shared<modelec_interfaces::srv::OdometryGetPid::Request>();

            // Make the asynchronous service call
            client_get_pid_->async_send_request(request, [this](rclcpp::Client<modelec_interfaces::srv::OdometryGetPid>::SharedFuture response) {
                RCLCPP_INFO(node_->get_logger(), "Received PID response.");
                if (auto res = response.get()) {
                    RCLCPP_INFO(node_->get_logger(), "PID values received: p=%f, i=%f, d=%f", res->p, res->i, res->d);
                    QMetaObject::invokeMethod(this, [this, res]() {
                        pPIDBox_->setText(QString("%1").arg(res->p));
                        iPIDBox_->setText(QString("%1").arg(res->i));
                        dPIDBox_->setText(QString("%1").arg(res->d));
                    });
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to get response for PID request.");
                }
            });
        });
        pPIDBox_ = new QLineEdit("");
        iPIDBox_ = new QLineEdit("");
        dPIDBox_ = new QLineEdit("");

        pidLayout_ = new QHBoxLayout;
        pidLayout_->addWidget(pPIDBox_);
        pidLayout_->addWidget(iPIDBox_);
        pidLayout_->addWidget(dPIDBox_);

        setPID_ = new QPushButton("Set PID");
        connect(setPID_, &QPushButton::clicked, this, [this]() {
            // Create a request for the PID service
            auto request = std::make_shared<modelec_interfaces::srv::OdometrySetPid::Request>();
            request->p = pPIDBox_->text().toFloat();
            request->i = iPIDBox_->text().toFloat();
            request->d = dPIDBox_->text().toFloat();

            // Make the asynchronous service call
            client_set_pid_->async_send_request(request, [this](rclcpp::Client<modelec_interfaces::srv::OdometrySetPid>::SharedFuture response) {
                if (response.get()->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Set PID command sent successfully.");
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to send set PID command.");
                }
            });
        });

        askSpeed_ = new QPushButton("Ask speed");
        connect(askSpeed_, &QPushButton::clicked, this, [this]() {
            // Create a request for the speed service
            auto request = std::make_shared<modelec_interfaces::srv::OdometrySpeed::Request>();

            // Make the asynchronous service call
            client_->async_send_request(request, [this](rclcpp::Client<modelec_interfaces::srv::OdometrySpeed>::SharedFuture response) {
                if (auto res = response.get()) {
                    QMetaObject::invokeMethod(this, [this, res]() {
                        xSpeedBox_->setText(QString("x: %1").arg(res->x));
                        ySpeedBox_->setText(QString("y: %1").arg(res->y));
                        thetaSpeedBox_->setText(QString("theta: %1").arg(res->theta));
                    });
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to get response for speed request.");
                }
            });
        });

        xSpeedBox_ = new QLineEdit("x speed: ?");
        ySpeedBox_ = new QLineEdit("y speed: ?");
        thetaSpeedBox_ = new QLineEdit("theta speed: ?");
        xSpeedBox_->setReadOnly(true);
        ySpeedBox_->setReadOnly(true);
        thetaSpeedBox_->setReadOnly(true);

        speedLayout_ = new QHBoxLayout;
        speedLayout_->addWidget(xSpeedBox_);
        speedLayout_->addWidget(ySpeedBox_);
        speedLayout_->addWidget(thetaSpeedBox_);

        startTest_ = new QPushButton("Start Test");
        connect(startTest_, &QPushButton::clicked, this, [this]() {
            auto firstRequest = std::make_shared<modelec_interfaces::msg::OdometryAddWaypoint>();
            firstRequest->id = 0;
            firstRequest->is_end = false;
            firstRequest->x = 100.0;
            firstRequest->y = 0.0;
            firstRequest->theta = 0.0;

            pub_add_waypoint_->publish(*firstRequest);

            auto secondRequest = std::make_shared<modelec_interfaces::msg::OdometryAddWaypoint>();
            secondRequest->id = 1;
            secondRequest->is_end = true;
            secondRequest->x = 0.0;
            secondRequest->y = 0.0;
            secondRequest->theta = 0.0;

            pub_add_waypoint_->publish(*secondRequest);
        });

        mainLayout_ = new QVBoxLayout(this);
        mainLayout_->addWidget(startButton_);
        mainLayout_->addLayout(posLayout_);
        mainLayout_->addWidget(askPID_);
        mainLayout_->addLayout(pidLayout_);
        mainLayout_->addWidget(setPID_);
        mainLayout_->addWidget(askSpeed_);
        mainLayout_->addLayout(speedLayout_);
        mainLayout_->addWidget(startTest_);
        setLayout(mainLayout_);

        // Set up subscription
        sub_ = node_->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "/odometry/position", 10,
            std::bind(&OdoPage::PositionCallback, this, std::placeholders::_1));

        pub_add_waypoint_ = node_->create_publisher<modelec_interfaces::msg::OdometryAddWaypoint>(
            "/odometry/add_waypoint", 10);

        client_ = node_->create_client<modelec_interfaces::srv::OdometrySpeed>("odometry/speed");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        client_start_ = node_->create_client<modelec_interfaces::srv::OdometryStart>("odometry/start");
        while (!client_start_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        client_get_pid_ = node_->create_client<modelec_interfaces::srv::OdometryGetPid>("odometry/get_pid");

        while (!client_get_pid_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }

        client_set_pid_ = node_->create_client<modelec_interfaces::srv::OdometrySetPid>("odometry/set_pid");
        while (!client_set_pid_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
        }
    }

    OdoPage::~OdoPage() = default;

    void OdoPage::PositionCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        QMetaObject::invokeMethod(this, [this, msg]() {
            xBox_->setText(QString("x: %1").arg(msg->x));
            yBox_->setText(QString("y: %1").arg(msg->y));
            thetaBox_->setText(QString("theta: %1").arg(msg->theta));
        });
    }
}
