#include "modelec_gui/modelec_gui.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QMetaObject>
#include <utility>

ROS2QtGUI::ROS2QtGUI(rclcpp::Node::SharedPtr node, QWidget *parent)
    : QWidget(parent), node_(std::move(node)) {

    startButton_ = new QPushButton("Start");
    connect(startButton_, &QPushButton::clicked, this, [this]() {
        // Create a request for the speed service
        RCLCPP_INFO(node_->get_logger(), "Start button clicked.");
        auto request = std::make_shared<modelec_interface::srv::OdometryStart::Request>();
        request->start = true;
        client_start_->async_send_request(request, [this](rclcpp::Client<modelec_interface::srv::OdometryStart>::SharedFuture response) {
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

    askSpeed_ = new QPushButton("Ask speed");
    connect(askSpeed_, &QPushButton::clicked, this, [this]() {
        // Create a request for the speed service
        auto request = std::make_shared<modelec_interface::srv::OdometrySpeed::Request>();

        // Make the asynchronous service call
        client_->async_send_request(request, [this](rclcpp::Client<modelec_interface::srv::OdometrySpeed>::SharedFuture response) {
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

    mainLayout_ = new QVBoxLayout(this);
    mainLayout_->addWidget(startButton_);
    mainLayout_->addLayout(posLayout_);
    mainLayout_->addWidget(askSpeed_);
    mainLayout_->addLayout(speedLayout_);
    setLayout(mainLayout_);

    // Set up subscription
    sub_ = node_->create_subscription<modelec_interface::msg::OdometryPos>(
        "/odometry/position", 10,
        std::bind(&ROS2QtGUI::PositionCallback, this, std::placeholders::_1));

    // Set up service client
    client_ = node_->create_client<modelec_interface::srv::OdometrySpeed>("odometry/speed");

    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }

    client_start_ = node_->create_client<modelec_interface::srv::OdometryStart>("odometry/start");

    while (!client_start_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
    }
}

ROS2QtGUI::~ROS2QtGUI() = default;

void ROS2QtGUI::PositionCallback(const modelec_interface::msg::OdometryPos::SharedPtr msg)
{
    QMetaObject::invokeMethod(this, [this, msg]() {
        xBox_->setText(QString("x: %1").arg(msg->x));
        yBox_->setText(QString("y: %1").arg(msg->y));
        thetaBox_->setText(QString("theta: %1").arg(msg->theta));
    });
}
