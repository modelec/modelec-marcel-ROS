#include "modelec_gui/modelec_gui.hpp"

ROS2QtGUI::ROS2QtGUI(QWidget *parent)
    : QWidget(parent), node_(rclcpp::Node::make_shared("qt_gui_node")) {

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
        auto request = std::make_shared<modelec_interface::srv::OdometrySpeed::Request>();

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
            return;
        }

        if (auto response = future.get()) {
            xSpeedBox_->setText(QString("x speed: %1").arg(response->x));
            ySpeedBox_->setText(QString("y speed: %1").arg(response->y));
            thetaSpeedBox_->setText(QString("theta speed: %1").arg(response->theta));
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service");
        }
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
    mainLayout_->addLayout(posLayout_);
    mainLayout_->addWidget(askSpeed_);
    mainLayout_->addLayout(speedLayout_);
    setLayout(mainLayout_);

    sub_ = node_->create_subscription<modelec_interface::msg::OdometryPos>(
        "/odometry/position", 10,
        std::bind(&ROS2QtGUI::PositionCallback, this, std::placeholders::_1));

    client_ = node_->create_client<modelec_interface::srv::OdometrySpeed>("odometry/speed");

    // ensure the server is up
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
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
