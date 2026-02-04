#include <modelec_gui/pages/action_page.hpp>

namespace ModelecGUI
{
    ActionPage::ActionPage(rclcpp::Node::SharedPtr node, QWidget* parent) :
        QWidget(parent),
        node_(node)
    {
        action_exec_pub_ = node_->create_publisher<ActionExec>(
            "action/exec", 10);

        servo_get_sub_ = node_->create_subscription<ActionServoPosArray>(
            "action/get/servo", 10, [this](const ActionServoPosArray::SharedPtr)
            {
            });

        servo_move_pub_ = node_->create_publisher<ActionServoPosArray>(
            "action/move/servo", 10);

        servo_move_res_sub_ = node_->create_subscription<ActionServoPosArray>(
            "action/move/servo/res", 10, [this](const ActionServoPosArray::SharedPtr)
            {
            });

        servo_timed_move_pub_ = node_->create_publisher<ActionServoTimedArray>(
            "action/move/servo/timed", 10);

        servo_timed_move_res_sub_ = node_->create_subscription<ActionServoTimedArray>(
            "action/move/servo/timed/res", 10, [this](const ActionServoTimedArray::SharedPtr)
            {
            });

        main_layout_ = new QVBoxLayout(this);
        setLayout(main_layout_);
        setWindowTitle("Action Page");

        deploy_max_button_ = new QPushButton("Deploy Max Size");
        connect(deploy_max_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::MAX_DEPLOY;
                action_exec_pub_->publish(action_exec);
            });

        deploy_action1_layout_ = new QHBoxLayout;
        deploy_action1_front_layout_ = new QVBoxLayout;
        deploy_action1_front_label_ = new QLabel("Front Action 1");

        deploy_action1_front_up_button_ = new QPushButton("Up");
        deploy_action1_front_down_button_ = new QPushButton("Down");
        deploy_action1_front_rotate_button_ = new QPushButton("DownRota");

        connect(deploy_action1_front_up_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::UP + ActionExec::DELIMITER + "1";
                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_front_down_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DOWN + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "0";
                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_front_rotate_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DOWN + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "1";
                action_exec_pub_->publish(action_exec);
            });

        deploy_action1_front_servos_layout_ = new QHBoxLayout;

        deploy_action1_front_servo1_button_ = new QPushButton("Servo 1");
        deploy_action1_front_servo2_button_ = new QPushButton("Servo 2");
        deploy_action1_front_servo3_button_ = new QPushButton("Servo 3");
        deploy_action1_front_servo4_button_ = new QPushButton("Servo 4");

        connect(deploy_action1_front_servo1_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_front_servo1_state_ = !deploy_action1_front_servo1_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_front_servo1_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "1";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_front_servo2_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_front_servo2_state_ = !deploy_action1_front_servo2_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_front_servo2_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "2";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_front_servo3_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_front_servo3_state_ = !deploy_action1_front_servo3_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_front_servo3_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "3";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_front_servo4_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_front_servo4_state_ = !deploy_action1_front_servo4_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_front_servo4_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "1" + ActionExec::DELIMITER + "4";

                action_exec_pub_->publish(action_exec);
            });

        deploy_action1_front_servos_layout_->addWidget(deploy_action1_front_servo1_button_);
        deploy_action1_front_servos_layout_->addWidget(deploy_action1_front_servo2_button_);
        deploy_action1_front_servos_layout_->addWidget(deploy_action1_front_servo3_button_);
        deploy_action1_front_servos_layout_->addWidget(deploy_action1_front_servo4_button_);

        deploy_action1_front_layout_->addWidget(deploy_action1_front_label_);
        deploy_action1_front_layout_->addWidget(deploy_action1_front_up_button_);
        deploy_action1_front_layout_->addLayout(deploy_action1_front_servos_layout_);
        deploy_action1_front_layout_->addWidget(deploy_action1_front_down_button_);

        deploy_action1_back_layout_ = new QVBoxLayout;
        deploy_action1_back_label_ = new QLabel("Back Action 1");

        deploy_action1_back_up_button_ = new QPushButton("Up");
        deploy_action1_back_down_button_ = new QPushButton("Down");
        deploy_action1_back_rotate_button_ = new QPushButton("DownRota");

        connect(deploy_action1_back_up_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::UP + ActionExec::DELIMITER + "0";
                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_back_down_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DOWN + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "0";
                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_back_rotate_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DOWN + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "1";
                action_exec_pub_->publish(action_exec);
            });

        deploy_action1_back_servos_layout_ = new QHBoxLayout;

        deploy_action1_back_servo1_button_ = new QPushButton("Servo 1");
        deploy_action1_back_servo2_button_ = new QPushButton("Servo 2");
        deploy_action1_back_servo3_button_ = new QPushButton("Servo 3");
        deploy_action1_back_servo4_button_ = new QPushButton("Servo 4");

        connect(deploy_action1_back_servo1_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_back_servo1_state_ = !deploy_action1_back_servo1_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_back_servo1_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "1";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_back_servo2_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_back_servo2_state_ = !deploy_action1_back_servo2_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_back_servo2_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "2";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_back_servo3_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_back_servo3_state_ = !deploy_action1_back_servo3_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_back_servo3_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "3";

                action_exec_pub_->publish(action_exec);
            });

        connect(deploy_action1_back_servo4_button_, &QPushButton::clicked, this,
            [this]()
            {
                deploy_action1_back_servo4_state_ = !deploy_action1_back_servo4_state_;

                ActionExec action_exec;

                action_exec.action = (deploy_action1_back_servo4_state_
                    ? ActionExec::TAKE : ActionExec::FREE) + ActionExec::DELIMITER + "0" + ActionExec::DELIMITER + "4";

                action_exec_pub_->publish(action_exec);
            });

        deploy_action1_back_servos_layout_->addWidget(deploy_action1_back_servo1_button_);
        deploy_action1_back_servos_layout_->addWidget(deploy_action1_back_servo2_button_);
        deploy_action1_back_servos_layout_->addWidget(deploy_action1_back_servo3_button_);
        deploy_action1_back_servos_layout_->addWidget(deploy_action1_back_servo4_button_);

        deploy_action1_back_layout_->addWidget(deploy_action1_back_label_);
        deploy_action1_back_layout_->addWidget(deploy_action1_back_up_button_);
        deploy_action1_back_layout_->addLayout(deploy_action1_back_servos_layout_);
        deploy_action1_back_layout_->addWidget(deploy_action1_back_down_button_);

        deploy_action1_layout_->addLayout(deploy_action1_front_layout_);
        deploy_action1_layout_->addLayout(deploy_action1_back_layout_);

        deploy_action2_layout_ = new QVBoxLayout;
        deploy_action2_label_ = new QLabel("Thermometrer");
        deploy_action2_button_ = new QPushButton("Deploy");
        undeploy_action2_button_ = new QPushButton("Undeploy");

        connect(deploy_action2_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::THERMO + ActionExec::DELIMITER + "-1" + ActionExec::DELIMITER + "1";
                action_exec_pub_->publish(action_exec);
            });

        connect(undeploy_action2_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::THERMO + ActionExec::DELIMITER + "-1" + ActionExec::DELIMITER + "0";
                action_exec_pub_->publish(action_exec);
            });

        deploy_action2_layout_->addWidget(deploy_action2_label_);
        deploy_action2_layout_->addWidget(deploy_action2_button_);
        deploy_action2_layout_->addWidget(undeploy_action2_button_);

        main_layout_->addWidget(deploy_max_button_);
        main_layout_->addLayout(deploy_action1_layout_);
        main_layout_->addLayout(deploy_action2_layout_);
    }

    ActionPage::~ActionPage()
    = default;
}
