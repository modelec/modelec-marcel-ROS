#include <modelec_gui/pages/action_page.hpp>

namespace ModelecGUI
{
    ActionPage::ActionPage(rclcpp::Node::SharedPtr node, QWidget* parent) :
        QWidget(parent),
        node_(node)
    {
        layout_ = new QVBoxLayout(this);
        setLayout(layout_);
        setWindowTitle("Action Page");

        max_size_button_ = new QPushButton("Deploy Max Size");

        connect(max_size_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DEPLOY_MAX_SIZE;
                action_exec.mission = {ActionExec::ASC_GO_UP, ActionExec::RETRACT_BOTTOM_PLATE};
                action_exec_pub_->publish(action_exec);
            });

        asc_layout_ = new QHBoxLayout;

        asc_action_ = new ActionWidget(this);
        asc_action_->SetSpinBoxRange(0, 1000);
        asc_action_->SetSpinBoxStep(1);
        asc_action_->SetSpinBoxValue(0);
        asc_action_->SetButtonText("Send ASC");

        waiting_for_move_asc_ = false;

        connect(asc_action_, &ActionWidget::ButtonClicked, this,
            [this](double value)
            {
                ActionAscPos asc_move;
                asc_move.value = value;
                asc_move.pos = 128;
                asc_set_pub_->publish(asc_move);
                asc_action_->setDisabled(true);

                waiting_for_move_asc_ = true;
            });

        asc_layout_->addWidget(asc_action_);

        servo_layout_ = new QGridLayout;

        servo0_action_ = new ActionWidget(this);
        servo0_action_->SetSpinBoxRange(0, 180);
        servo0_action_->SetSpinBoxStep(.1);
        servo0_action_->SetSpinBoxValue(90);
        servo0_action_->SetButtonText("Servo 0");

        servo1_action_ = new ActionWidget(this);
        servo1_action_->SetSpinBoxRange(0, 180);
        servo1_action_->SetSpinBoxStep(.1);
        servo1_action_->SetSpinBoxValue(90);
        servo1_action_->SetButtonText("Servo 1");

        servo2_action_ = new ActionWidget(this);
        servo2_action_->SetSpinBoxRange(0, 180);
        servo2_action_->SetSpinBoxStep(.1);
        servo2_action_->SetSpinBoxValue(90);
        servo2_action_->SetButtonText("Servo 2");

        servo3_action_ = new ActionWidget(this);
        servo3_action_->SetSpinBoxRange(0, 180);
        servo3_action_->SetSpinBoxStep(.1);
        servo3_action_->SetSpinBoxValue(90);
        servo3_action_->SetButtonText("Servo 3");

        servo4_action_ = new ActionWidget(this);
        servo4_action_->SetSpinBoxRange(0, 180);
        servo4_action_->SetSpinBoxStep(.1);
        servo4_action_->SetSpinBoxValue(90);
        servo4_action_->SetButtonText("Servo 4");

        servo5_action_ = new ActionWidget(this);
        servo5_action_->SetSpinBoxRange(0, 180);
        servo5_action_->SetSpinBoxStep(.1);
        servo5_action_->SetSpinBoxValue(90);
        servo5_action_->SetButtonText("Servo 5");

        servo6_action_ = new ActionWidget(this);
        servo6_action_->SetSpinBoxRange(0, 180);
        servo6_action_->SetSpinBoxStep(.1);
        servo6_action_->SetSpinBoxValue(90);
        servo6_action_->SetButtonText("Servo 6");

        servo7_action_ = new ActionWidget(this);
        servo7_action_->SetSpinBoxRange(0, 180);
        servo7_action_->SetSpinBoxStep(.1);
        servo7_action_->SetSpinBoxValue(90);
        servo7_action_->SetButtonText("Servo 7");

        servo8_action_ = new ActionWidget(this);
        servo8_action_->SetSpinBoxRange(0, 180);
        servo8_action_->SetSpinBoxStep(.1);
        servo8_action_->SetSpinBoxValue(90);
        servo8_action_->SetButtonText("Servo 8");

        servo_layout_->addWidget(servo0_action_, 0, 0);
        servo_layout_->addWidget(servo1_action_, 0, 1);
        servo_layout_->addWidget(servo2_action_, 0, 2);
        servo_layout_->addWidget(servo3_action_, 1, 0);
        servo_layout_->addWidget(servo4_action_, 1, 1);
        servo_layout_->addWidget(servo5_action_, 1, 2);
        servo_layout_->addWidget(servo6_action_, 2, 0);
        servo_layout_->addWidget(servo7_action_, 2, 1);
        servo_layout_->addWidget(servo8_action_, 2, 2);

        servo_actions_.push_back(servo0_action_);
        servo_actions_.push_back(servo1_action_);
        servo_actions_.push_back(servo2_action_);
        servo_actions_.push_back(servo3_action_);
        servo_actions_.push_back(servo4_action_);
        servo_actions_.push_back(servo5_action_);
        servo_actions_.push_back(servo6_action_);
        servo_actions_.push_back(servo7_action_);
        servo_actions_.push_back(servo8_action_);
        waiting_for_move_servo_ = std::vector<bool>(servo_actions_.size(), false);

        for (size_t i = 0; i < servo_actions_.size(); ++i)
        {
            connect(servo_actions_[i], &ActionWidget::ButtonClicked, this,
                [this, i](double value)
                {
                    ActionServoPos servo_move;
                    servo_move.id = i;
                    servo_move.pos = 128;
                    servo_move.angle = value * M_PI / 180.0;
                    servo_set_pub_->publish(servo_move);
                    servo_actions_[i]->setDisabled(true);

                    waiting_for_move_servo_[i] = true;
                });
        }

        relay_layout_ = new QHBoxLayout;

        relay_top_button_ = new QPushButton(this);
        relay_top_button_->setText("Toggle Relay Top");

        relay_bottom_button_ = new QPushButton(this);
        relay_bottom_button_->setText("Toggle Relay Bottom");

        relay_third_button_ = new QPushButton(this);
        relay_third_button_->setText("Toggle Third Relay");

        relay_layout_->addWidget(relay_top_button_);
        relay_layout_->addWidget(relay_bottom_button_);
        relay_layout_->addWidget(relay_third_button_);

        relay_buttons_.push_back(relay_top_button_);
        relay_buttons_.push_back(relay_bottom_button_);
        relay_buttons_.push_back(relay_third_button_);
        relay_values_ = {false, false, false};

        for (size_t i = 0; i < relay_buttons_.size(); ++i)
        {
            connect(relay_buttons_[i], &QPushButton::clicked, this,
                [this, i]()
                {
                    ActionRelayState relay_state;
                    relay_state.id = i;
                    relay_state.state = !relay_values_[i];
                    relay_move_pub_->publish(relay_state);
                    relay_buttons_[i]->setDisabled(true);
                });
        }

        deploy_banner_button_ = new QPushButton("Deploy Banner");
        connect(deploy_banner_button_, &QPushButton::clicked, this,
            [this]()
            {
                ActionExec action_exec;
                action_exec.action = ActionExec::DEPLOY_BANNER;
                action_exec.mission = {ActionExec::DEPLOY_BANNER_STEP};
                action_exec_pub_->publish(action_exec);
            });

        layout_->addWidget(max_size_button_);
        layout_->addLayout(asc_layout_);
        layout_->addLayout(servo_layout_);
        layout_->addLayout(relay_layout_);
        layout_->addWidget(deploy_banner_button_);

        action_exec_pub_ = node_->create_publisher<ActionExec>(
            "action/exec", 10);

        asc_get_sub_ = node_->create_subscription<ActionAscPos>(
            "action/get/asc", 10, [this](const ActionAscPos::SharedPtr msg)
            {
                asc_action_->SetSpinBoxValue(msg->value);
            });

        asc_set_pub_ = node_->create_publisher<ActionAscPos>(
            "action/set/asc", 10);

        asc_set_res_sub_ = node_->create_subscription<ActionAscPos>(
            "action/set/asc/res", 10, [this](const ActionAscPos::SharedPtr msg)
            {
                if (waiting_for_move_asc_)
                {
                    ActionAscPos asc_move;
                    asc_move.pos = msg->pos;
                    asc_move_pub_->publish(asc_move);
                    waiting_for_move_asc_ = false;
                }
            });

        asc_move_pub_ = node_->create_publisher<ActionAscPos>(
            "action/move/asc", 10);

        asc_move_res_sub_ = node_->create_subscription<ActionAscPos>(
            "action/move/asc/res", 10, [this](const ActionAscPos::SharedPtr)
            {
                asc_action_->setDisabled(false);
            });

        servo_get_sub_ = node_->create_subscription<ActionServoPos>(
            "action/get/servo", 10, [this](const ActionServoPos::SharedPtr msg)
            {
                if (static_cast<int>(servo_actions_.size()) > msg->id && servo_actions_[msg->id] != nullptr)
                {
                    servo_actions_[msg->id]->SetSpinBoxValue(msg->angle * 180.0 / M_PI);
                }
            });

        servo_set_pub_ = node_->create_publisher<ActionServoPos>(
            "action/set/servo", 10);

        servo_set_res_sub_ = node_->create_subscription<ActionServoPos>(
            "action/set/servo/res", 10, [this](const ActionServoPos::SharedPtr msg)
            {
                if (waiting_for_move_servo_[msg->id])
                {
                    ActionServoPos servo_move;
                    servo_move.id = msg->id;
                    servo_move.pos = msg->pos;
                    servo_move_pub_->publish(servo_move);
                    waiting_for_move_servo_[msg->id] = false;
                }
            });

        servo_move_pub_ = node_->create_publisher<ActionServoPos>(
            "action/move/servo", 10);

        servo_move_res_sub_ = node_->create_subscription<ActionServoPos>(
            "action/move/servo/res", 10, [this](const ActionServoPos::SharedPtr msg)
            {
                if (static_cast<int>(servo_actions_.size()) > msg->id && servo_actions_[msg->id] != nullptr)
                {
                    servo_actions_[msg->id]->setDisabled(false);
                }
            });

        relay_get_sub_ = node_->create_subscription<ActionRelayState>(
            "action/get/relay", 10, [this](const ActionRelayState::SharedPtr msg)
            {
                if (static_cast<int>(relay_values_.size()) > msg->id)
                {
                    relay_values_[msg->id] = msg->state;
                }
            });

        relay_move_pub_ = node_->create_publisher<ActionRelayState>(
            "action/move/relay", 10);

        relay_move_res_sub_ = node_->create_subscription<ActionRelayState>(
            "action/move/relay/res", 10, [this](const ActionRelayState::SharedPtr msg)
            {
                if (static_cast<int>(relay_buttons_.size()) > msg->id && relay_buttons_[msg->id] != nullptr && static_cast<int>(relay_values_.size()) > msg->id)
                {
                    relay_buttons_[msg->id]->setDisabled(false);
                    relay_values_[msg->id] = msg->state;
                }
            });
    }

    ActionPage::~ActionPage()
    = default;
}
