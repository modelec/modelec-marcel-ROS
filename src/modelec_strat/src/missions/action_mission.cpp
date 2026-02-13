#include <modelec_strat/missions/action_mission.hpp>
#include <modelec_strat/action_executor.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec {

    double ActionMission::TIMEOUT = 5.0;
    bool ActionMission::IsInit = false;

    ActionMission::ActionMission(const std::shared_ptr<ActionExecutor>& action_executor)
        : action_executor_(action_executor)
    {
    }

    void ActionMission::InitConfig()
    {
        TIMEOUT = Config::get<double>("config.mission.action.timeout.s", 5.0);
        IsInit = true;
    }

    void ActionMission::Start(const rclcpp::Node::SharedPtr& node)
    {
        Mission::Start(node);

        if (!IsInit)
        {
            InitConfig();
        }

        deploy_time_ = node->now();
    }

    bool ActionMission::Update()
    {
        if (!action_executor_->IsActionDone())
        {
            if ((node_->now() - deploy_time_).seconds() > TIMEOUT)
            {
                RCLCPP_WARN(node_->get_logger(), "ActionMission Update TIMEOUT");
                return true;
            }
            return false;
        }

        return true;
    }
}