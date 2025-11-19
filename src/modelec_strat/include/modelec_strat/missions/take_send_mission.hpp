#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <modelec_strat/action_executor.hpp>

namespace Modelec {
    class TakeSendMission : public Mission {
    public:
        TakeSendMission(const std::shared_ptr<NavigationHelper>& nav,
                      const std::shared_ptr<ActionExecutor>& action_executor);

        void Start(rclcpp::Node::SharedPtr node) override;
        void Update() override;
        void Clear() override;
        MissionStatus GetStatus() const override;
        std::string GetName() const override { return "GoHome"; }

    private:
        enum Step
        {
            GO_TO_TAKE,
            TAKE,
            WAIT_5S,
            GO_TO_SEND,
            SEND,
            DONE,
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Time go_home_time_;
        rclcpp::Node::SharedPtr node_;

        rclcpp::Time go_timeout_;
        rclcpp::Time deploy_time_;
    };
}