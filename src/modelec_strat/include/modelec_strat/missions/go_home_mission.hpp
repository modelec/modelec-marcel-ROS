#pragma once

#include <modelec_strat/missions/move_mission.hpp>
#include <std_msgs/msg/int64.hpp>

#include "action_mission.hpp"
#include "min_time_mission.hpp"
#include "modelec_strat/action_executor.hpp"

namespace Modelec
{
    class GoHomeMission : public MoveMission, public MinTimeMission, public ActionMission
    {
    public:
        GoHomeMission(const std::shared_ptr<NavigationHelper>& nav,
                          const std::shared_ptr<ActionExecutor>& action_executor,
                          const rclcpp::Time& start_time);

        void Start(const rclcpp::Node::SharedPtr& node) override;
        bool Update() override;
        void Clear() override;
        std::string GetName() const override { return "GoHome"; }

    private:
        enum Step
        {
            ROTATE_TO_HOME,
            AWAIT_90,
            GO_HOME,
            GO_CLOSE,
            RELEASE_BLOCK_IF_NOT_EMPTY,
            DONE,
        };

        rclcpp::Time go_home_time_;
        rclcpp::Time start_time_;
        Point home_point_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;
        int mission_score_ = 0;
    };
}
