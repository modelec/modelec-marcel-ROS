#pragma once

#include <modelec_strat/missions/move_mission.hpp>
#include <std_msgs/msg/int64.hpp>

namespace Modelec
{
    class GoHomeMission : public MoveMission
    {
    public:
        GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time);

        void Start(const rclcpp::Node::SharedPtr& node) override;
        bool Update() override;
        void Clear() override;
        std::string GetName() const override { return "GoHome"; }

    private:
        enum Step
        {
            ROTATE_TO_HOME,
            GO_HOME,
            GO_CLOSE,
            DONE,
        };

        rclcpp::Time go_home_time_;
        rclcpp::Time start_time_;
        Point home_point_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;
        int mission_score_ = 0;
    };
}
