#pragma once

#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>


namespace Modelec
{
    class GoHomeMission : public Mission
    {
    public:
        GoHomeMission(const std::shared_ptr<NavigationHelper>& nav, const rclcpp::Time& start_time);

        void start(rclcpp::Node::SharedPtr node) override;
        void update() override;
        void clear() override;
        MissionStatus getStatus() const override;
        std::string name() const override { return "GoHome"; }

    private:
        enum Step {
            GO_HOME,
            GO_CLOSE,
            DONE
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time start_time_;
        Point home_point_;
    };
}
