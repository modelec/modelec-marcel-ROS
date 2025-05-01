#pragma once

#include <modelec_strat/navigation_helper.hpp>

#include "mission_base.hpp"

namespace Modelec
{
    class GoHomeMission : public Mission
    {
    public:
        GoHomeMission(const std::shared_ptr<NavigationHelper>& nav);

        void start(rclcpp::Node::SharedPtr node) override;
        void update() override;
        MissionStatus getStatus() const override;
        std::string name() const override { return "GoHome"; }

    private:
        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        rclcpp::Node::SharedPtr node_;
    };
}
