#include <modelec_strat/missions/mission_base.hpp>

namespace Modelec
{
    Mission::Mission(MissionStatus status) 
        : status_(status) 
    {
    }

    void Mission::Start(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
    }

    MissionStatus Mission::GetStatus() const
    {
        return status_;
    }
}