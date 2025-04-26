#include <modelec_strat/mission_manager.hpp>

namespace Modelec
{
    MissionManager::MissionManager()
    {
    }

    MissionManager::MissionManager(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
    }

    rclcpp::Node::SharedPtr MissionManager::getNode() const
    {
        return node_;
    }
}
