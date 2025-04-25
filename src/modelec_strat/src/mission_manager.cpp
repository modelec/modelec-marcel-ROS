#include <modelec_strat/mission_manager.hpp>

namespace Modelec
{
    MissionManager::MissionManager()
    {
    }

    MissionManager::MissionManager(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
    }

    rclcpp::Node::SharedPtr MissionManager::getNode() const
    {
        return node_;
    }
}
