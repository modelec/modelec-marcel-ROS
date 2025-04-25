#include <modelec_strat/action_executor.hpp>

namespace Modelec
{
    ActionExecutor::ActionExecutor()
    {
    }

    ActionExecutor::ActionExecutor(rclcpp::Node::SharedPtr node) : node_(std::move(node))
    {
    }

    rclcpp::Node::SharedPtr ActionExecutor::getNode() const
    {
        return node_;
    }
}
