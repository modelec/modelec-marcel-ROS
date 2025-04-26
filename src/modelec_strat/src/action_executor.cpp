#include <modelec_strat/action_executor.hpp>

namespace Modelec
{
    ActionExecutor::ActionExecutor()
    {
    }

    ActionExecutor::ActionExecutor(const rclcpp::Node::SharedPtr& node) : node_(node)
    {
    }

    rclcpp::Node::SharedPtr ActionExecutor::getNode() const
    {
        return node_;
    }
}
