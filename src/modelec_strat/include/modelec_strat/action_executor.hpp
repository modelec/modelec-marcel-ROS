#pragma once

#include <rclcpp/rclcpp.hpp>

namespace Modelec
{

    class ActionExecutor
    {
    public:
        ActionExecutor();

        ActionExecutor(rclcpp::Node::SharedPtr node);

        rclcpp::Node::SharedPtr getNode() const;

    private:
        rclcpp::Node::SharedPtr node_;
    };

}