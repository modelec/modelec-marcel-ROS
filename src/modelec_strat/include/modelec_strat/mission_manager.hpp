#pragma once

#include <rclcpp/rclcpp.hpp>

namespace Modelec
{

    class MissionManager
    {
    public:
        MissionManager();

        MissionManager(const rclcpp::Node::SharedPtr& node);

        rclcpp::Node::SharedPtr getNode() const;

    private:
        rclcpp::Node::SharedPtr node_;
    };

}