#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>


namespace Modelec
{
    enum class MissionStatus
    {
        READY,
        RUNNING,
        DONE,
        FINISH_ALL,
        FAILED
    };

    class Mission
    {
    public:
        virtual ~Mission() = default;
        virtual void start(rclcpp::Node::SharedPtr node) = 0;
        virtual void update() = 0;
        virtual void clear() = 0;
        virtual MissionStatus getStatus() const = 0;
        virtual std::string name() const = 0;
    };
}
