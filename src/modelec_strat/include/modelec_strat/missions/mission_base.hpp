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
        virtual void Start(rclcpp::Node::SharedPtr node) = 0;
        virtual void Update() = 0;
        virtual void Clear() = 0;
        virtual MissionStatus GetStatus() const = 0;
        virtual std::string GetName() const = 0;
    };
}
