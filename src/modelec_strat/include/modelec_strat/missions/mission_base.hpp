#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <queue>

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
        Mission(MissionStatus status = MissionStatus::READY);
        virtual ~Mission() = default;

        virtual void Start(rclcpp::Node::SharedPtr node);
        virtual bool Update() = 0;
        virtual void Clear() = 0;

        virtual MissionStatus GetStatus() const;
        virtual std::string GetName() const = 0;

    protected:
        std::queue<int> steps_;
        rclcpp::Node::SharedPtr node_;
        MissionStatus status_;
    };
}