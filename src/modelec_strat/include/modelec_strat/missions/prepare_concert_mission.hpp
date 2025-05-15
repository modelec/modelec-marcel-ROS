#pragma once

#include <modelec_strat/action_executor.hpp>
#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <modelec_strat/obstacle/column.hpp>
#include <std_msgs/msg/int64.hpp>

namespace Modelec
{
    class PrepareConcertMission : public Mission
    {
    public:
        PrepareConcertMission(const std::shared_ptr<NavigationHelper>& nav,
                              const std::shared_ptr<ActionExecutor>& action_executor);

        void Start(rclcpp::Node::SharedPtr node) override;
        void Update() override;
        void Clear() override;
        MissionStatus GetStatus() const override;
        std::string GetName() const override { return "PrepareConcert"; }

    private:
        enum Step
        {
            GO_TO_COLUMN,
            GO_CLOSE_TO_COLUMN,
            TAKE_COLUMN,
            GO_BACK,
            GO_TO_PLATFORM,
            GO_CLOSE_TO_PLATFORM,
            PLACE_PLATFORM,
            GO_BACK_2,
            DONE
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<ColumnObstacle> column_;
        std::shared_ptr<DepositeZone> closestDepoZone_;
        Point closestDepoZonePoint_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;
        int mission_score_ = 0;
    };
}
