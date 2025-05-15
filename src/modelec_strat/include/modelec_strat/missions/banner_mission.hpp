#pragma once

#include <modelec_strat/action_executor.hpp>
#include <modelec_strat/missions/mission_base.hpp>
#include <modelec_strat/navigation_helper.hpp>
#include <std_msgs/msg/int64.hpp>

namespace Modelec
{
    /*
     * Banderole mission
     *
     */
    class BannerMission : public Mission
    {
    public:
        BannerMission(const std::shared_ptr<NavigationHelper>& nav,
                        const std::shared_ptr<ActionExecutor>& action_executor);

        void start(rclcpp::Node::SharedPtr node) override;
        void update() override;
        void clear() override;
        MissionStatus getStatus() const override;
        std::string name() const override { return "Promotion"; }

        enum Step {
            GO_TO_FRONT,
            DEPLOY_BANNER,
            GO_FORWARD,
            DONE
        } step_;

        MissionStatus status_;
        std::shared_ptr<NavigationHelper> nav_;
        std::shared_ptr<ActionExecutor> action_executor_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr score_pub_;
        int mission_score_ = 0;
    };
}
