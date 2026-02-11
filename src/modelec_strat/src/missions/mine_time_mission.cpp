#include <modelec_strat/missions/min_time_mission.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec {

    double MinTimeMission::MIN_TIME_DURATION = 0.1;
    bool MinTimeMission::IsInit = false;

    MinTimeMission::MinTimeMission() : min_time_(std::nullopt)
    {
    }

    void MinTimeMission::InitConfig()
    {
        MIN_TIME_DURATION = Config::get<double>("config.mission.min_time.min_time_duration.s", 0.1);
        IsInit = true;
    }

    void MinTimeMission::Start(rclcpp::Node::SharedPtr node)
    {
        Mission::Start(node);

        if (!IsInit)
        {
            InitConfig();
        }
    }

    bool MinTimeMission::Update()
    {
        if (min_time_.has_value())
        {
            if ((node_->now() - min_time_.value()).seconds() < MIN_TIME_DURATION)
            {
                return false;
            }
            else
            {
                min_time_.reset();
            }
        }
        return true;
    }
}