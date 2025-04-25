#include <modelec_strat/strat_fms.hpp>

namespace Modelec
{
    StratFMS::StratFMS() : Node("start_fms")
    {
    }

    void StratFMS::Init()
    {
        nav_ = std::make_unique<NavigationHelper>(shared_from_this());
        mission_manager_ = std::make_unique<MissionManager>(shared_from_this());
        action_executor_ = std::make_unique<ActionExecutor>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "Strat fully initialized");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::StratFMS>();
    node->Init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
