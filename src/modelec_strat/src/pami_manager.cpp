#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_strat/pami_manager.hpp>

namespace Modelec
{
    PamiManger::PamiManger() : Node("pami_manager")
    {
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::PamiManger>());
    rclcpp::shutdown();
    return 0;
}
