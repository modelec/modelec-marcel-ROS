#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_strat/pami_manager.hpp>

namespace Modelec
{
    PamiManger::PamiManger() : Node("pami_manager")
    {
        std::string config_file = ament_index_cpp::get_package_share_directory("modelec_strat") +
            "/data/config.xml";

        if (!Config::load(config_file))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load configuration");
        }

        time_to_put_zone_ = Config::get<int>("config.pami.time_to_put_zone", 80);
        time_to_remove_top_pot_ = Config::get<int>("config.pami.time_to_remove_top_pot", 70);

        score_goupie_ = Config::get<int>("config.mission_score.pami.goupie");
        score_superstar_ = Config::get<int>("config.mission_score.pami.superstar");
        score_all_party_ = Config::get<int>("config.mission_score.pami.all_party");
        score_free_zone_ = 0;

        score_to_add_ = score_goupie_ + score_superstar_ + score_all_party_ + score_free_zone_;

        std::string obstacles_path = ament_index_cpp::get_package_share_directory("modelec_strat") +
            "/data/pami_zone.xml";
        if (!ReadFromXML(obstacles_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load obstacles from XML");
        }

        start_time_sub_ = create_subscription<std_msgs::msg::Int64>(
            "/strat/start_time", 10, [this](const std_msgs::msg::Int64::SharedPtr msg)
            {
                auto now = std::chrono::system_clock::now().time_since_epoch();
                auto goal = std::chrono::nanoseconds(msg->data) + std::chrono::seconds(time_to_remove_top_pot_);
                auto second_goal = std::chrono::nanoseconds(msg->data) + std::chrono::seconds(time_to_put_zone_);

                timer_remove_ = create_wall_timer(
                    goal - now,
                    [this]()
                    {
                        modelec_interfaces::msg::Obstacle topLeft;
                        topLeft.id = 10;
                        remove_obs_pub_->publish(topLeft);

                        modelec_interfaces::msg::Obstacle topRight;
                        topRight.id = 20;
                        remove_obs_pub_->publish(topRight);

                        timer_remove_->cancel();
                    });

                timer_add_ = create_wall_timer(
                    second_goal - now,
                    [this]()
                    {
                        for (const auto & obs : zones_)
                        {
                            add_obs_pub_->publish(obs.toMsg());

                            std_msgs::msg::Int64 msg;
                            msg.data = score_to_add_;
                            score_pub_->publish(msg);
                        }

                        timer_add_->cancel();
                    });
            });

        add_obs_pub_ = create_publisher<modelec_interfaces::msg::Obstacle>(
            "obstacle/add", 10);

        remove_obs_pub_ = create_publisher<modelec_interfaces::msg::Obstacle>(
            "obstacle/remove", 10);

        score_pub_ = create_publisher<std_msgs::msg::Int64>("/strat/score", 10);

        reset_strat_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/strat/reset", 10, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                if (timer_add_)
                {
                    timer_add_->cancel();
                }
                if (timer_remove_)
                {
                    timer_remove_->cancel();
                }
            });
    }

    bool PamiManger::ReadFromXML(const std::string& filename)
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load obstacle XML file: %s", filename.c_str());
            return false;
        }

        tinyxml2::XMLElement* root = doc.FirstChildElement("pami_zone");
        if (!root)
        {
            RCLCPP_ERROR(get_logger(), "No <pami-zone> root element in file");
            return false;
        }

        for (tinyxml2::XMLElement* elem = root->FirstChildElement("zone");
            elem;
            elem = elem->NextSiblingElement("zone"))
        {
            zones_.push_back(Obstacle(elem));
        }

        RCLCPP_INFO(get_logger(), "Loaded zone obstacles from XML");
        return true;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::PamiManger>());
    rclcpp::shutdown();
    return 0;
}
