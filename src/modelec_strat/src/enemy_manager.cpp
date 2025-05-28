#include <modelec_strat/enemy_manager.hpp>
#include <cmath>
#include <modelec_utils/config.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Modelec
{
    EnemyManager::EnemyManager() : Node("enemy_manager")
    {
        std::string config_path = ament_index_cpp::get_package_share_directory("modelec_strat") + "/data/config.xml";
        if (!Config::load(config_path))
        {
            RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", config_path.c_str());
        }

        min_move_threshold_mm_ = Config::get<float>("config.enemy.detection.min_move_threshold_mm", 50);

        refresh_rate_s_ = Config::get<float>("config.enemy.detection.refresh_rate", 1.0);

        max_stationary_time_s_ = Config::get<float>("config.enemy.detection.max_stationary_time_s", 10.0);

        map_width_ = Config::get<float>("config.map.size.map_width_mm", 3000.0);
        map_height_ = Config::get<float>("config.map.size.map_height_mm", 2000.0);

        margin_detection_table_ = Config::get<float>("config.enemy.detection.margin_detection_table_mm", 50.0);

        robot_width_ = Config::get<float>("config.robot.size.width_mm", 500.0);
        robot_length_ = Config::get<float>("config.robot.size.length_mm", 500.0);
        robot_radius_ = std::max(robot_width_, robot_length_) / 3.0;

        min_emergency_distance_ = Config::get<float>("config.enemy.detection.min_emergency_distance_mm", 100.0);

        current_pos_sub_ = this->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                OnCurrentPos(msg);
            });

        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                OnLidarData(msg);
            });

        enemy_pos_pub_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "enemy/position", 10);

        close_enemy_pos_pub_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "enemy/position/emergency", 10);

        state_sub_ = create_subscription<modelec_interfaces::msg::StratState>("/strat/state", 10,
                                                                              [this](
                                                                              const
                                                                              modelec_interfaces::msg::StratState::SharedPtr
                                                                              msg)
                                                                              {
                                                                                  if (!game_stated_ && msg->state ==
                                                                                      modelec_interfaces::msg::StratState::SELECT_MISSION)
                                                                                  {
                                                                                      game_stated_ = true;
                                                                                  }
                                                                              });

        enemy_long_time_pub_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "/enemy/long_time", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                TimerCallback();
            }
        );

        last_publish_time_ = this->now();

        ping_service_ = this->create_service<std_srvs::srv::Empty>(
            "enemy_manager/ping",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
            {
            });
    }

    void EnemyManager::OnCurrentPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        current_pos_ = *msg;
    }

    void EnemyManager::OnLidarData(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (std::isnan(current_pos_.x) || std::isnan(current_pos_.y))
        {
            RCLCPP_WARN(this->get_logger(), "Current robot position unknown, cannot compute enemy position");
            return;
        }

        const double robot_x = current_pos_.x;
        const double robot_y = current_pos_.y;
        const double robot_theta = current_pos_.theta;

        double angle = msg->angle_min;

        double min_distance = std::numeric_limits<double>::max();
        double best_x = 0.0;
        double best_y = 0.0;

        // RCLCPP_INFO(this->get_logger(), "Processing Lidar min %f | %f | %f", msg->range_min, msg->range_max, msg->ranges[1]);
        // RCLCPP_INFO(this->get_logger(), "Robot pos %f | %f | %f", robot_x, robot_y, robot_theta);

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];

            if (std::isnan(range) || range < msg->range_min || range > msg->range_max)
            {
                // RCLCPP_INFO(this->get_logger(), "Ignoring Lidar point with invalid range: %.2f", range);

                angle += msg->angle_increment;
                continue;
            }

            double range_mm = range * 1000.0;  // conversion en mm

            if (range_mm < robot_radius_)
            {
                RCLCPP_INFO(this->get_logger(), "Ignoring Lidar point too close to robot body: range=%.2f mm", range_mm);
                angle += msg->angle_increment;
                continue;
            }

            // EMERGENCY detection: obstacle very close but not the robot itself
            if (range_mm < min_emergency_distance_)
            {
                // Convert to local robot frame
                double x_local = range * std::cos(angle) * 1000.0; // meters -> mm
                double y_local = range * std::sin(angle) * 1000.0; // meters -> mm

                // Rotate + translate into global frame
                double x_global = robot_x + (x_local * std::cos(robot_theta) - y_local * std::sin(robot_theta));
                double y_global = robot_y + (x_local * std::sin(robot_theta) + y_local * std::cos(robot_theta));

                // Check if in bounds
                if (x_global >= 0 && x_global <= (map_width_ - margin_detection_table_) && y_global >= 0 && y_global <= (map_height_ - margin_detection_table_))
                {
                    modelec_interfaces::msg::OdometryPos emergency_msg;
                    emergency_msg.x = x_global;
                    emergency_msg.y = y_global;
                    emergency_msg.theta = 0.0;

                    close_enemy_pos_pub_->publish(emergency_msg);
                    RCLCPP_WARN(this->get_logger(), "EMERGENCY CLOSE OBJECT DETECTED at x=%.2f y=%.2f (%.1f mm)", x_global, y_global, range_mm);
                }

                break;;
            }

            // Convert to local robot frame
            double x_local = range * std::cos(angle) * 1000.0; // meters -> mm
            double y_local = range * std::sin(angle) * 1000.0; // meters -> mm

            // Rotate + translate into global frame
            double x_global = robot_x + (x_local * std::cos(robot_theta) - y_local * std::sin(robot_theta));
            double y_global = robot_y + (x_local * std::sin(robot_theta) + y_local * std::cos(robot_theta));

            // Ignore points outside of the table
            if (x_global < 0 || x_global > (map_width_ - margin_detection_table_) || y_global < 0 || y_global > (map_height_ - margin_detection_table_))
            {
                // RCLCPP_INFO(this->get_logger(), "Lidar point out of bounds: x=%.2f, y=%.2f", x_global, y_global);

                angle += msg->angle_increment;
                continue;
            }

            // RCLCPP_INFO(this->get_logger(), "Lidar point at x=%.2f, y=%.2f, range=%.2f", x_global, y_global, range);

            if (range < min_distance)
            {
                min_distance = range;
                best_x = x_global;
                best_y = y_global;
            }

            angle += msg->angle_increment;
        }

        if (min_distance < std::numeric_limits<double>::max())
        {
            modelec_interfaces::msg::OdometryPos enemy_pos;
            enemy_pos.x = best_x;
            enemy_pos.y = best_y;
            enemy_pos.theta = 0.0;

            bool need_publish = false;

            if (!enemy_initialized_)
            {
                need_publish = true;
                enemy_initialized_ = true;
            }
            else
            {
                float dx = enemy_pos.x - last_enemy_pos_.x;
                float dy = enemy_pos.y - last_enemy_pos_.y;
                float distance_squared = dx * dx + dy * dy;

                if (distance_squared > min_move_threshold_mm_ * min_move_threshold_mm_)
                {
                    need_publish = true;
                }
            }

            if (need_publish)
            {
                last_enemy_pos_ = enemy_pos;
                last_publish_time_ = this->now();
                last_movement_time_ = this->now(); // Mise à jour du dernier vrai mouvement
                enemy_pos_pub_->publish(enemy_pos);
                RCLCPP_INFO(this->get_logger(), "Enemy moved: x=%d, y=%d", enemy_pos.x, enemy_pos.y);
            }
            else
            {
                auto now = this->now();
                if ((now - last_movement_time_).seconds() > max_stationary_time_s_)
                {
                    enemy_long_time_pub_->publish(last_enemy_pos_);
                    RCLCPP_WARN(this->get_logger(), "Enemy has been stationary for too long at x=%d y=%d",
                                last_enemy_pos_.x, last_enemy_pos_.y);

                    last_movement_time_ = now;
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No enemy detected in Lidar scan");
        }
    }

    void EnemyManager::TimerCallback()
    {
        if (!enemy_initialized_)
            return;

        rclcpp::Duration duration_since_last = this->now() - last_publish_time_;
        if (duration_since_last.seconds() >= refresh_rate_s_)
        {
            enemy_pos_pub_->publish(last_enemy_pos_);
            last_publish_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Periodic refresh of enemy position");
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::EnemyManager>());
    rclcpp::shutdown();
    return 0;
}
