#include <cmath>
#include <modelec_strat/deposite_zone.hpp>
#include <rclcpp/logging.hpp>

namespace Modelec
{
    DepositeZone::DepositeZone(int id)
    {
        id_ = Config::get<int>("Map.DepositeZone[" + std::to_string(id) + "]@id", id);
        w_ = Config::get<int>("Map.DepositeZone[" + std::to_string(id) + "]@w");
        h_ = Config::get<int>("Map.DepositeZone[" + std::to_string(id) + "]@h");
        position_ = Point(
            Config::get<int>("Map.DepositeZone[" + std::to_string(id) + "].Pos@x"),
            Config::get<int>("Map.DepositeZone[" + std::to_string(id) + "].Pos@y"),
            Config::get<double>("Map.DepositeZone[" + std::to_string(id) + "].Pos@theta")
            );

        if (Config::count("Map.DepositeZone[" + std::to_string(id) + "].TakeAngle.Angle") > 0)
        {
            take_angle_ = Config::getArray<double>("Map.DepositeZone[" + std::to_string(id) + "].TakeAngle.Angle");
        } else
        {
            auto take_angle = Config::get<double>("Map.DepositeZone[" + std::to_string(id) + "].TakeAngle.Angle", position_.theta);
            take_angle_ = {take_angle};
        }
    }

    Point DepositeZone::GetPosition() const
    {
        return position_;
    }

    Point DepositeZone::GetBestTakePosition(const Point& currentPos, int dist) const
    {
        if (take_angle_.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("DepositeZone"), "No take angles defined for DepositeZone id=%d", id_);
            return position_;
        }

        double min_distance = std::numeric_limits<double>::max();
        double best_angle = take_angle_.front();

        for (const auto& angle : take_angle_)
        {
            Point p = Point(
                position_.x + dist * std::cos(angle),
                position_.y + dist * std::sin(angle),
                angle);

            double distance = p.distance(currentPos);

            if (distance < min_distance)
            {
                min_distance = distance;
                best_angle = angle;
            }
        }

        Point best_position = position_;
        best_position.theta = best_angle;
        return best_position;
    }

    void DepositeZone::Validate(bool valid)
    {
        has_box_ = valid;
    }

    bool DepositeZone::Validate() const
    { return has_box_; }

    int DepositeZone::GetId() const
    { return id_; }

    int DepositeZone::GetWidth() const
    { return w_; }

    int DepositeZone::GetHeight() const
    { return h_; }
}
