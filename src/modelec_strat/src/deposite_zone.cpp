#include <cmath>
#include <modelec_strat/deposite_zone.hpp>
#include <rclcpp/logging.hpp>

namespace Modelec
{
    DepositeZone::DepositeZone(tinyxml2::XMLElement* obstacleElem)
    {
        obstacleElem->QueryIntAttribute("id", &id_);

        auto posElem = obstacleElem->FirstChildElement("Pos");
        if (posElem)
        {
            posElem->QueryIntAttribute("x", &position_.x);
            posElem->QueryIntAttribute("y", &position_.y);
            posElem->QueryDoubleAttribute("theta", &position_.theta);
            posElem->QueryIntAttribute("w", &w_);
            posElem->QueryIntAttribute("h", &h_);

            if (auto TakeAngleElem = obstacleElem->FirstChildElement("TakeAngle"))
            {
                for (auto takePos = TakeAngleElem->FirstChildElement("Angle"); takePos; takePos = takePos->NextSiblingElement("Angle"))
                {
                    double angle;
                    takePos->QueryDoubleAttribute("value", &angle);
                    take_angle_.push_back(angle);
                }
            }
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
                static_cast<int>(position_.x + dist * std::cos(angle)),
                static_cast<int>(position_.y + dist * std::sin(angle)),
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

    int DepositeZone::GetMaxPot() const
    { return max_pot_; }

    int DepositeZone::GetWidth() const
    { return w_; }

    int DepositeZone::GetHeight() const
    { return h_; }
}
