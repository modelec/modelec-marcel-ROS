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
        }
    }

    Point DepositeZone::GetPosition() const
    {
        return position_;
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
