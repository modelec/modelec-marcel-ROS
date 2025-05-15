#include <modelec_strat/deposite_zone.hpp>
#include <rclcpp/logging.hpp>

namespace Modelec
{
    DepositeZone::DepositeZone(tinyxml2::XMLElement* obstacleElem)
    {
        obstacleElem->QueryIntAttribute("id", &id_);
        obstacleElem->QueryIntAttribute("team", &team_);
        obstacleElem->QueryIntAttribute("max_pot", &max_pot_);

        auto posElem = obstacleElem->FirstChildElement("Pos");
        if (posElem)
        {
            posElem->QueryIntAttribute("x", &position_.x);
            posElem->QueryIntAttribute("y", &position_.y);
            posElem->QueryDoubleAttribute("theta", &position_.theta);
            posElem->QueryIntAttribute("w", &w_);
            posElem->QueryIntAttribute("h", &h_);
        }

        auto posPotList = obstacleElem->FirstChildElement("PotPos");
        if (posPotList)
        {
            for (auto elemPos = posPotList->FirstChildElement("Pos"); elemPos; elemPos = elemPos->
                 NextSiblingElement("Pos"))
            {
                Point pos;
                elemPos->QueryIntAttribute("x", &pos.x);
                elemPos->QueryIntAttribute("y", &pos.y);
                elemPos->QueryDoubleAttribute("theta", &pos.theta);
                pot_queue_.emplace(pos);
            }
        }
    }
}
