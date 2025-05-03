#pragma once

#include <queue>
#include <tinyxml2.h>
#include <modelec_utils/point.hpp>

namespace Modelec {
    class DepositeZone
    {
    public:
        DepositeZone(tinyxml2::XMLElement * obstacleElem);

        Point GetPosition() const { return position_; }
        Point GetNextPotPos()
        {
            if (pot_queue_.empty())
                return Point();
            auto elem = pot_queue_.front();
            pot_queue_.pop();
            return elem;
        }

        int GetTeam() const { return team_; }
        int GetId() const { return id_; }
        int GetMaxPot() const { return max_pot_; }

    protected:
        int id_, team_, max_pot_;
        Point position_;

        std::queue<Point> pot_queue_;
    };
}
