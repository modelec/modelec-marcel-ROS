#pragma once

#include <limits>
#include <queue>
#include <tinyxml2.h>
#include <modelec_utils/point.hpp>

namespace Modelec
{
    class DepositeZone
    {
    public:
        DepositeZone(tinyxml2::XMLElement* obstacleElem);

        Point GetPosition() const { return position_; }

        Point GetNextPotPos()
        {
            if (pot_queue_.empty())
                return Point(std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), 0);
            return pot_queue_.front();
        }

        Point ValidNextPotPos()
        {
            if (pot_queue_.empty())
                return Point(std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), 0);
            Point p = pot_queue_.front();
            pot_queue_.pop();
            return p;
        }

        int GetTeam() const { return team_; }
        int GetId() const { return id_; }
        int GetMaxPot() const { return max_pot_; }

        int GetWidth() const { return w_; }
        int GetHeight() const { return h_; }

        int RemainingPotPos() const
        {
            return pot_queue_.size();
        }

    protected:
        int id_, team_, max_pot_;
        int w_, h_;
        Point position_;

        std::queue<Point> pot_queue_;
    };
}
