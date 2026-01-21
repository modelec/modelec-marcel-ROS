#pragma once

#include <queue>
#include <tinyxml2.h>
#include <modelec_utils/point.hpp>

namespace Modelec
{
    class DepositeZone
    {
    public:
        DepositeZone(tinyxml2::XMLElement* obstacleElem);

        Point GetPosition() const;

        Point GetBestTakePosition(const Point& currentPos, int dist = CLOSE_DISTANCE) const;

        void Validate(bool valid);

        bool Validate() const;

        int GetId() const;
        int GetMaxPot() const;

        int GetWidth() const;
        int GetHeight() const;

    protected:
        int id_, max_pot_;
        int w_, h_;
        Point position_;

        std::vector<double> take_angle_;

        bool has_box_ = false;
    };
}
