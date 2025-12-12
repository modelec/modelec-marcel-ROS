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

        Point GetPosition() const;

        void Validate(bool valid);

        int GetId() const;
        int GetMaxPot() const;

        int GetWidth() const;
        int GetHeight() const;

    protected:
        int id_, max_pot_;
        int w_, h_;
        Point position_;

        bool has_box_ = false;
    };
}
