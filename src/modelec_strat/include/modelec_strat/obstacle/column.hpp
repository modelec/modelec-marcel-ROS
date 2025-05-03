#pragma once

#include "obstacle.hpp"

namespace Modelec
{
    class ColumnObstacle : public Obstacle
    {
    public:
        ColumnObstacle() = default;
        ColumnObstacle(const ColumnObstacle&) = default;

        ColumnObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        ColumnObstacle(tinyxml2::XMLElement * obstacleElem);
        ColumnObstacle(const modelec_interfaces::msg::Obstacle& msg);

        bool IsAtObjective() const;
        void SetAtObjective(bool atObjective);

    protected:
        bool isAtObjective = false;
    };
}
