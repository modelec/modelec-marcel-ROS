#pragma once

#include "obstacle.hpp"
#include <modelec_interfaces/msg/odometry_pos.hpp>

namespace Modelec
{
    class ColumnObstacle : public Obstacle
    {
    public:
        ColumnObstacle() = default;
        ColumnObstacle(const ColumnObstacle&) = default;

        ColumnObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        ColumnObstacle(tinyxml2::XMLElement* obstacleElem);
        ColumnObstacle(const modelec_interfaces::msg::Obstacle& msg);

        bool IsAtObjective() const;
        void SetAtObjective(bool atObjective);

        Point GetOptimizedGetPos(const modelec_interfaces::msg::OdometryPos::SharedPtr& msg) const;
        Point GetOptimizedGetPos(const Point& currentPos) const;

        std::vector<Point> GetAllPossiblePositions() const;

    protected:
        bool isAtObjective = false;

        std::vector<double> possible_angles_;
    };
}
