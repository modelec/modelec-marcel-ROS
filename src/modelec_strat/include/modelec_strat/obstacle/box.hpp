#pragma once

#include "obstacle.hpp"
#include <modelec_interfaces/msg/odometry_pos.hpp>

namespace Modelec
{
    class BoxObstacle : public Obstacle
    {
    public:
        BoxObstacle() = default;
        BoxObstacle(const BoxObstacle&) = default;

        BoxObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        BoxObstacle(tinyxml2::XMLElement* obstacleElem);
        BoxObstacle(const modelec_interfaces::msg::Obstacle& msg);

        Point GetOptimizedGetPos(const modelec_interfaces::msg::OdometryPos::SharedPtr& msg) const;
        Point GetOptimizedGetPos(const Point& currentPos) const;

        std::vector<Point> GetAllPossiblePositions() const;

    protected:

        std::vector<double> possible_angles_;
    };
}
