#pragma once

#include "obstacle.hpp"
#include <modelec_interfaces/msg/odometry_pos.hpp>

namespace Modelec
{
    class BoxObstacle : public Obstacle
    {
    public:
        enum Team
        {
            YELLOW = 0,
            BLUE = 1,
        };

        BoxObstacle() = default;
        BoxObstacle(const BoxObstacle&) = default;

        BoxObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        BoxObstacle(tinyxml2::XMLElement* obstacleElem);
        BoxObstacle(const modelec_interfaces::msg::Obstacle& msg);

        Point GetOptimizedGetPos(const modelec_interfaces::msg::OdometryPos::SharedPtr& msg) const;
        Point GetOptimizedGetPos(const Point& currentPos) const;

        std::vector<Point> GetAllPossiblePositions() const;

        void SetColor(size_t index, Team team);
        Team GetColor(size_t index) const;
        std::array<Team, 4> GetColors() const;

        std::vector<int> GetSide(Team team) const;

    protected:

        std::vector<double> possible_angles_;

        std::array<Team, 4> colors_ = {
            YELLOW,
            YELLOW,
            YELLOW,
            YELLOW
        };
    };
}
