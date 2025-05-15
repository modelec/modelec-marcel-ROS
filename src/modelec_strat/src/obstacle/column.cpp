#include <modelec_strat/obstacle/column.hpp>

namespace Modelec
{
    ColumnObstacle::ColumnObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type)
        : Obstacle(id, x, y, theta, w, h, type)
    {
    }

    ColumnObstacle::ColumnObstacle(tinyxml2::XMLElement* obstacleElem)
        : Obstacle(obstacleElem)
    {
        possible_angles_.push_back(theta_);
        for (auto elem = obstacleElem->FirstChildElement("possible-angle");
             elem;
             elem = elem->NextSiblingElement("possible-angle"))
        {
            possible_angles_.push_back(elem->DoubleAttribute("theta"));
        }
    }

    ColumnObstacle::ColumnObstacle(const modelec_interfaces::msg::Obstacle& msg) :
        Obstacle(msg)
    {
    }

    bool ColumnObstacle::IsAtObjective() const
    {
        return isAtObjective;
    }

    void ColumnObstacle::SetAtObjective(bool atObjective)
    {
        isAtObjective = atObjective;
    }

    Point ColumnObstacle::GetOptimizedGetPos(const modelec_interfaces::msg::OdometryPos::SharedPtr& msg) const
    {
        Point p = Point(msg->x, msg->y, msg->theta);
        return GetOptimizedGetPos(p);
    }

    Point ColumnObstacle::GetOptimizedGetPos(const Point& currentPos) const
    {
        auto distance = std::numeric_limits<double>::max();
        double optimizedAngle = 0;
        for (const auto& angle : possible_angles_)
        {
            auto newPos = GetPosition().GetTakePosition(400, angle);
            double dist = Point::distance(currentPos, newPos);
            if (dist < distance)
            {
                distance = dist;
                optimizedAngle = angle;
            }
        }

        return Point(x_, y_, optimizedAngle);
    }

    std::vector<Point> ColumnObstacle::GetAllPossiblePositions() const
    {
        std::vector<Point> positions;
        for (const auto& angle : possible_angles_)
        {
            positions.push_back(Point(x_, y_, angle));
        }
        return positions;
    }
}
