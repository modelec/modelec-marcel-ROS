#include <modelec_strat/obstacle/box.hpp>

#include "modelec_utils/utils.hpp"

namespace Modelec
{
    BoxObstacle::BoxObstacle(int id, int x, int y, double theta, int w, int h, const std::string& type)
        : Obstacle(id, x, y, theta, w, h, type)
    {
    }

    BoxObstacle::BoxObstacle(int id) : Obstacle(id)
    {
        if (Config::count("Obstacles.Obstacle[" + std::to_string(id) + "].possible-angle") > 0)
        {
            possible_angles_ = Config::getArray<double>("Obstacles.Obstacle[" + std::to_string(id) + "].possible-angle", [](const std::string& base)
            {
                return Config::get<double>(base + "@theta");
            });
            possible_angles_.push_back(theta_);
        } else {
            possible_angles_ = {theta_};
        }
    }

    BoxObstacle::BoxObstacle(const modelec_interfaces::msg::Obstacle& msg) :
        Obstacle(msg)
    {
    }

    Point BoxObstacle::GetOptimizedGetPos(const modelec_interfaces::msg::OdometryPos::SharedPtr& msg) const
    {
        Point p = Point(msg->x, msg->y, msg->theta);
        return GetOptimizedGetPos(p);
    }

    Point BoxObstacle::GetOptimizedGetPos(const Point& currentPos) const
    {
        auto distance = std::numeric_limits<double>::max();
        double optimizedAngle = 0;
        for (const auto& angle : possible_angles_)
        {
            auto newPos = GetPosition().GetTakePosition(310, angle);
            double dist = Point::distance(currentPos, newPos);
            if (dist < distance)
            {
                distance = dist;
                optimizedAngle = angle;
            }
        }

        return {x_, y_, optimizedAngle};
    }

    std::vector<Point> BoxObstacle::GetAllPossiblePositions() const
    {
        std::vector<Point> positions;
        for (const auto& angle : possible_angles_)
        {
            positions.push_back(Point(x_, y_, angle));
        }
        return positions;
    }

    void BoxObstacle::SetColor(size_t index, Team team)
    {
        if (index < colors_.size())
        {
            colors_[index] = team;
        }
    }

    BoxObstacle::Team BoxObstacle::GetColor(size_t index) const
    {
        if (index < colors_.size())
        {
            return colors_[index];
        }
        return YELLOW;
    }

    std::array<BoxObstacle::Team, 4> BoxObstacle::GetColors() const
    {
        return colors_;
    }

    std::vector<int> BoxObstacle::GetSide(Team team) const
    {
        std::vector<int> sideColors;
        for (size_t i = 0; i < colors_.size(); ++i)
        {
            if (colors_[i] == team)
            {
                sideColors.push_back(i);
            }
        }
        return sideColors;
    }

    void BoxObstacle::ParseColor(const std::string& colorStr)
    {
        std::vector<std::string> tokens = split(colorStr, ';');
        for (size_t i = 0; i < tokens.size() && i < colors_.size(); ++i)
        {
            if (tokens[i] == "yellow")
            {
                colors_[i] = YELLOW;
            }
            else if (tokens[i] == "blue")
            {
                colors_[i] = BLUE;
            }
        }
    }

    void BoxObstacle::ParseColor(const std::vector<std::string>& colorVec)
    {
        for (size_t i = 0; i < colorVec.size() && i < colors_.size(); ++i)
        {
            if (colorVec[i] == "yellow")
            {
                colors_[i] = YELLOW;
            }
            else if (colorVec[i] == "blue")
            {
                colors_[i] = BLUE;
            }
        }
    }
}
