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
}
