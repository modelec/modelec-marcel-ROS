#include <cmath>
#include <modelec_utils/point.hpp>

namespace Modelec
{
    double Point::distance(const Point& p1, const Point& p2)
    {
        return sqrt(std::pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    Point Point::GetTakeBasePosition()
    {
        Point pos;
        pos.x = x + 400 * cos(theta);
        pos.y = y + 400 * sin(theta);
        pos.theta = theta + M_PI;
        return pos;
    }

    Point Point::GetTakeClosePosition()
    {
        Point pos;
        pos.x = x + 210 * cos(theta);
        pos.y = y + 210 * sin(theta);
        pos.theta = theta + M_PI;
        return pos;
    }

}
