#include <cmath>
#include <modelec_utils/point.hpp>

namespace Modelec
{
    double Point::distance(const Point& p1, const Point& p2)
    {
        return sqrt(std::pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    Point Point::GetTakePosition(int distance, double angle) const
    {
        Point pos;
        pos.x = x + distance * cos(angle);
        pos.y = y + distance * sin(angle);
        pos.theta = angle + M_PI;
        return pos;
    }

    Point Point::GetTakePosition(int distance) const
    {
        return GetTakePosition(distance, theta);
    }

    Point Point::GetTakeBasePosition() const
    {
        return GetTakePosition(320, theta);
    }

    Point Point::GetTakeClosePosition() const
    {
        return GetTakePosition(210, theta);
    }
}
