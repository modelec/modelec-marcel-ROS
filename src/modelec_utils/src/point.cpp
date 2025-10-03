#include <cmath>
#include <modelec_utils/point.hpp>

namespace Modelec
{
    double Point::distance(const Point& p1, const Point& p2)
    {
        return sqrt(std::pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    double Point::angleDiff(const Point &p1, const Point &p2) {
        double diff = std::fmod(p1.theta - p2.theta + M_PI, 2 * M_PI);
        if (diff < 0)
            diff += 2 * M_PI;
        return diff - M_PI;
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
        return GetTakePosition(290, theta);
    }

    Point Point::GetTakeClosePosition() const
    {
        return GetTakePosition(150, theta);
    }
}
