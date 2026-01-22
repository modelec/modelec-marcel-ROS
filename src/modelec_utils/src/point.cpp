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

    double Point::distance(const Point& p2) const
    {
        return distance(*this, p2);
    }

    double Point::angleDiff(const Point& p2) const
    {
        return angleDiff(*this, p2);
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
        return GetTakePosition(BASE_DISTANCE, theta);
    }

    Point Point::GetTakeClosePosition() const
    {
        return GetTakePosition(CLOSE_DISTANCE, theta);
    }
}
