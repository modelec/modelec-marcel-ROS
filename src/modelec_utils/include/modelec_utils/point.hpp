#pragma once

namespace Modelec
{
    struct Point {
        int x;
        int y;
        double theta;

        Point() : x(0), y(0), theta(0) {}
        Point(int x, int y, double theta) : x(x), y(y), theta(theta) {}

        static double distance(const Point& p1, const Point& p2);

        Point GetTakePosition(int distance, double angle) const;

        Point GetTakeBasePosition() const;
        Point GetTakeClosePosition() const;
    };
}
