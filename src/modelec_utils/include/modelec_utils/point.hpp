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
        static double angleDiff(const Point& p1, const Point& p2);

        [[nodiscard]] Point GetTakePosition(int distance, double angle) const;
        [[nodiscard]] Point GetTakePosition(int distance) const;

        [[nodiscard]] Point GetTakeBasePosition() const;
        [[nodiscard]] Point GetTakeClosePosition() const;
    };
}
