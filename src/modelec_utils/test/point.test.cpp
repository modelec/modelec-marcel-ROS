#include <gtest/gtest.h>
#include <modelec_utils/point.hpp>
#include <cmath>

TEST(PointTest, Distance) {
    Modelec::Point p1(0, 0, 0);
    Modelec::Point p2(3, 4, 0);
    double dist = Modelec::Point::distance(p1, p2);
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

TEST(PointTest, AngleDiff) {
    Modelec::Point p1(0, 0, 0);
    Modelec::Point p2(0, 0, M_PI / 2);
    double diff = Modelec::Point::angleDiff(p1, p2);
    EXPECT_NEAR(diff, -M_PI_2, 1e-6);
}

TEST(PointTest, GetTakePositionWithAngle) {
    Modelec::Point p(0, 0, 0);
    Modelec::Point result = p.GetTakePosition(10, 3.141592653589793 / 2);
    EXPECT_EQ(result.x, 0.0);
    EXPECT_EQ(result.y, 10.0);
}

TEST(PointTest, GetTakePositionDefaultAngle) {
    Modelec::Point p(1, 1, 0);
    Modelec::Point result = p.GetTakePosition(5);
    EXPECT_NEAR(result.x, 6.0, 1e-6);
    EXPECT_NEAR(result.y, 1.0, 1e-6);
}

TEST(PointTest, GetTakeBasePosition) {
    Modelec::Point p(0, 0, 0);
    Modelec::Point base = p.GetTakeBasePosition();
    EXPECT_EQ(base.x, 290);
    EXPECT_EQ(base.y, 0);
}

TEST(PointTest, GetTakeClosePosition) {
    Modelec::Point p(0, 0, 0);
    Modelec::Point close = p.GetTakeClosePosition();
    EXPECT_EQ(close.x, 165);
    EXPECT_EQ(close.y, 0);
}
