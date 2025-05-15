#pragma once
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/obstacle.hpp>

#include <tinyxml2.h>

#include <modelec_utils/point.hpp>

namespace Modelec
{
    class Obstacle
    {
    public:
        Obstacle() : id_(-1), x_(0), y_(0), w_(0), h_(0), theta_(0), type_("unknown")
        {
        }

        Obstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        Obstacle(tinyxml2::XMLElement* obstacleElem);
        Obstacle(const modelec_interfaces::msg::Obstacle& msg);
        Obstacle(const Obstacle& other) = default;

        virtual ~Obstacle() = default;

        virtual modelec_interfaces::msg::Obstacle toMsg() const;

        int GetId() const { return id_; }
        int GetX() const { return x_; }
        int GetY() const { return y_; }
        double GetTheta() const { return theta_; }
        int GetWidth() const { return w_; }
        int GetHeight() const { return h_; }
        const std::string& Type() const { return type_; }
        Point GetPosition() const { return Point(x_, y_, theta_); }

        void SetId(int id) { id_ = id; }
        void SetX(int x) { x_ = x; }
        void SetY(int y) { y_ = y; }
        void SetTheta(double theta) { theta_ = theta; }
        void SetWidth(int w) { w_ = w; }
        void SetHeight(int h) { h_ = h; }
        void SetType(const std::string& type) { type_ = type; }

        void SetPosition(int x, int y, double theta)
        {
            x_ = x;
            y_ = y;
            theta_ = theta;
        }

        void SetPosition(const Point& p)
        {
            x_ = p.x;
            y_ = p.y;
            theta_ = p.theta;
        }

        void SetSize(int w, int h)
        {
            w_ = w;
            h_ = h;
        }

    protected:
        int id_, x_, y_, w_, h_;
        double theta_;
        std::string type_;
    };
}
