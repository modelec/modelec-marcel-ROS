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
        Obstacle(const modelec_interfaces::msg::Obstacle& msg);
        Obstacle(int id);
        Obstacle(const Obstacle& other) = default;

        virtual ~Obstacle() = default;

        virtual modelec_interfaces::msg::Obstacle toMsg() const;

        int GetId() const;
        int GetX() const;
        int GetY() const;
        double GetTheta() const;
        int GetWidth() const;
        int GetHeight() const;
        const std::string& GetType() const;
        Point GetPosition() const;

        void SetId(int id);
        void SetX(int x);
        void SetY(int y);
        void SetTheta(double theta);
        void SetWidth(int w);
        void SetHeight(int h);
        void SetType(const std::string& type);

        void SetPosition(int x, int y, double theta);

        void SetPosition(const Point& p);

        void SetSize(int w, int h);

        bool IsAtObjective() const;
        void SetAtObjective(bool atObjective);

    protected:
        int id_, x_, y_, w_, h_;
        double theta_;
        std::string type_;

        bool isAtObjective = false;
    };
}
