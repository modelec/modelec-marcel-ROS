#include <modelec_strat/obstacle/obstacle.hpp>

#include "modelec_utils/config.hpp"

namespace Modelec
{
    Obstacle::Obstacle(int id, int x, int y, double theta, int w, int h, const std::string& type)
        : id_(id), x_(x), y_(y), w_(w), h_(h), theta_(theta), type_(type)
    {
    }

    Obstacle::Obstacle(const modelec_interfaces::msg::Obstacle& msg)
        : id_(msg.id), x_(msg.x), y_(msg.y), w_(msg.width), h_(msg.height), theta_(msg.theta), type_("unknown")
    {
    }

    Obstacle::Obstacle(int id)
    {
        id_ = Config::get<int>("Obstacles.Obstacle[" + std::to_string(id) + "]@id", id);
        x_ = Config::get<int>("Obstacles.Obstacle[" + std::to_string(id) + "]@x");
        y_ = Config::get<int>("Obstacles.Obstacle[" + std::to_string(id) + "]@y");
        theta_ = Config::get<double>("Obstacles.Obstacle[" + std::to_string(id) + "]@theta");
        w_ = Config::get<int>("Obstacles.Obstacle[" + std::to_string(id) + "]@width");
        h_ = Config::get<int>("Obstacles.Obstacle[" + std::to_string(id) + "]@height");
        type_ = Config::get<std::string>("Obstacles.Obstacle[" + std::to_string(id) + "]@type", "box");
    }

    modelec_interfaces::msg::Obstacle Obstacle::toMsg() const
    {
        modelec_interfaces::msg::Obstacle msg;

        msg.id = id_;
        msg.x = x_;
        msg.y = y_;
        msg.width = w_;
        msg.height = h_;
        msg.theta = theta_;
        msg.type = type_;

        return msg;
    }

    int Obstacle::GetId() const
    { return id_; }

    int Obstacle::GetX() const
    { return x_; }

    int Obstacle::GetY() const
    { return y_; }

    double Obstacle::GetTheta() const
    { return theta_; }

    int Obstacle::GetWidth() const
    { return w_; }

    int Obstacle::GetHeight() const
    { return h_; }

    const std::string& Obstacle::Type() const
    { return type_; }

    Point Obstacle::GetPosition() const
    { return Point(x_, y_, theta_); }

    void Obstacle::SetId(int id)
    { id_ = id; }

    void Obstacle::SetX(int x)
    { x_ = x; }

    void Obstacle::SetY(int y)
    { y_ = y; }

    void Obstacle::SetTheta(double theta)
    { theta_ = theta; }

    void Obstacle::SetWidth(int w)
    { w_ = w; }

    void Obstacle::SetHeight(int h)
    { h_ = h; }

    void Obstacle::SetType(const std::string& type)
    { type_ = type; }

    void Obstacle::SetPosition(int x, int y, double theta)
    {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }

    void Obstacle::SetPosition(const Point& p)
    {
        x_ = p.x;
        y_ = p.y;
        theta_ = p.theta;
    }

    void Obstacle::SetSize(int w, int h)
    {
        w_ = w;
        h_ = h;
    }

    bool Obstacle::IsAtObjective() const
    {
        return isAtObjective;
    }

    void Obstacle::SetAtObjective(bool atObjective)
    {
        isAtObjective = atObjective;
    }
}
