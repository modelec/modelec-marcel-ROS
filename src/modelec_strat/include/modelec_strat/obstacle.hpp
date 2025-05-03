#pragma once
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <modelec_interfaces/msg/obstacle.hpp>

#include <tinyxml2.h>

namespace Modelec {
    class Obstacle {
    public:
        Obstacle() : id_(-1), x_(0), y_(0), w_(0), h_(0), theta_(0), type_("unknown") {}
        Obstacle(int id, int x, int y, double theta, int w, int h, const std::string& type);
        Obstacle(tinyxml2::XMLElement * obstacleElem);
        Obstacle(const modelec_interfaces::msg::Obstacle& msg);
        Obstacle(const Obstacle& other) = default;

        modelec_interfaces::msg::Obstacle toMsg() const;

        int id() const { return id_; }
        int x() const { return x_; }
        int y() const { return y_; }
        double theta() const { return theta_; }
        int width() const { return w_; }
        int height() const { return h_; }
        const std::string& type() const { return type_; }

        void setId(int id) { id_ = id; }
        void setX(int x) { x_ = x; }
        void setY(int y) { y_ = y; }
        void setTheta(double theta) { theta_ = theta; }
        void setWidth(int w) { w_ = w; }
        void setHeight(int h) { h_ = h; }
        void setType(const std::string& type) { type_ = type; }

        void setPosition(int x, int y, double theta) {
            x_ = x;
            y_ = y;
            theta_ = theta;
        }

        void setSize(int w, int h) {
            w_ = w;
            h_ = h;
        }
    private:
        int id_, x_, y_, w_, h_;
        double theta_;
        std::string type_;
    };
}
