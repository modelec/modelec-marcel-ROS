#include <modelec_strat/obstacle/obstacle.hpp>

namespace Modelec
{
    Obstacle::Obstacle(int id, int x, int y, double theta, int w, int h, const std::string& type)
        : id_(id), x_(x), y_(y), w_(w), h_(h), theta_(theta), type_(type)
    {
    }

    Obstacle::Obstacle(tinyxml2::XMLElement* obstacleElem)
    {
        const char* type = nullptr;
        if (obstacleElem->QueryIntAttribute("id", &id_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryIntAttribute("x", &x_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryIntAttribute("y", &y_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryDoubleAttribute("theta", &theta_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryIntAttribute("width", &w_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryIntAttribute("height", &h_) != tinyxml2::XML_SUCCESS ||
            obstacleElem->QueryStringAttribute("type", &type) != tinyxml2::XML_SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("Obstacle"), "Failed to parse obstacle element");
            return;
        }
        type_ = type;
    }

    Obstacle::Obstacle(const modelec_interfaces::msg::Obstacle& msg)
        : id_(msg.id), x_(msg.x), y_(msg.y), w_(msg.width), h_(msg.height), theta_(msg.theta), type_("unknown")
    {
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

        return msg;
    }
}
