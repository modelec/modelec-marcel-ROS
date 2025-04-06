#include <modelec/pcb_odo_interface.h>
#include <modelec/utils.hpp>
#include <modelec_interface/srv/add_serial_listener.hpp>

namespace Modelec
{
    PCBOdoInterface::PCBOdoInterface() : Node("pcb_odo_interface")
    {
        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interface::srv::AddSerialListener::Request>();
        request->name = "pcb_odo";
        request->bauds = 115200;
        request->serial_port = "/dev/serial1"; // TODO : check the real serial port
        auto client = this->create_client<modelec_interface::srv::AddSerialListener>("add_serial_listener");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (!result.get()->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Added serial listener");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(result.get()->subscriber, 10);
        pcb_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            result.get()->publisher, 10,
            std::bind(&PCBOdoInterface::PCBCallback, this, std::placeholders::_1));

        odo_pos_publisher_ = this->create_publisher<modelec_interface::msg::OdometryPos>(
            "odometry/position", 10);

        odo_speed_publisher_ = this->create_publisher<modelec_interface::msg::OdometrySpeed>(
            "odometry/speed", 10);

        odo_tof_publisher_ = this->create_publisher<modelec_interface::msg::OdometryToF>(
            "odometry/tof", 10);

        odo_waypoint_reach_publisher_ = this->create_publisher<modelec_interface::msg::OdometryWaypointReach>(
            "odometry/waypoint-reach", 10);

        odo_add_waypoint_subscriber_ = this->create_subscription<modelec_interface::msg::OdometryAddWaypoint>(
            "odometry/add-waypoint", 10,
            std::bind(&PCBOdoInterface::AddWaypointCallback, this, std::placeholders::_1));

        odo_set_pos_subscriber_ = this->create_subscription<modelec_interface::msg::OdometryPos>(
            "odometry/set-position", 10,
            std::bind(&PCBOdoInterface::SetPosCallback, this, std::placeholders::_1));

        // Services
        get_tof_service_ = create_service<modelec_interface::srv::OdometryToF>(
            "odometry/tof",
            std::bind(&PCBOdoInterface::HandleGetTof, this, std::placeholders::_1, std::placeholders::_2));

        get_speed_service_ = create_service<modelec_interface::srv::OdometrySpeed>(
            "odometry/speed",
            std::bind(&PCBOdoInterface::HandleGetSpeed, this, std::placeholders::_1, std::placeholders::_2));

        get_position_service_ = create_service<modelec_interface::srv::OdometryPosition>(
            "odometry/position",
            std::bind(&PCBOdoInterface::HandleGetPosition, this, std::placeholders::_1, std::placeholders::_2));
    }

    void PCBOdoInterface::PCBCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<std::string> tokens = split(msg->data, ';');
        if (tokens.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid message format: %s", msg->data.c_str());
            return;
        }

        if (tokens[0] == "SET")
        {
            if (tokens[1] == "POS")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                long theta = std::stol(tokens[4]);

                auto message = modelec_interface::msg::OdometryPos();
                message.x = x;
                message.y = y;
                message.theta = theta;

                odo_pos_publisher_->publish(message);
                ResolvePositionRequest({ x, y, theta });
            }
            else if (tokens[1] == "SPEED")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                long theta = std::stol(tokens[4]);

                auto message = modelec_interface::msg::OdometrySpeed();
                message.x = x;
                message.y = y;
                message.theta = theta;

                odo_speed_publisher_->publish(message);
                ResolveSpeedRequest({ x, y, theta });
            }
            else if (tokens[1] == "DIST")
            {
                int n = std::stoi(tokens[2]);
                long dist = std::stol(tokens[3]);

                auto message = modelec_interface::msg::OdometryToF();
                message.n = n;
                message.distance = dist;

                odo_tof_publisher_->publish(message);
                ResolveToFRequest(dist);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                int id = std::stoi(tokens[2]);

                auto message = modelec_interface::msg::OdometryWaypointReach();
                message.id = id;

                odo_waypoint_reach_publisher_->publish(message);
            }
        }
        else if (tokens[0] == "OK")
        {
            RCLCPP_INFO(this->get_logger(), "PCB response: %s", msg->data.c_str());
        }
        else if (tokens[0] == "KO")
        {
            RCLCPP_ERROR(this->get_logger(), "PCB error: %s", msg->data.c_str());
        }
    }

    void PCBOdoInterface::AddWaypointCallback(const modelec_interface::msg::OdometryAddWaypoint::SharedPtr msg) const
    {
        AddWaypoint(msg);
    }

    void PCBOdoInterface::SetPosCallback(const modelec_interface::msg::OdometryPos::SharedPtr msg) const
    {
        SetRobotPos(msg);
    }

    void PCBOdoInterface::HandleGetTof(const std::shared_ptr<modelec_interface::srv::OdometryToF::Request> request,
                                       std::shared_ptr<modelec_interface::srv::OdometryToF::Response> response)
    {
        std::promise<long> promise;
        auto future = promise.get_future();
        {
            std::lock_guard<std::mutex> lock(tof_mutex_);
            tof_promises_.push(std::move(promise));
        }

        GetToF(request->n);

        response->distance = future.get();
    }

    void PCBOdoInterface::HandleGetSpeed(const std::shared_ptr<modelec_interface::srv::OdometrySpeed::Request>,
        std::shared_ptr<modelec_interface::srv::OdometrySpeed::Response> response)
    {
        std::promise<OdometryData> promise;
        auto future = promise.get_future();
        {
            std::lock_guard<std::mutex> lock(speed_mutex_);
            speed_promises_.push(std::move(promise));
        }

        GetSpeed();

        OdometryData result = future.get();

        response->x = result.x;
        response->y = result.y;
        response->theta = result.theta;
    }

    void PCBOdoInterface::HandleGetPosition(
        const std::shared_ptr<modelec_interface::srv::OdometryPosition::Request>,
        std::shared_ptr<modelec_interface::srv::OdometryPosition::Response> response)
    {
        std::promise<OdometryData> promise;
        auto future = promise.get_future();
        {
            std::lock_guard<std::mutex> lock(pos_mutex_);
            pos_promises_.push(std::move(promise));
        }

        GetPos();

        OdometryData result = future.get();

        response->x = result.x;
        response->y = result.y;
        response->theta = result.theta;
    }

    void PCBOdoInterface::ResolveToFRequest(const long distance)
    {
        std::lock_guard<std::mutex> lock(tof_mutex_);
        if (!tof_promises_.empty())
        {
            std::promise<long> promise = std::move(tof_promises_.front());
            tof_promises_.pop();
            promise.set_value(distance);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received ToF response but no promise waiting");
        }
    }

    void PCBOdoInterface::ResolveSpeedRequest(const OdometryData& speed)
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        if (!speed_promises_.empty())
        {
            std::promise<OdometryData> promise = std::move(speed_promises_.front());
            speed_promises_.pop();
            promise.set_value(speed);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received Speed response but no promise waiting");
        }
    }

    void PCBOdoInterface::ResolvePositionRequest(const OdometryData& position)
    {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        if (!pos_promises_.empty())
        {
            std::promise<OdometryData> promise = std::move(pos_promises_.front());
            pos_promises_.pop();
            promise.set_value(position);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Received Position response but no promise waiting");
        }
    }

    void PCBOdoInterface::SendToPCB(const std::string& data) const
    {
        auto message = std_msgs::msg::String();
        message.data = data;
        pcb_publisher_->publish(message);
    }

    void PCBOdoInterface::SendToPCB(const std::string& order, const std::string& elem,
        const std::vector<std::string>& data) const
    {
        std::string command = order + ";" + elem;
        for (const auto& d : data)
        {
            command += ";" + d;
        }

        SendToPCB(command);
    }

    void PCBOdoInterface::GetData(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("GET", elem, data);
    }

    void PCBOdoInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data) const
    {
        SendToPCB("SET", elem, data);
    }

    void PCBOdoInterface::GetPos() const
    {
        GetData("POS");
    }

    void PCBOdoInterface::GetSpeed() const
    {
        GetData("SPEED");
    }

    void PCBOdoInterface::GetToF(const int& tof) const
    {
        GetData("DIST", { std::to_string(tof) });
    }

    void PCBOdoInterface::SetRobotPos(const modelec_interface::msg::OdometryPos::SharedPtr msg) const
    {
        SetRobotPos(msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::SetRobotPos(const long x, const long y, const long theta) const
    {
        std::vector<std::string> data = {
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("POS", data);
    }

    void PCBOdoInterface::AddWaypoint(
        const modelec_interface::msg::OdometryAddWaypoint::SharedPtr msg) const
    {
        AddWaypoint(msg->id, msg->is_end, msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::AddWaypoint(const int index, const bool IsStopPoint, const long x, const long y, const long theta) const
    {
        std::vector<std::string> data = {
            std::to_string(index),
            std::to_string(IsStopPoint),
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("WAYPOINT", data);
    }
} // Modelec


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Modelec::PCBOdoInterface>());
    rclcpp::shutdown();
    return 0;
}
