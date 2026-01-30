#include <modelec_com/pcb_odo_interface.hpp>
#include <modelec_utils/utils.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <modelec_utils/config.hpp>

namespace Modelec
{
    PCBOdoInterface::PCBOdoInterface() : Node("pcb_odo_interface")
    {
        declare_parameter<std::string>("serial_port", "/dev/USB_ODO");
        declare_parameter<int>("baudrate", 115200);
        declare_parameter<std::string>("name", "pcb_odo");

        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = get_parameter("name").as_string();
        request->bauds = get_parameter("baudrate").as_int();
        request->serial_port = get_parameter("serial_port").as_string();

        odo_pos_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 10);

        odo_get_pos_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "odometry/get/pos", 30, [this](const std_msgs::msg::Empty::SharedPtr)
            {
                if (IsOk())
                {
                    GetPos();
                }
            });

        odo_speed_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometrySpeed>(
            "odometry/speed", 10);

        odo_tof_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryToF>(
            "odometry/tof", 10);

        odo_waypoint_reach_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/waypoint_reach", 10);

        odo_pid_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPid>(
            "odometry/get_pid", 10);

        odo_add_waypoint_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryWaypoint>(
            "odometry/add_waypoint", 30,
            [this](const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
            {
                AddWaypointCallback(msg);
            });

        odo_add_waypoints_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryWaypoints>(
            "odometry/add_waypoints", 30,
            [this](const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
            {
                AddWaypointsCallback(msg);
            });

        odo_set_pos_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryPos>(
            "odometry/set_position", 10,
            [this](const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
            {
                SetPosCallback(msg);
            });

        odo_set_pid_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryPid>(
            "odometry/set_pid", 10,
            [this](const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
            {
                SetPIDCallback(msg);
            });

        odo_ask_active_waypoint_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
            "odometry/ask_active_waypoint", 10,
            [this](const std_msgs::msg::Empty::SharedPtr)
            {
                GetData("WAYPOINT", {"ACTIVE"});
            });

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 30,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg)
            {
                double left_axis = msg->axes[1];
                double right_axis = msg->axes[2];

                if (fabs(left_axis) < 0.05) left_axis = 0.0;
                if (fabs(right_axis) < 0.05) right_axis = 0.0;

                int left_motor  = static_cast<int>(left_axis * 626 - right_axis * 626);
                int right_motor = static_cast<int>(left_axis * 626 + right_axis * 626);

                // int left_motor  = static_cast<int>(left_axis  * 626);
                // int right_motor = static_cast<int>(right_axis * 626);

                left_motor  = std::max(-626, std::min(626, left_motor));
                right_motor = std::max(-626, std::min(626, right_motor));

                SetMotor(left_motor, right_motor);
            });

        start_odo_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "odometry/start", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                start_odo_ = msg->data;
                SendOrder("START", {std::to_string(msg->data)});
            });

        this->open(request->name, request->bauds, request->serial_port, MAX_MESSAGE_LEN);

        SetPID("THETA", 14, 0, 0);
        SetPID("POS", 5, 0, 0);
        SetPID("LEFT", 5, 0, 0);
        SetPID("RIGHT", 5, 0, 0);
    }

    PCBOdoInterface::~PCBOdoInterface()
    {
        SetStart(false);
    }

    void PCBOdoInterface::read(const std::string& msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received from PCB: %s", trim(msg).c_str());
        RCLCPP_INFO_ONCE(this->get_logger(), "Received from PCB: %s", trim(msg).c_str());
        RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "Received from PCB: %s", msg.c_str());
        std::vector<std::string> tokens = split(trim(msg), ';');
        if (tokens.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid message format: %s", trim(msg).c_str());
            return;
        }

        if (tokens[0] == "SET")
        {
            if (tokens[1] == "POS")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                double theta = std::stod(tokens[4]);

                auto message = modelec_interfaces::msg::OdometryPos();
                message.x = x;
                message.y = y;
                message.theta = theta;

                if (odo_pos_publisher_) {
                    odo_pos_publisher_->publish(message);
                }
            }
            else if (tokens[1] == "SPEED")
            {
                long x = std::stol(tokens[2]);
                long y = std::stol(tokens[3]);
                double theta = std::stod(tokens[4]);

                auto message = modelec_interfaces::msg::OdometrySpeed();
                message.x = x;
                message.y = y;
                message.theta = theta;

                odo_speed_publisher_->publish(message);
            }
            else if (tokens[1] == "DIST")
            {
                int n = std::stoi(tokens[2]);
                long dist = std::stol(tokens[3]);

                auto message = modelec_interfaces::msg::OdometryToF();
                message.n = n;
                message.distance = dist;

                odo_tof_publisher_->publish(message);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                if (tokens[2] == "REACH")
                {
                    RCLCPP_DEBUG(this->get_logger(), "Waypoint reached: ID %s", tokens[3].c_str());

                    int id = std::stoi(tokens[3]);

                    auto message = modelec_interfaces::msg::OdometryWaypoint();
                    message.id = id;

                    odo_waypoint_reach_publisher_->publish(message);
                }
                else if (tokens.size() >= 7)
                {
                    int id = std::stoi(tokens[3]);
                    bool is_end = tokens[4] == "1";
                    long x = std::stol(tokens[5]);
                    long y = std::stol(tokens[6]);
                    double theta = std::stod(tokens[7]);
                    bool reach = tokens.size() > 7 ? tokens[8] == "1" : false;

                    if (reach)
                    {
                        auto message = modelec_interfaces::msg::OdometryWaypoint();
                        message.id = id;
                        message.is_end = is_end;
                        message.x = x;
                        message.y = y;
                        message.theta = theta;

                        odo_waypoint_reach_publisher_->publish(message);
                    }
                }
            }
            else if (tokens[1] == "PID")
            {
                std::string name = tokens[2];
                float p = std::stof(tokens[3]);
                float i = std::stof(tokens[4]);
                float d = std::stof(tokens[5]);

                auto message = modelec_interfaces::msg::OdometryPid();
                message.name = name;
                message.p = p;
                message.i = i;
                message.d = d;

                odo_pid_publisher_->publish(message);
            }
        }
        else if (tokens[0] == "OK")
        {
            if (tokens[1] == "START")
            {
                start_odo_ = tokens[2] == "1";
            }
            else if (tokens[1] == "WAYPOINT")
            {
                // RCLCPP_INFO(this->get_logger(), "Waypoint added successfully.");
            }
            else if (tokens[1] == "PID")
            {
            }
            else if (tokens[1] == "POS")
            {
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "PCB response: %s", trim(msg).c_str());
            }
        }
        else if (tokens[0] == "KO")
        {
            if (tokens[1] == "START")
            {
            }
            else if (tokens[1] == "WAYPOINT")
            {
            }

            else if (tokens[1] == "PID")
            {
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "PCB error: %s", trim(msg).c_str());
            }
        }
    }

    void PCBOdoInterface::AddWaypointsCallback(const modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg) {
        AddWaypoints(msg);
    }

    void PCBOdoInterface::AddWaypointCallback(const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
    {
        AddWaypoint(msg);
    }

    void PCBOdoInterface::SetPosCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        SetRobotPos(msg);
    }

    void PCBOdoInterface::SetPIDCallback(const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
    {
        SetPID(msg);
    }

    void PCBOdoInterface::SendToPCB(const std::string& data)
    {
        if (IsOk())
        {
            // RCLCPP_INFO(this->get_logger(), "Sending to PCB: %s", trim(data).c_str());
            RCLCPP_INFO_ONCE(this->get_logger(), "Sending to PCB: %s", trim(data).c_str());
            RCLCPP_DEBUG_SKIPFIRST(this->get_logger(), "Sending to PCB: %s", data.c_str());
            this->write(data);
        }
    }

    void PCBOdoInterface::SendToPCB(const std::string& order, const std::string& elem,
                                    const std::vector<std::string>& data)
    {
        std::string command = order + ";" + elem;
        for (const auto& d : data)
        {
            command += ";" + d;
        }
        command += "\n";

        SendToPCB(command);
    }

    void PCBOdoInterface::GetData(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("GET", elem, data);
    }

    void PCBOdoInterface::SendOrder(const std::string& elem, const std::vector<std::string>& data)
    {
        SendToPCB("SET", elem, data);
    }

    void PCBOdoInterface::GetPos()
    {
        GetData("POS");
    }

    void PCBOdoInterface::GetSpeed()
    {
        GetData("SPEED");
    }

    void PCBOdoInterface::GetToF(const int& tof)
    {
        GetData("DIST", {std::to_string(tof)});
    }

    void PCBOdoInterface::SetRobotPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg)
    {
        SetRobotPos(msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::SetRobotPos(const long x, const long y, const double theta)
    {
        std::vector<std::string> data = {
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("POS", data);
    }

    void PCBOdoInterface::AddWaypoints(modelec_interfaces::msg::OdometryWaypoints::SharedPtr msg)
    {
        if (!start_odo_)
        {
            SendOrder("START", {std::to_string(true)});
        }

        std::vector<std::string> data;

        for (const auto& wp : msg->waypoints)
        {
            data.push_back(std::to_string(wp.id));
            data.push_back(std::to_string(wp.is_end));
            data.push_back(std::to_string(wp.x));
            data.push_back(std::to_string(wp.y));
            data.push_back(std::to_string(wp.theta));
       }

        SendOrder("WAYPOINT", data);
    }

    void PCBOdoInterface::AddWaypoint(
        const modelec_interfaces::msg::OdometryWaypoint::SharedPtr msg)
    {
        AddWaypoint(msg->id, msg->is_end, msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::AddWaypoint(const int index, const bool IsStopPoint, const long x, const long y,
                                      const double theta)
    {
        if (!start_odo_)
        {
            SendOrder("START", {std::to_string(true)});
        }

        std::vector<std::string> data = {
            std::to_string(index),
            std::to_string(IsStopPoint),
            std::to_string(x),
            std::to_string(y),
            std::to_string(theta)
        };

        SendOrder("WAYPOINT", data);
    }

    void PCBOdoInterface::SetStart(const modelec_interfaces::msg::OdometryStart::SharedPtr msg)
    {
        SetStart(msg->start);
    }

    void PCBOdoInterface::SetStart(bool start)
    {
        SendOrder("START", {std::to_string(start)});
    }

    void PCBOdoInterface::GetPID()
    {
        GetData("PID");
    }

    void PCBOdoInterface::SetPID(const modelec_interfaces::msg::OdometryPid::SharedPtr msg)
    {
        SetPID(msg->name, msg->p, msg->i, msg->d);
    }

    void PCBOdoInterface::SetPID(std::string name, float p, float i, float d, std::optional<float> min, std::optional<float> max)
    {
        std::vector<std::string> data = {
            name,
            std::to_string(p),
            std::to_string(i),
            std::to_string(d)
        };

        if (min.has_value())
        {
            data.push_back(std::to_string(min.value()));
        }

        if (max.has_value())
        {
            data.push_back(std::to_string(max.value()));
        }

        SendOrder("PID", data);
    }

    void PCBOdoInterface::SetMotor(int left, int right)
    {
        std::vector<std::string> data = {
            std::to_string(left),
            std::to_string(right)
        };

        SendOrder("MOTOR", data);
    }
} // Modelec

#ifndef MODELEC_COM_TESTING
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::PCBOdoInterface>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#endif