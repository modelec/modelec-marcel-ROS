#include <modelec_com/pcb_odo_interface.hpp>
#include <modelec_utils/utils.hpp>
#include <modelec_interfaces/srv/add_serial_listener.hpp>

namespace Modelec
{
    PCBOdoInterface::PCBOdoInterface() : Node("pcb_odo_interface")
    {
        // Service to create a new serial listener
        auto request = std::make_shared<modelec_interfaces::srv::AddSerialListener::Request>();
        request->name = "pcb_odo";
        request->bauds = 115200;
        request->serial_port = "/dev/pts/10"; // TODO : check the real serial port
        auto client = this->create_client<modelec_interfaces::srv::AddSerialListener>("add_serial_listener");
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
            if (auto res = result.get())
            {
                if (res->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Publisher: %s", res->publisher.c_str());
                    RCLCPP_INFO(this->get_logger(), "Subscriber: %s", res->subscriber.c_str());

                    pcb_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                    rclcpp::SubscriptionOptions options;
                    options.callback_group = pcb_callback_group_;

                    pcb_subscriber_ = this->create_subscription<std_msgs::msg::String>(
                        res->publisher, 10,
                        [this](const std_msgs::msg::String::SharedPtr msg) {
                            PCBCallback(msg);
                        },
                        options);

                    pcb_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
                    pcb_executor_->add_callback_group(pcb_callback_group_, this->get_node_base_interface());

                    pcb_executor_thread_ = std::thread([this]() {
                        pcb_executor_->spin();
                    });

                    pcb_publisher_ = this->create_publisher<std_msgs::msg::String>(res->subscriber, 10);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add serial listener");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to ask for a serial listener");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        odo_pos_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPos>(
            "odometry/position", 10);

        odo_speed_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometrySpeed>(
            "odometry/speed", 10);

        odo_tof_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryToF>(
            "odometry/tof", 10);

        odo_waypoint_reach_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryWaypointReach>(
            "odometry/waypoint_reach", 10);

        odo_pid_publisher_ = this->create_publisher<modelec_interfaces::msg::OdometryPid>(
            "odometry/get_pid", 10);

        odo_add_waypoint_subscriber_ = this->create_subscription<modelec_interfaces::msg::OdometryAddWaypoint>(
            "odometry/add_waypoint", 30,
            [this](const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg)
            {
                AddWaypointCallback(msg);
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

        // Services
        get_tof_service_ = create_service<modelec_interfaces::srv::OdometryToF>(
            "odometry/tof",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometryToF::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometryToF::Response> response)
            {
                HandleGetTof(request, response);
            });

        get_speed_service_ = create_service<modelec_interfaces::srv::OdometrySpeed>(
            "odometry/speed",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Response> response)
            {
                HandleGetSpeed(request, response);
            });

        get_position_service_ = create_service<modelec_interfaces::srv::OdometryPosition>(
            "odometry/get_position",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Response> response)
            {
                HandleGetPosition(request, response);
            });

        set_start_service_ = create_service<modelec_interfaces::srv::OdometryStart>(
            "odometry/start",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometryStart::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometryStart::Response> response)
            {
                HandleGetStart(request, response);
            });

        get_pid_service_ = create_service<modelec_interfaces::srv::OdometryGetPid>(
            "odometry/get_pid",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Response> response)
            {
                HandleGetPID(request, response);
            });

        set_pid_service_ = create_service<modelec_interfaces::srv::OdometrySetPid>(
            "odometry/set_pid",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Response> response)
            {
                HandleSetPID(request, response);
            });

        add_waypoint_service_ = create_service<modelec_interfaces::srv::OdometryAddWaypoint>(
            "odometry/add_waypoint",
            [this](const std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Request> request,
                std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Response> response)
            {
                HandleAddWaypoint(request, response);
            });
    }

    PCBOdoInterface::~PCBOdoInterface()
    {
        pcb_executor_->cancel();
        if (pcb_executor_thread_.joinable()) {
            pcb_executor_thread_.join();
        }
    }

    void PCBOdoInterface::PCBCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received from PCB: %s", msg->data.c_str());
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
                double theta = std::stod(tokens[4]);

                auto message = modelec_interfaces::msg::OdometryPos();
                message.x = x;
                message.y = y;
                message.theta = theta;

                odo_pos_publisher_->publish(message);

                ResolvePositionRequest({x, y, theta});
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
                ResolveSpeedRequest({x, y, theta});
            }
            else if (tokens[1] == "DIST")
            {
                int n = std::stoi(tokens[2]);
                long dist = std::stol(tokens[3]);

                auto message = modelec_interfaces::msg::OdometryToF();
                message.n = n;
                message.distance = dist;

                odo_tof_publisher_->publish(message);
                ResolveToFRequest(dist);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                int id = std::stoi(tokens[2]);

                auto message = modelec_interfaces::msg::OdometryWaypointReach();
                message.id = id;

                odo_waypoint_reach_publisher_->publish(message);
            }
            else if (tokens[1] == "PID")
            {
                float p = std::stof(tokens[2]);
                float i = std::stof(tokens[3]);
                float d = std::stof(tokens[4]);

                auto message = modelec_interfaces::msg::OdometryPid();
                message.p = p;
                message.i = i;
                message.d = d;

                odo_pid_publisher_->publish(message);
                ResolveGetPIDRequest({p, i, d});
            }
        }
        else if (tokens[0] == "OK")
        {
            if (tokens[1] == "START")
            {
                bool start = std::stoi(tokens[2]);
                ResolveStartRequest(start);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                bool success = true;
                ResolveAddWaypointRequest(success);
            }

            else if (tokens[1] == "PID")
            {
                bool success = true;
                ResolveSetPIDRequest(success);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "PCB response: %s", msg->data.c_str());
            }
        }
        else if (tokens[0] == "KO")
        {
            if (tokens[1] == "START")
            {
                bool start = false;
                ResolveStartRequest(start);
            }
            else if (tokens[1] == "WAYPOINT")
            {
                bool success = false;
                ResolveAddWaypointRequest(success);
            }

            else if (tokens[1] == "PID")
            {
                bool success = false;
                ResolveSetPIDRequest(success);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "PCB error: %s", msg->data.c_str());
            }
        }
    }

    void PCBOdoInterface::AddWaypointCallback(const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) const
    {
        AddWaypoint(msg);
    }

    void PCBOdoInterface::SetPosCallback(const modelec_interfaces::msg::OdometryPos::SharedPtr msg) const
    {
        SetRobotPos(msg);
    }

    void PCBOdoInterface::SetPIDCallback(const modelec_interfaces::msg::OdometryPid::SharedPtr msg) const
    {
        SetPID(msg);
    }

    void PCBOdoInterface::HandleGetTof(
        const std::shared_ptr<modelec_interfaces::srv::OdometryToF::Request> request,
        std::shared_ptr<modelec_interfaces::srv::OdometryToF::Response> response)
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

    void PCBOdoInterface::HandleGetSpeed(
        const std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Request>,
        std::shared_ptr<modelec_interfaces::srv::OdometrySpeed::Response> response)
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
        const std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Request>,
        std::shared_ptr<modelec_interfaces::srv::OdometryPosition::Response> response)
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

    void PCBOdoInterface::HandleGetStart(const std::shared_ptr<modelec_interfaces::srv::OdometryStart::Request> request,
        std::shared_ptr<modelec_interfaces::srv::OdometryStart::Response> response)
    {
        std::promise<bool> promise;
        auto future = promise.get_future();

        {
            std::lock_guard<std::mutex> lock(start_mutex_);
            start_promises_.push(std::move(promise));
        }

        SetStart(request->start);
        response->success = future.get();
    }

    void PCBOdoInterface::HandleGetPID(const std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Request>,
        std::shared_ptr<modelec_interfaces::srv::OdometryGetPid::Response> response)
    {
        std::promise<PIDData> promise;
        auto future = promise.get_future();

        {
            std::lock_guard<std::mutex> lock(get_pid_mutex_);
            get_pid_promises_.push(std::move(promise));
        }

        GetPID();

        PIDData result = future.get();

        response->p = result.p;
        response->i = result.i;
        response->d = result.d;
    }

    void PCBOdoInterface::HandleSetPID(const std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Request> request,
        std::shared_ptr<modelec_interfaces::srv::OdometrySetPid::Response> response)
    {
        std::promise<bool> promise;
        auto future = promise.get_future();

        {
            std::lock_guard<std::mutex> lock(set_pid_mutex_);
            set_pid_promises_.push(std::move(promise));
        }

        SetPID(request->p, request->i, request->d);

        bool result = future.get();
        response->success = result;
    }

    void PCBOdoInterface::HandleAddWaypoint(
        const std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Request> request,
        std::shared_ptr<modelec_interfaces::srv::OdometryAddWaypoint::Response> response)
    {
        std::promise<bool> promise;
        auto future = promise.get_future();

        {
            std::lock_guard<std::mutex> lock(add_waypoint_mutex_);
            add_waypoint_promises_.push(std::move(promise));
        }
        AddWaypoint(request->id, request->is_end, request->x, request->y, request->theta);
        bool result = future.get();
        response->success = result;
    }

    void PCBOdoInterface::ResolveToFRequest(const long distance)
    {
        std::lock_guard<std::mutex> lock(tof_mutex_);
        if (!tof_promises_.empty()) {
            auto promise = std::move(tof_promises_.front());
            tof_promises_.pop();
            promise.set_value(distance);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending ToF request to resolve.");
        }
    }

    void PCBOdoInterface::ResolveSpeedRequest(const OdometryData& speed)
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        if (!speed_promises_.empty()) {
            auto promise = std::move(speed_promises_.front());
            speed_promises_.pop();
            promise.set_value(speed);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending Speed request to resolve.");
        }
    }

    void PCBOdoInterface::ResolvePositionRequest(const OdometryData& position)
    {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        if (!pos_promises_.empty()) {
            auto promise = std::move(pos_promises_.front());
            pos_promises_.pop();
            promise.set_value(position);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending Position request to resolve.");
        }
    }

    void PCBOdoInterface::ResolveStartRequest(bool start)
    {
        std::lock_guard<std::mutex> lock(start_mutex_);
        if (!start_promises_.empty()) {
            auto promise = std::move(start_promises_.front());
            start_promises_.pop();
            promise.set_value(start);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending Start request to resolve.");
        }
    }

    void PCBOdoInterface::ResolveGetPIDRequest(const PIDData& pid)
    {
        std::lock_guard<std::mutex> lock(get_pid_mutex_);
        if (!get_pid_promises_.empty()) {
            auto promise = std::move(get_pid_promises_.front());
            get_pid_promises_.pop();
            promise.set_value(pid);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending PID request to resolve.");
        }
    }

    void PCBOdoInterface::ResolveSetPIDRequest(bool success)
    {
        std::lock_guard<std::mutex> lock(set_pid_mutex_);
        if (!set_pid_promises_.empty()) {
            auto promise = std::move(set_pid_promises_.front());
            set_pid_promises_.pop();
            promise.set_value(success);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending Set PID request to resolve.");
        }
    }

    void PCBOdoInterface::ResolveAddWaypointRequest(bool success)
    {
        std::lock_guard<std::mutex> lock(add_waypoint_mutex_);
        if (!add_waypoint_promises_.empty()) {
            auto promise = std::move(add_waypoint_promises_.front());
            add_waypoint_promises_.pop();
            promise.set_value(success);
        } else {
            RCLCPP_DEBUG(get_logger(), "No pending Add Waypoint request to resolve.");
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
        command += "\n";

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
        GetData("DIST", {std::to_string(tof)});
    }

    void PCBOdoInterface::SetRobotPos(const modelec_interfaces::msg::OdometryPos::SharedPtr msg) const
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
        const modelec_interfaces::msg::OdometryAddWaypoint::SharedPtr msg) const
    {
        AddWaypoint(msg->id, msg->is_end, msg->x, msg->y, msg->theta);
    }

    void PCBOdoInterface::AddWaypoint(const int index, const bool IsStopPoint, const long x, const long y,
                                      const double theta) const
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

    void PCBOdoInterface::SetStart(const modelec_interfaces::msg::OdometryStart::SharedPtr msg) const
    {
        SetStart(msg->start);
    }

    void PCBOdoInterface::SetStart(bool start) const
    {
        SendOrder("START", {std::to_string(start)});
    }

    void PCBOdoInterface::GetPID() const
    {
        GetData("PID");
    }

    void PCBOdoInterface::SetPID(const modelec_interfaces::msg::OdometryPid::SharedPtr msg) const
    {
        SetPID(msg->p, msg->i, msg->d);
    }

    void PCBOdoInterface::SetPID(float p, float i, float d) const
    {
        std::vector<std::string> data = {
            std::to_string(p),
            std::to_string(i),
            std::to_string(d)
        };

        SendOrder("PID", data);
    }

} // Modelec

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Modelec::PCBOdoInterface>();

    // Increase number of threads explicitly!
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2 /* or more threads! */);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
