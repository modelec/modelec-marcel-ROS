#include <modelec/speed_result.hpp>

namespace Modelec {
  SpeedResultNode::SpeedResultNode() : Node("SpeedResult"), fileName_("speed_data.csv"), file_(fileName_)
  {
    if (!file_.is_open()) {
      std::cerr << "Error opening file: " << fileName_ << std::endl;
    }

    sub_speed_ = this->create_subscription<modelec_interface::msg::OdometrySpeed>(
      "odometry/speed", 10,
      std::bind(&SpeedResultNode::SpeedCallback, this, std::placeholders::_1));

    start_time_ = this->now();
    file_ << "time,x,y,theta" << std::endl;  // Write header to the file
  }

  SpeedResultNode::~SpeedResultNode()
  {
    file_.close();
  }

  void SpeedResultNode::SpeedCallback(const modelec_interface::msg::OdometrySpeed::SharedPtr msg)
  {
    std::string time = std::to_string(this->now().nanoseconds() - start_time_.nanoseconds());
    file_ << time << ","
          << msg->x << "," << msg->y << "," << msg->theta << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Modelec::SpeedResultNode>());
  rclcpp::shutdown();
  return 0;
}
