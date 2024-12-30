#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

class ForceApplier : public rclcpp::Node
{
public:
  ForceApplier() : Node("force_applier")
  {
    force_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/apply_force", 10);

    auto timer_callback = [this]() -> void {
      auto wrench_msg = geometry_msgs::msg::Wrench();
      wrench_msg.force.x = 0.0;  // force in the x-direction
      wrench_msg.force.y =  -1110.0;
      wrench_msg.force.z = 0.0;
      wrench_msg.torque.x = 0.0;
      wrench_msg.torque.y = 0.0;
      wrench_msg.torque.z = 0.0;

      force_pub_->publish(wrench_msg);
    };

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceApplier>());
  rclcpp::shutdown();
  return 0;
}


