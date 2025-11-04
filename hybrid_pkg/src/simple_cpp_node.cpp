#include "rclcpp/rclcpp.hpp"

class SimpleCppNode : public rclcpp::Node
{
public:
  SimpleCppNode() : Node("simple_cpp_node")
  {
    RCLCPP_INFO(this->get_logger(), "C++ Node started!");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { RCLCPP_INFO(this->get_logger(), "Hello from C++ node"); });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleCppNode>());
  rclcpp::shutdown();
  return 0;
}
