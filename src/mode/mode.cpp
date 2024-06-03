#include "ros2_px4_interface/common/constants.hpp"
#include "ros2_px4_interface/modes/mode.hpp"

namespace rpi::mode
{
ModeBasePublisher::ModeBasePublisher(rclcpp::Node & node, Settings settings)
: px4_ros2::ModeBase(node, settings)
{
  activation_publisher_ = node.create_publisher<std_msgs::msg::Bool>("activation", 1);
  // Spin publish active state at STATUS_RATE
  status_timer_ = node.create_wall_timer(
    std::chrono::duration<double>(1.0 / rpi::constants::STATUS_RATE), [this]()
    {
      std_msgs::msg::Bool msg;
      msg.data = isActive();
      activation_publisher_->publish(msg);
    });
}

ExecutorBasePublisher::ExecutorBasePublisher(
  rclcpp::Node & node, Settings settings,
  px4_ros2::ModeBase & owned_mode)
: px4_ros2::ModeExecutorBase(node, settings, owned_mode)
{
}
} /* namespace rpi::mode */
