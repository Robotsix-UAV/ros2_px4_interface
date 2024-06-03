#include <std_msgs/msg/bool.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>

namespace rpi::mode
{
class ModeBasePublisher : public px4_ros2::ModeBase,
  public std::enable_shared_from_this<ModeBasePublisher>
{
public:
  explicit ModeBasePublisher(rclcpp::Node & node, Settings settings);

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr activation_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

class ExecutorBasePublisher : public px4_ros2::ModeExecutorBase
{
public:
  explicit ExecutorBasePublisher(
    rclcpp::Node & node, Settings settings,
    px4_ros2::ModeBase & owned_mode);
};
} /* namespace rpi::mode */
