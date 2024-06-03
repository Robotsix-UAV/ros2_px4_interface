#include <px4_ros2/components/node_with_mode.hpp>
#include <ros2_px4_interface/modes/arm_spin.hpp>

using MyNodeWithMode = px4_ros2::NodeWithModeExecutor<rpi::mode::ExecutorArmSpin,
    rpi::mode::ArmSpin>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>("mode_arm_spin", true));
  rclcpp::shutdown();
  return 0;
}
