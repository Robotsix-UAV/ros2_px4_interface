#include <px4_ros2/components/node_with_mode.hpp>
#include <ros2_px4_interface/modes/spin.hpp>
#include <ros2_uav_parameters/parameter_client.hpp>

using ArmSpinMode = px4_ros2::NodeWithModeExecutor<ros2_uav::mode::ExecutorArmSpin,
    ros2_uav::mode::Spin>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> required_parameters = {"spin_thrust"};
  auto parameter_client_ = std::make_shared<ros2_uav::parameters::ParameterClient>("spin_parameters", required_parameters);
  auto parameters = parameter_client_->
  rclcpp::spin(std::make_shared<ArmSpinMode>(parameters));
  rclcpp::shutdown();
  return 0;
}
