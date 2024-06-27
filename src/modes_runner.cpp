#include <ros2_uav_px4/modes/spin.hpp>
#include <ros2_uav_parameters/parameter_client.hpp>
#include <ros2_uav_cpp/ros2_logger.hpp>
#include "ros2_uav_px4/param_node_with_mode.hpp"

using uav_cpp::parameters::ParameterMap;
using ros2_uav::utils::RosLoggerInterface;
using SpinMode = ros2_uav::ParamNodeWithModeExecutor<ros2_uav::modes::ExecutorSpin, ros2_uav::modes::Spin>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> required_parameters{"spin.thrust"};
  auto parameter_client_ = std::make_shared<ros2_uav::parameters::ParameterClient>("spin_parameters", required_parameters);
  auto parameters = std::make_shared<ParameterMap>(parameter_client_->getParameters());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parameter_client_);
  auto mode_executor = std::make_shared<SpinMode>("spin_mode", parameters, true);
  executor.add_node(mode_executor);

  // Set the logger to node logger for the uav_cpp library
  auto logger = std::make_shared<RosLoggerInterface>(mode_executor->get_logger());
  uav_cpp::logger::Logger::setCustomLogger(logger);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
