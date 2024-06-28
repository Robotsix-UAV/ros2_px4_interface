#include <ros2_uav_parameters/parameter_client.hpp>
#include <ros2_uav_cpp/ros2_logger.hpp>
#include "ros2_uav_px4/modes/spin.hpp"
#include "ros2_uav_px4/modes/position.hpp"

using uav_cpp::parameters::ParameterMap;
using ros2_uav::utils::RosLoggerInterface;
using ros2_uav::modes::Spin;
using ros2_uav::modes::Position;
using ros2_uav::modes::ExecutorSpin;
using ros2_uav::modes::ExecutorPosition;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto mode_node = std::make_shared<rclcpp::Node>("mode_node");
  auto spin_mode = std::make_unique<Spin>(*mode_node);
  auto spin_executor = std::make_unique<ExecutorSpin>(*mode_node, *spin_mode);

  std::vector<std::string> required_parameters_spin;
  spin_mode->getRequiredParameters(required_parameters_spin);

  auto position_mode = std::make_unique<Position>(*mode_node);
  auto position_executor = std::make_unique<ExecutorPosition>(*mode_node, *position_mode);

  std::vector<std::string> required_parameters_position;
  position_mode->getRequiredParameters(required_parameters_position);

  std::vector<std::string> required_parameters;
  required_parameters.insert(
    required_parameters.end(), required_parameters_spin.begin(), required_parameters_spin.end());
  required_parameters.insert(
    required_parameters.end(),
    required_parameters_position.begin(), required_parameters_position.end());
  std::sort(required_parameters.begin(), required_parameters.end());

  auto parameter_client = std::make_shared<ros2_uav::parameters::ParameterClient>(
    "mode_parameter_client", required_parameters);
  auto parameters = std::make_shared<ParameterMap>(parameter_client->getParameters());
  executor.add_node(parameter_client);

  spin_mode->setParameters(parameters);
  position_mode->setParameters(parameters);
  executor.add_node(mode_node);

  // Set the logger to node logger for the uav_cpp library
  auto uav_cpp_logger = rclcpp::get_logger("uav_cpp");
  auto ret = rcutils_logging_set_logger_level("uav_cpp", RCUTILS_LOG_SEVERITY_DEBUG);
  (void) ret;
  auto logger = std::make_shared<RosLoggerInterface>(uav_cpp_logger);
  uav_cpp::logger::Logger::setCustomLogger(logger);

  spin_executor->doRegister();
  position_executor->doRegister();

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
