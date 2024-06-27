#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

namespace ros2_uav
{

using uav_cpp::parameters::ParameterMap;

template<typename ModeT>
class ParamNodeWithMode : public rclcpp::Node
{
  static_assert(
    std::is_base_of<px4_ros2::ModeBase, ModeT>::value,
    "Template type ModeT must be derived from px4_ros2::ModeBase");

public:
  explicit ParamNodeWithMode(std::string node_name, std::shared_ptr<ParameterMap> parameters,bool enable_debug_output = false)
  : Node(node_name)
  {
    if (enable_debug_output) {
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

    _mode = std::make_unique<ModeT>(*this, parameters);

    if (!_mode->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

  ModeT & getMode() const
  {
    return *_mode;
  }

private:
  std::unique_ptr<ModeT> _mode;
};

template<typename ModeExecutorT, typename ModeT>
class ParamNodeWithModeExecutor : public rclcpp::Node
{
  static_assert(
    std::is_base_of<px4_ros2::ModeExecutorBase, ModeExecutorT>::value,
    "Template type ModeExecutorT must be derived from px4_ros2::ModeExecutorBase");
  static_assert(
    std::is_base_of<px4_ros2::ModeBase, ModeT>::value,
    "Template type ModeT must be derived from px4_ros2::ModeBase");

public:
  explicit ParamNodeWithModeExecutor(std::string node_name, std::shared_ptr<ParameterMap> parameters, bool enable_debug_output = false)
  : Node(node_name)
  {
    if (enable_debug_output) {
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

    _mode = std::make_unique<ModeT>(*this, parameters);
    _mode_executor = std::make_unique<ModeExecutorT>(*this, *_mode);

    RCLCPP_DEBUG(get_logger(), "Registering mode executor");
    if (!_mode_executor->doRegister()) {
      throw std::runtime_error("Registration failed");
    }
  }

  ModeT & getMode() const
  {
    return *_mode;
  }

private:
  std::unique_ptr<ModeExecutorT> _mode_executor;
  std::unique_ptr<ModeT> _mode;
};

}  // namespace ros2_uav
