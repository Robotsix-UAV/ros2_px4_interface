#pragma once

#include <uav_cpp/modes/spin.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

namespace ros2_uav::mode
{
using uav_cpp::parameters::ParameterMap;

class Spin : public px4_ros2::ModeBase, public uav_cpp::modes::Spin
{
public:
  Spin(rclcpp::Node & node, std::shared_ptr<ParameterMap> parameters);

private:
  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint([[maybe_unused]] float dt) override;
};

class ExecutorArmSpin : public px4_ros2::ModeExecutorBase
{
public:
  enum class State
  {
    ARM,
    SPIN
  };
  explicit ExecutorArmSpin(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode);
  void onActivate() override;
  void onDeactivate(DeactivateReason) override {}
  void runState(State state);
};

} /* namespace rpi::mode */
