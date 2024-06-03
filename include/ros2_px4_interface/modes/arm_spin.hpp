#pragma once

#include "ros2_px4_interface/common/parameters.hpp"
#include "ros2_px4_interface/modes/mode.hpp"
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

namespace rpi::mode
{
using rpi::parameters::AutoRosParameter;

class ArmSpin : public ModeBasePublisher
{
public:
  explicit ArmSpin(rclcpp::Node & node);

private:
  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint([[maybe_unused]] float dt) override;

  rclcpp::Node & node_;
  AutoRosParameter<float> spin_thrust_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
};

class ExecutorArmSpin : public ExecutorBasePublisher
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
