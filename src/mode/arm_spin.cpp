#include "ros2_px4_interface/modes/arm_spin.hpp"

using std::chrono_literals::operator""s;

namespace rpi::mode
{
using rcl_interfaces::msg::ParameterDescriptor;

ArmSpin::ArmSpin(rclcpp::Node & node)
: ModeBasePublisher(node, Settings("Arm and Spin", true)), node_(node), spin_thrust_(node)
{
  auto spin_param = ParameterDescriptor();
  spin_param.description = "Spinning thrust value";
  spin_param.set__additional_constraints("Value between 0.0 and 1.0");
  rcl_interfaces::msg::FloatingPointRange float_range;
  float_range.from_value = 0.0;
  float_range.to_value = 1.0;
  spin_param.set__floating_point_range({float_range});

  spin_thrust_.createParameter("spin_thrust", 0.05f, spin_param);

  attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
}

void ArmSpin::updateSetpoint([[maybe_unused]] float dt)
{
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -spin_thrust_};
  // TODO: Get initial odometry and set the attitude to that
  const Eigen::Quaternionf attitude = Eigen::Quaternionf::Identity();
  attitude_setpoint_->update(attitude, thrust_sp);
}

ExecutorArmSpin::ExecutorArmSpin(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
: rpi::mode::ExecutorBasePublisher(node, Settings{true}, owned_mode)
{
}

void ExecutorArmSpin::onActivate()
{
  RCLCPP_DEBUG(node().get_logger(), "ExecutorArmSpin activated.");
  runState(State::ARM);
}

void ExecutorArmSpin::runState(State state)
{
  switch (state) {
    case State::ARM:
      // Arm the vehicle
      arm(
        [this](px4_ros2::Result result)
        {
          if (result == px4_ros2::Result::Success) {
            runState(State::SPIN);
          } else {
            // TODO: publish failure
          }
        });
      break;

    case State::SPIN:
      scheduleMode(
        ownedMode().id(), [this](px4_ros2::Result) {this->});
      break;
  }
}

} /* namespace rpi::mode */
