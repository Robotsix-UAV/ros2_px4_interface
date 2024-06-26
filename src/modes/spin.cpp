#include "ros2_uav_px4/modes/arm_spin.hpp"

namespace ros2_uav::mode
{
using uav_cpp::controllers::ControlInputs;

Spin::Spin(rclcpp::Node & node, std::shared_ptr<ParameterMap> parameters)
: px4_ros2::ModeBase("Spin", node, true),
  uav_cpp::modes::Spin(parameters)
{}

void Spin::updateSetpoint([[maybe_unused]] float dt)
{
  ControlInputs control_inputs;
  double time = node_->now().seconds();
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -spin_thrust_};
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
