#include "ros2_uav_px4/modes/attitude_thrust.hpp"

namespace ros2_uav::modes
{
using uav_cpp::controllers::ControlInputs;

    AttitudeThrust::AttitudeThrust(std::string mode_name, rclcpp::Node &node, std::shared_ptr<uav_cpp::modes::Mode> mode)
: px4_ros2::ModeBase(node, Settings{mode_name, true}),
  node_(node), mode_(mode)
{
  attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  time_init_ = node_.now();
}

void AttitudeThrust::updateSetpoint([[maybe_unused]] float dt)
{
  ControlInputs control_inputs;
  double elapsed_time = (node_.now() - time_init_).seconds();
  mode_->triggerMode(elapsed_time, control_inputs);
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, static_cast<float>(-control_inputs["spin_thrust"])};
  const Eigen::Quaternionf attitude = Eigen::Quaternionf{
    static_cast<float>(control_inputs["qw"]),
    static_cast<float>(control_inputs["qx"]),
    static_cast<float>(control_inputs["qy"]),
    static_cast<float>(control_inputs["qz"])};
  attitude_setpoint_->update(attitude, thrust_sp);
}

} /* namespace ros2_uav::modes */