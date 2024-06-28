#include "ros2_uav_px4/modes/attitude_thrust.hpp"
#include "ros2_uav_px4/utils/tf2_eigen.hpp"

namespace ros2_uav::modes
{
using uav_cpp::controllers::ControlInputs;

AttitudeThrust::AttitudeThrust(
  std::string mode_name, rclcpp::Node & node,
  std::shared_ptr<uav_cpp::modes::Mode> mode)
: px4_ros2::ModeBase(node, Settings{mode_name, true}),
  node_(node), mode_(mode)
{
  attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  vehicle_local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  vehicle_angular_velocity_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*this);
  vehicle_attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*this);
  time_init_ = node_.now();
}

void AttitudeThrust::odometryUpdate()
{
  auto position = vehicle_local_position_->positionNed();
  auto velocity = vehicle_local_position_->velocityNed();
  auto attitude = vehicle_attitude_->attitude();
  auto angular_velocity = vehicle_angular_velocity_->angularVelocityFrd();
  mode_->setCurrentOdometry(
    eigenNedToTf2Nwu(position),
    eigenNedToTf2Nwu(attitude),
    eigenNedToTf2Nwu(velocity),
    eigenNedToTf2Nwu(angular_velocity));
}

void AttitudeThrust::updateSetpoint([[maybe_unused]] float dt)
{
  odometryUpdate();
  ControlInputs control_inputs;
  double elapsed_time = (node_.now() - time_init_).seconds();
  mode_->triggerMode(elapsed_time, control_inputs);
  // Check the control inputs
  if (control_inputs.find("thrust") == control_inputs.end() ||
    control_inputs.find("qw") == control_inputs.end() ||
    control_inputs.find("qx") == control_inputs.end() ||
    control_inputs.find("qy") == control_inputs.end() ||
    control_inputs.find("qz") == control_inputs.end())
  {
    RCLCPP_ERROR(node_.get_logger(), "[Attitude Thrust Mode] Not all control inputs are set");
    return;
  }
  // Set the attitude setpoint
  // Conversion of the thrust
  double thrust_constant_coefficient, thrust_linear_coefficient, thrust_quadratic_coefficient;
  (*parameters_)["px4.thrust_constant_coefficient"]->getValue(thrust_constant_coefficient);
  (*parameters_)["px4.thrust_linear_coefficient"]->getValue(thrust_linear_coefficient);
  (*parameters_)["px4.thrust_quadratic_coefficient"]->getValue(thrust_quadratic_coefficient);
  float thrust = control_inputs["thrust"];
  float normalized_thrust = 0.0;
  // Find the normalized with thrust = c + l * t_n + q * t_n^2
  float discriminant = thrust_linear_coefficient * thrust_linear_coefficient -
    4 * thrust_quadratic_coefficient * (thrust_constant_coefficient - thrust);
  if (discriminant < 0) {
    RCLCPP_WARN(
      node_.get_logger(), "[Attitude Thrust Mode] Negative discriminant in thrust conversion");
  } else {
    normalized_thrust = (-thrust_linear_coefficient + std::sqrt(discriminant)) /
      (2 * thrust_quadratic_coefficient);
  }
  const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -normalized_thrust};
  const Eigen::Quaternionf attitude_sp = Eigen::Quaternionf{
    static_cast<float>(control_inputs["qw"]),
    static_cast<float>(control_inputs["qx"]),
    -static_cast<float>(control_inputs["qy"]),
    -static_cast<float>(control_inputs["qz"])};
  RCLCPP_DEBUG(
    node_.get_logger(), "[Attitude Thrust Mode] Attitude: %f %f %f %f, Thrust: %f",
    attitude_sp.w(), attitude_sp.x(), attitude_sp.y(), attitude_sp.z(), normalized_thrust);
  attitude_setpoint_->update(attitude_sp, thrust_sp);
}

} /* namespace ros2_uav::modes */
