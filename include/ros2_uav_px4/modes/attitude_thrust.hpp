#pragma once

#include <uav_cpp/modes/mode.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParameterMap;

class AttitudeThrust : public px4_ros2::ModeBase
{
public:
  AttitudeThrust(
    std::string mode_name, rclcpp::Node & node,
    std::shared_ptr<uav_cpp::modes::Mode> mode);

  void setParameters(std::shared_ptr<ParameterMap> parameters)
  {
    parameters_ = parameters;
    mode_->setParameters(parameters);
  }

  void getRequiredParameters(std::vector<std::string> & required_parameters)
  {
    mode_->getRequiredParameters(required_parameters);
    required_parameters.push_back("px4.thrust_constant_coefficient");
    required_parameters.push_back("px4.thrust_linear_coefficient");
    required_parameters.push_back("px4.thrust_quadratic_coefficient");
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<uav_cpp::modes::Mode> mode_;
  std::shared_ptr<ParameterMap> parameters_;
  rclcpp::Time time_init_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> vehicle_local_position_;
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> vehicle_angular_velocity_;
  std::shared_ptr<px4_ros2::OdometryAttitude> vehicle_attitude_;
  void onActivate() override {odometryUpdate(); mode_->reset();}
  void onDeactivate() override {}
  void odometryUpdate();
  void updateSetpoint([[maybe_unused]] float dt) override;
};

} /* namespace ros2_uav::modes */
