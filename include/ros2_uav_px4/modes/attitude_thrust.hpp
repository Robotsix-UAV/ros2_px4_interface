#pragma once

#include <uav_cpp/modes/mode.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

namespace ros2_uav::modes
{
using uav_cpp::parameters::ParameterMap;

class AttitudeThrust : public px4_ros2::ModeBase
{
public:
  AttitudeThrust(std::string mode_name, rclcpp::Node & node, std::shared_ptr<uav_cpp::modes::Mode> mode);

private:
  rclcpp::Node & node_;
  std::shared_ptr<uav_cpp::modes::Mode> mode_;
  rclcpp::Time time_init_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint([[maybe_unused]] float dt) override;
};

} /* namespace ros2_uav::modes */
