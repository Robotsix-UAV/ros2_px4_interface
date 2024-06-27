#pragma once

#include <ros2_uav_px4/modes/attitude_thrust.hpp>
#include <uav_cpp/modes/spin.hpp>


namespace ros2_uav::modes
{
class Spin : public ros2_uav::modes::AttitudeThrust
{
public:
  Spin(rclcpp::Node & node, std::shared_ptr<ParameterMap> parameters)
  : ros2_uav::modes::AttitudeThrust("Spin", node, std::make_shared<uav_cpp::modes::Spin>(parameters))
  {
  }
};

class ExecutorSpin : public px4_ros2::ModeExecutorBase
{
public:
  enum class State
  {
    ARM,
    SPIN
  };

  ExecutorSpin(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
    : px4_ros2::ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{false} ,owned_mode)
    {}

  void onActivate() override
  {
    RCLCPP_DEBUG(node().get_logger(), "ExecutorSpin activated.");
    runState(State::ARM);
  }

  void onDeactivate(DeactivateReason) override
  {
    RCLCPP_DEBUG(node().get_logger(), "ExecutorSpin deactivated.");
  }

  void runState(State state){
    switch (state) {
      case State::ARM:
        RCLCPP_INFO(node().get_logger(), "Arming.");
        arm(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::SPIN);
            }
          });
        break;
        case State::SPIN:
            scheduleMode(
                ownedMode().id(), [](px4_ros2::Result){ return; });
            break;
    }
  }
};

} /* namespace ros2_uav::modes */