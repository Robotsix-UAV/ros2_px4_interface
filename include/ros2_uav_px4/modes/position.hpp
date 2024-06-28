#pragma once

#include <ros2_uav_px4/modes/attitude_thrust.hpp>
#include <uav_cpp/modes/se3_position.hpp>


namespace ros2_uav::modes
{
class Position : public ros2_uav::modes::AttitudeThrust
{
public:
  Position(rclcpp::Node & node)
  : ros2_uav::modes::AttitudeThrust("Offboard Position", node,
      std::make_shared<uav_cpp::modes::Se3Position>())
  {
  }
};

class ExecutorPosition : public px4_ros2::ModeExecutorBase
{
public:
  enum class State
  {
    ARM,
    TAKEOFF,
    POSITION
  };

  ExecutorPosition(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
  : px4_ros2::ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{false}, owned_mode)
  {}

  void onActivate() override
  {
    runState(State::ARM);
  }

  void onDeactivate(DeactivateReason) override
  {
  }

  void runState(State state)
  {
    switch (state) {
      case State::ARM:
        RCLCPP_INFO(node().get_logger(), "[Position executor] Arming.");
        arm(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::TAKEOFF);
            }
          });
        break;
      case State::TAKEOFF:
        RCLCPP_INFO(node().get_logger(), "[Position executor] Taking off.");
        takeoff(
          [this](px4_ros2::Result result)
          {
            if (result == px4_ros2::Result::Success) {
              runState(State::POSITION);
            }
          },
          5.0,
          0.0); // TODO:(robotsix) Get it from parameters
        break;

      case State::POSITION:
        RCLCPP_INFO(node().get_logger(), "[Position executor] Position Control.");
        scheduleMode(
          ownedMode().id(), [](px4_ros2::Result) {return;});
        break;
    }
  }
};

} /* namespace ros2_uav::modes */
