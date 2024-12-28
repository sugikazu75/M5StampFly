#pragma once

#include <BasicLinearAlgebra.h>
#include <utils/pid/pid.hpp>
#include <memory>
#include <aerial_robot/state_estimation/state_estimator.hpp>
#include <aerial_robot/navigation/navigation_base.hpp>

class UnderActuatedFlightController
{
public:
  UnderActuatedFlightController(std::shared_ptr<Odometry> odom,
                                std::shared_ptr<BaseNavigator> navigator);
  ~UnderActuatedFlightController() = default;

  void update();
  BLA::Matrix<4, 1> getControlInput() {return control_input_;}

private:
  std::shared_ptr<Odometry> odom_;
  std::shared_ptr<BaseNavigator> navigator_;
  BLA::Matrix<4, 1> control_input_;
  PID z_pid_;
  PID roll_pid_;
  PID pitch_pid_;
  PID yaw_pid_;
};
