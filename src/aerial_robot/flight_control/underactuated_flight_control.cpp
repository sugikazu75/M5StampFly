#include <aerial_robot/flight_control/underactuated_flight_control.hpp>

UnderActuatedFlightController::UnderActuatedFlightController(std::shared_ptr<Odometry> odom,
                                                             std::shared_ptr<BaseNavigator> navigator):
  odom_(odom),
  navigator_(navigator)
{
  control_input_ = {0.0, 0.0, 0.0, 0.0};
}

void UnderActuatedFlightController::update()
{
  float target_z = navigator_->getTargetPosZ();
  float target_roll = navigator_->getTargetRoll();
  float target_pitch = navigator_->getTargetPitch();
  float target_yaw = navigator_->getTargetYaw();

  float z = odom_->getPosZ();
  float roll = odom_->getRoll();
  float pitch = odom_->getPitch();
  float yaw = odom_->getYaw();

  float omega_x = odom_->getOmegaX();
  float omega_y = odom_->getOmegaY();
  float omega_z = odom_->getOmegaZ();

  // z_pid_.update();
  // roll_pid_.update();
  // pitch_pid_.update();
  // yaw_pid_.update();

  // control_input_(0) = z_pid_.getResult();
  // control_input_(1) = roll_pid_.getResult();
  // control_input_(2) = pitch_pid_.getResult();
  // control_input_(3) = yaw_pid_.getResult();
}
