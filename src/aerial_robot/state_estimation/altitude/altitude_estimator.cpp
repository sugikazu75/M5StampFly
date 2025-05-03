#include <aerial_robot/state_estimation/altitude/altitude_estimator.hpp>

void AltitudeEstimator::initialize()
{
  altitude_kalman_filter_.initialize();

  raw_az_filter_.set_parameter(0.003, 0.0025);
  raw_az_d_filter_.set_parameter(0.1, 0.0025);  // alt158
  az_filter_.set_parameter(0.1, 0.0025);        // alt158
}

void AltitudeEstimator::update()
{
  range_prev_ = range_;

  int16_t raw_range = tof_bottom_get_range();

  // TODO: better process for tof sensor value
  if(raw_range > 40)
  {
    range_ = raw_range;
  }

  if(std::abs(range_ - range_prev_) > 100)
    return;

  // estimate process
  float filter_acc_z = raw_az_filter_.update(imu_->getAccZ(), 0.01);
  altitude_kalman_filter_.update(range_ / 1000.0, filter_acc_z, 0.01);
}
