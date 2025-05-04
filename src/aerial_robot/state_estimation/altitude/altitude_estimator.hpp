#pragma once

#include <sensor/imu/imu.hpp>
#include <aerial_robot/state_estimation/altitude/alt_kalman.hpp>
#include <memory>
#include <utils/lpf/lpf.hpp>
#include <tof.hpp>

class AltitudeEstimator
{
public:
  AltitudeEstimator(std::shared_ptr<Imu> imu):
    imu_(imu)
  {}
  ~AltitudeEstimator() = default;

  void initialize();
  void update();
  float getAltitude() {return altitude_kalman_filter_.getAltitude();}

private:
  std::shared_ptr<Imu> imu_;
  Alt_kalman altitude_kalman_filter_;

  float gravity_ = 9.80665;

  Filter raw_az_filter_;

  int16_t min_tof_ = 40;
  int16_t last_input_tof_ = 0;
  int16_t tof_diff_torelance_ = 500;
};
