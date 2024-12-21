#pragma once

#include <vector>
#include <aerial_robot/motor/motor.hpp>
#include <memory>

class QuadrotorHardware
{
public:
  QuadrotorHardware(std::vector<std::shared_ptr<Motor>> motors);
  ~QuadrotorHardware() = default;

  void update(BLA::Matrix<4, 1> actuator_input);

private:
  std::vector<std::shared_ptr<Motor>> motors_;
};
