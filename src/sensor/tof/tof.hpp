#pragma once

#include <BasicLinearAlgebra.h>
#include <vl53lx_api.h>
#include <vl53lx_platform.h>

class Tof
{
public:
  Tof() {};
  ~Tof() = default;

  virtual void initialize(VL53LX_DEV dev) {};
  int16_t update();
  int16_t tof_get_range();
  void tof_test_ranging();

protected:
  VL53LX_DEV dev_;
};
