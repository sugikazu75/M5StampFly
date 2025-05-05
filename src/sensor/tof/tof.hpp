#pragma once

#include <BasicLinearAlgebra.h>
#include <vl53lx_api.h>
#include <vl53lx_platform.h>

class Tof
{
public:
  Tof() {};
  ~Tof() = default;

  virtual void initialize() {};
  // int16_t update();

protected:
};
