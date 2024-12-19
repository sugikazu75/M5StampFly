#pragma once

#include <BasicLinearAlgebra.h>
#include <stdint.h>
#include <vl53lx_api.h>
#include <vl53lx_platform.h>
#include <Arduino.h>
#include <sensor/tof/tof.hpp>

#define INT_BOTTOM   6
#define XSHUT_BOTTOM 7
#define INT_FRONT    8
#define XSHUT_FRONT  9
#define USER_A       0

class TofVL53L3C : public Tof
{
public:
  TofVL53L3C() {};
  ~TofVL53L3C() = default;

  void initialize(VL53LX_DEV dev) override;
  int16_t tof_get_range();
  void tof_test_ranging();
  void tof_int();
  static TofVL53L3C* instance;
  volatile uint8_t ToF_bottom_data_ready_flag_;
  VL53LX_Dev_t tof_front;
  VL53LX_Dev_t tof_bottom;
protected:
};
