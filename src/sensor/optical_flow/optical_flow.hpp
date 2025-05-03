#pragma once

#include <BasicLinearAlgebra.h>
#include <memory>

class OpticalFlow
{
public:
  OpticalFlow() = default;
  ~OpticalFlow() = default;

  virtual void initialize();
  virtual void update();;
  void dumpFlow() {USBSerial.printf("OpticalFlow: %d %d\n", flow_delta_x_, flow_delta_y_);}

protected:
  int16_t flow_delta_x_;
  int16_t flow_delta_y_;
};