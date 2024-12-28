#pragma once

#include <aerial_robot/navigation/navigation_base.hpp>
#include <communication/rc.hpp>

class UnderActuatedNavigator : public BaseNavigator
{
public:
  UnderActuatedNavigator(){};
  ~UnderActuatedNavigator() = default;

  void initialize() override;
  void update() override;
};
