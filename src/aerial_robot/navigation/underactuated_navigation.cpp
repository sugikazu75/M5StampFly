#include <aerial_robot/navigation/underactuated_navigation.hpp>

void UnderActuatedNavigator::initialize()
{
  rc_init();
}

void UnderActuatedNavigator::update()
{

#if 0
  USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r",
                   Stick[THROTTLE],
                   Stick[AILERON],
                   Stick[ELEVATOR],
                   Stick[RUDDER],
                   Stick[BUTTON_ARM],
                   Stick[BUTTON_FLIP],
                   Stick[CONTROLMODE],
                   Stick[ALTCONTROLMODE],
                   Stick[LOG]);
#endif
}
