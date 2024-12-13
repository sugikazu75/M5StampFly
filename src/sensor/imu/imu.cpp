#include <sensor/imu/imu.hpp>

void Imu::update()
{
  readImuData();
}
