/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"

#include <vector>
#include <memory>
#include <imu.hpp>
#include <opt.hpp>
#include <i2c.hpp>
#include <INA3221.h>
#include <bmm150.h>

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76

#define SDA_PIN      (3)
#define SCL_PIN      (4)

std::shared_ptr<INA3221> battery_voltage;
std::shared_ptr<BMM150> mag;

void setup() {

#if 0
  /* begin debug */
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.printf("serial begin!\r\n");

  // init spi bus
  USBSerial.printf("SPI Initilize status:%d\n\r", spi_init());

  // init imu
  imu_init();

  // init optical flow
  powerUp(&optconfig);
  initRegisters();

  // i2c
  Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);
  // i2c_master_init();
  i2c_scan();

  // battery
  battery_voltage = std::make_shared<INA3221>(INA3221_ADDR40_GND);
  battery_voltage->begin(&Wire1);
  battery_voltage->reset();

  // TODO mag(bmm150)
  mag = std::make_shared<BMM150>();
  if (mag->initialize() == BMM150_E_ID_NOT_CONFORM)
    {
      USBSerial.printf("#BMM150 Chip ID can not read!\n\r");
      while (1);
    }
  else
    USBSerial.printf("#BMM150 Initialize done!\n\r");
  mag->set_op_mode(BMM150_FORCED_MODE);
  /* end debug*/
#endif

#if 1
  /* begin deploy */
  init_copter();
  delay(100);
  /* end deploy */
#endif
}

void loop() {
#if 0
  /* begin debug */
  // optical flow (PMW3901)
  int16_t flow_deltaX, flow_deltaY;
  readMotionCount(&flow_deltaX, &flow_deltaY);
  USBSerial.print("X: ");
  USBSerial.print(flow_deltaX);
  USBSerial.print(", Y: ");
  USBSerial.print(flow_deltaY);
  USBSerial.print("\n");

  // imu (bmi270)
  imu_update();
  float gyro_x = imu_get_gyro_x();
  float gyro_y = imu_get_gyro_y();
  float gyro_z = imu_get_gyro_z();
  float acc_x = imu_get_acc_x();
  float acc_y = imu_get_acc_y();
  float acc_z = imu_get_acc_z();
  USBSerial.printf("gyro: %8.4f %8.4f %8.4f\n", gyro_x, gyro_y, gyro_z);
  USBSerial.printf("acc: %8.4f %8.4f %8.4f\n", acc_x, acc_y, acc_z);

  USBSerial.printf("Voltage: %f\n\r", battery_voltage->getVoltage(INA3221_CH2));

  bmm150_mag_data magvalue;
  mag->read_mag_data();
  mag->set_op_mode(BMM150_FORCED_MODE);
  int16_t mag_x = - mag->raw_mag_data.raw_datay;
  int16_t mag_y =   mag->raw_mag_data.raw_datax;
  int16_t mag_z =   mag->raw_mag_data.raw_dataz;
  USBSerial.printf("mag: %8.4f %8.4f %8.4f\n", mag_x, mag_y, mag_z);

  USBSerial.print("\n");
  // double T,P;
  // uint8_t result = pressure_.startMeasurment();
  // USBSerial.print("delay: ");
  // USBSerial.print(result);
  // USBSerial.print(" ");
  // delay(result);
  // result = pressure_.getTemperatureAndPressure(T,P);

  // USBSerial.print(T);
  // USBSerial.print(" ");
  // USBSerial.print(P);
  // USBSerial.println();

  delay(10);
  /* end debug */
#endif

#if 1
  /* begin deploy */
  loop_400Hz();
  /* end deploy */
#endif

}
