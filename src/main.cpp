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
// #include <FastLED.h>
// #include "flight_control.hpp"
#include <Bitcraze_PMW3901.h>

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76

Bitcraze_PMW3901 flow(12);

void setup() {
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.printf("serial begin!\r\n");

  boolean success = flow.begin();
  delay(1000);

  USBSerial.printf("flow begin\n");
  if(success)
    USBSerial.printf("flow success\n");
  else
    USBSerial.printf("flow fail\n");

    // init_copter();
    // delay(100);
}

void loop() {
    // loop_400Hz();
}
