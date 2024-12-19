#include <sensor/tof/tof_vl53l3c.hpp>



TofVL53L3C* TofVL53L3C::instance = nullptr;


void IRAM_ATTR tof_int_wrapper() {
    if (TofVL53L3C::instance) {
        TofVL53L3C::instance->tof_int();
    }
}

void IRAM_ATTR TofVL53L3C::tof_int() {
    ToF_bottom_data_ready_flag_ = 1;
}

void TofVL53L3C::initialize(VL53LX_DEV dev) {

    VL53LX_DEV ToF_front  = &tof_front;
    VL53LX_DEV ToF_bottom = &tof_bottom;
    dev_ = dev;
    uint8_t byteData;
    uint16_t wordData;

    dev_->comms_speed_khz   = 400;
    dev_->i2c_slave_address = 0x29;

    // USBSerial.printf("#tof_i2c_init_status:%d\r\n",vl53lx_i2c_init());

    // ToF Pin Initialize
    pinMode(XSHUT_BOTTOM, OUTPUT);
    pinMode(XSHUT_FRONT, OUTPUT);
    pinMode(INT_BOTTOM, INPUT);
    pinMode(INT_FRONT, INPUT);
    pinMode(USER_A, INPUT_PULLUP);

    // ToF Disable
    digitalWrite(XSHUT_BOTTOM, LOW);
    digitalWrite(XSHUT_FRONT, LOW);

    // Front ToF I2C address to 0x54
    digitalWrite(XSHUT_FRONT, HIGH);
    delay(100);
    VL53LX_SetDeviceAddress(ToF_front, 0x54);
    ToF_front->i2c_slave_address = 0x2A;

    delay(100);
    digitalWrite(XSHUT_BOTTOM, HIGH);

    // Bttom ToF setting
    USBSerial.printf("#1 WaitDeviceBooted Status:%d\n\r", VL53LX_WaitDeviceBooted(ToF_bottom));
    USBSerial.printf("#1 DataInit Status:%d\n\r", VL53LX_DataInit(ToF_bottom));
    USBSerial.printf("#1 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_bottom, VL53LX_DISTANCEMODE_MEDIUM));
    USBSerial.printf("#1 SetMeasurementTimingBuget Status:%d\n\r",
                     VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_bottom, 33000));
    USBSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x010F, &byteData));
    USBSerial.printf("#1 VL53LX Model_ID: %02X\n\r", byteData);
    USBSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x0110, &byteData));
    USBSerial.printf("#1 VL53LX Module_Type: %02X\n\r", byteData);
    USBSerial.printf("#1 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_bottom, 0x010F, &wordData));
    USBSerial.printf("#1 VL53LX: %04X\n\r", wordData);

    // Front ToF Setting
    USBSerial.printf("#2 WaitDeviceBooted Status:%d\n\r", VL53LX_WaitDeviceBooted(ToF_front));
    USBSerial.printf("#2 DataInit Status:%d\n\r", VL53LX_DataInit(ToF_front));
    USBSerial.printf("#1 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_front, VL53LX_DISTANCEMODE_LONG));
    USBSerial.printf("#2 SetMeasurementTimingBuget Status:%d\n\r",
                     VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_front, 33000));
    USBSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x010F, &byteData));
    USBSerial.printf("#2 VL53LX Model_ID: %02X\n\r", byteData);
    USBSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x0110, &byteData));
    USBSerial.printf("#2 VL53LX Module_Type: %02X\n\r", byteData);
    USBSerial.printf("#2 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_front, 0x010F, &wordData));
    USBSerial.printf("#2 VL53LX: %04X\n\r", wordData);

    attachInterrupt(INT_BOTTOM, tof_int_wrapper, FALLING);

    VL53LX_ClearInterruptAndStartMeasurement(ToF_bottom);
    delay(100);
    USBSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(ToF_bottom));
}

int16_t TofVL53L3C::tof_get_range() {
    int16_t range;
    int16_t range_min;
    int16_t range_max;
    int16_t range_ave;
    uint8_t count;

    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

    // uint32_t start_time = micros();
    VL53LX_GetMultiRangingData(dev_, pMultiRangingData);
    // uint32_t end_time = micros();
    // USBSerial.printf("ToF Time%f\n", (float)(end_time - start_time)*1.0e-6);
    uint8_t no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    // USBSerial.printf("Total N=%d ",no_of_object_found);
    range_min = 10000;
    range_max = 0;
    range_ave = 0;
    if (no_of_object_found == 0) {
        range_min = 9999;
        range_max = 0;
    } else {
        count = 0;
        for (uint8_t j = 0; j < no_of_object_found; j++) {
            if (MultiRangingData.RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID) {
                count++;
                range = MultiRangingData.RangeData[j].RangeMilliMeter;
                if (range_min > range) range_min = range;
                if (range_max < range) range_max = range;
                range_ave = range_ave + range;
            }
            // USBSerial.printf("No %d Status=%d Range %d mm ",j+1, MultiRangingData.RangeData[j].RangeStatus, range);
        }
        // USBSerial.printf("Max %d mm\n\r", range_max);
        if (count != 0) range_ave = range_ave / count;
    }
    VL53LX_ClearInterruptAndStartMeasurement(dev_);
    return range_max;
}

void TofVL53L3C::tof_test_ranging() {
    uint8_t status     = 0;
    uint8_t data_ready = 0;
    int16_t range;
    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

    if (status == 0) {
        status = VL53LX_ClearInterruptAndStartMeasurement(dev_);
    }
    delay(100);
    USBSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(dev_));

    USBSerial.printf("#Count ObjNo Status Range Signal(Mcps) Ambient(Mcps)\n\r");

    u_long st, now, old, end;
    uint16_t count;
    st  = micros();
    now = st;
    old = st;

    count = 0;
    USBSerial.printf("Start!\n\r");
    while (count < 500) {
        // VL53LX_WaitMeasurementDataReady(dev);
        // if(digitalRead(INT_BOTTOM)==0)

        VL53LX_GetMeasurementDataReady(dev_, &data_ready);
        if (data_ready == 1) {
            data_ready = 0;
            count++;
            VL53LX_GetMultiRangingData(dev_, pMultiRangingData);
            old                        = now;
            now                        = micros();
            uint8_t no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            USBSerial.printf("%7.4f %7.4f ", (float)(now - st) * 1e-6, (float)(now - old) * 1e-6);
            USBSerial.printf("%5d ", pMultiRangingData->StreamCount);
            USBSerial.printf("%1d ", no_of_object_found);
            for (uint8_t j = 0; j < no_of_object_found; j++) {
                if (j != 0) USBSerial.printf("\n\r                     ");
                USBSerial.printf("%d %5d %2.2f %2.2f ", MultiRangingData.RangeData[j].RangeStatus,
                                 MultiRangingData.RangeData[j].RangeMilliMeter,
                                 MultiRangingData.RangeData[j].SignalRateRtnMegaCps / 65536.0,
                                 MultiRangingData.RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            }

            VL53LX_ClearInterruptAndStartMeasurement(dev_);
            end = micros();
            USBSerial.printf("%8.6f", (float)(end - now) * 1.0e-6);
            USBSerial.printf("\n\r");
        }
    }
    USBSerial.printf("End!\n\r");
}
