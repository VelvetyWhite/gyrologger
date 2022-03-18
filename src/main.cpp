/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <memory>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "rtc.h"

#include "mpuBase.hpp"
#include "mpu6500.hpp"
#include "mpu6050.hpp"
#include "timer.hpp"

#include "gyroLogger.hpp"

// void calibrate_gyro(MpuBase *mpu, int16_t gyroCal[3], int count)  //Used to calibrate the gyro. The gyro must be still while calibration happens
// {
//     int16_t *dataReading;
//     int64_t temp[3] = {0};
//     for (int i = 0; i < count; i++)
//     {
//         dataReading = mpu->getRawGyro();
//         temp[0] += dataReading[0];
//         temp[1] += dataReading[1];
//         temp[2] += dataReading[2];
//     }
//     gyroCal[0] = temp[0] / count;
//     gyroCal[1] = temp[1] / count;
//     gyroCal[2] = temp[2] / count;
// }

int main() {
    stdio_init_all();
    time_init();

    GyroLogger gyroLogger;
    gyroLogger.init(100, 1000);
    gyroLogger.run();

    return 0;
}
//https://github.com/mathertel/OneButton