/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "rtc.h"

#include "gyroLogger.hpp"
#include "ds1307.hpp"

#define MPU_PIN_MISO 12
#define MPU_PIN_CS   13
#define MPU_PIN_SCK  10
#define MPU_PIN_MOSI 11

#define MPU_PIN_SDA 14
#define MPU_PIN_SCL 15

#define RTC_PIN_SDA 20
#define RTC_PIN_SCL 21

#define MPU_SPI_PORT spi1
#define MPU_I2C_PORT i2c1
#define RTC_I2C_PORT i2c0

#define BUFFER_QUEUE_SIZE 100
#define RATE_US 1000

int main() {
    stdio_init_all();
    rtc_init();
    time_init();

    sleep_ms(2000);

    std::unique_ptr<MpuBase> mpu = std::make_unique<Mpu6050>();
    std::unique_ptr<SdCardWorker> sdCardWorker = std::make_unique<SdCardWorker>();
    std::unique_ptr<MD_DS1307> rtc = std::make_unique<MD_DS1307>(RTC_I2C_PORT, RTC_PIN_SDA, RTC_PIN_SCL);

    if (!rtc->isRunning()) {
        rtc->control(DS1307_CLOCK_HALT, DS1307_OFF);
    }

    // rtc->yyyy = 2022;
    // rtc->mm = 3;
    // rtc->dd = 23;
    // rtc->h = 2;
    // rtc->m = 15;
    // rtc->s = 0;
    // rtc->writeTime();

    rtc->readTime();
    printf("%d.%d.%d %d:%d:%d Running: %d\n", rtc->yyyy, rtc->mm, rtc->dd, rtc->h, rtc->m, rtc->s, rtc->isRunning());

    sdCardWorker->init(BUFFER_QUEUE_SIZE);
    //mpu->init(MPU_SPI_PORT, MPU_PIN_MISO, MPU_PIN_MOSI, MPU_PIN_SCK, MPU_PIN_CS);
    mpu->init(MPU_I2C_PORT, MPU_PIN_SDA, MPU_PIN_SCL);

    mpu->setAccelerometerRange(MPU_ACCELEROMETER_RANGE::RANGE_16_G);
    mpu->setDlpfBandwidth(MPU_DLPF_BANDWIDTH::BAND_94_98_HZ);
    mpu->setGyroRange(MPU_GYRO_RANGE::RANGE_2000_DEG);
    sleep_ms(10);

    GyroLogger gyroLogger;
    gyroLogger.init(std::move(mpu), std::move(sdCardWorker), RATE_US);
    gyroLogger.run();

    return 0;
}
