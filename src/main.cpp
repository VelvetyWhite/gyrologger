/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "rtc.h"

#include "gyroLogger.hpp"

#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

#define PIN_SDA 14
#define PIN_SCL 15

#define SPI_PORT spi1
#define I2C_PORT i2c1

#define BUFFER_QUEUE_SIZE 100
#define RATE_US 1000

int main() {
    stdio_init_all();
    rtc_init();
    time_init();

    sleep_ms(2000);

    std::unique_ptr<MpuBase> mpu = std::make_unique<Mpu6050>();
    std::unique_ptr<SdCardWorker> sdCardWorker = std::make_unique<SdCardWorker>();

    sdCardWorker->init(BUFFER_QUEUE_SIZE);
    //mpu->init(SPI_PORT, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS);
    mpu->init(I2C_PORT, PIN_SDA, PIN_SCL);

    mpu->setAccelerometerRange(MPU_ACCELEROMETER_RANGE::RANGE_16_G);
    mpu->setDlpfBandwidth(MPU_DLPF_BANDWIDTH::BAND_94_98_HZ);
    mpu->setGyroRange(MPU_GYRO_RANGE::RANGE_2000_DEG);
    sleep_ms(10);

    GyroLogger gyroLogger;
    gyroLogger.init(std::move(mpu), std::move(sdCardWorker), RATE_US);
    gyroLogger.run();

    return 0;
}
