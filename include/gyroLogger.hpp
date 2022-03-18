#ifndef __GYRO_LOGGER__
#define __GYRO_LOGGER__

#include "mpuBase.hpp"
#include "mpu6500.hpp"
#include "mpu6050.hpp"
#include "sdCardWorker.hpp"

#include <memory>

#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

#define PIN_SDA 14
#define PIN_SCL 15

#define SPI_PORT spi1
#define I2C_PORT i2c1

class GyroLogger {
public:
    void init(uint16_t bufferQueueSize, uint16_t rateUs);
    void run();

private:
    static void sdWorkerLauncher();

    int16_t m_acceleration[3];
    int16_t m_gyro[3];
    int16_t m_gyroCalibration[3];

    uint16_t m_rateUs;

    std::unique_ptr<MpuBase> m_mpu;
    SdCardWorker m_sdCardWorker;
};

#endif