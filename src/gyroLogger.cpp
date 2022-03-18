#include "gyroLogger.hpp"
#include "timer.hpp"
#include <cstring>

void GyroLogger::init(uint16_t bufferQueueSize, uint16_t rateUs) {
    m_rateUs = rateUs;
    m_sdCardWorker.init(bufferQueueSize);
    m_mpu = std::make_unique<Mpu6050>();
    //m_mpu->init(SPI_PORT, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS);
    m_mpu->init(I2C_PORT, PIN_SDA, PIN_SCL);
    sleep_ms(10);
}

void GyroLogger::run() {
    uint32_t count = 0;
    absolute_time_t loopDurationOverOneSecond = 0;

    queue_entry_t entry;

    m_gyroCalibration[0] = MPU6050_GYRO_CALIBRATION_X;
    m_gyroCalibration[1] = MPU6050_GYRO_CALIBRATION_Y;
    m_gyroCalibration[2] = MPU6050_GYRO_CALIBRATION_Z;

    Timer timer_1;
    Timer timer_2;
    Timer timer_3;

    timer_2.start();
    timer_3.start();
    while (1) {
        timer_1.start();
        memcpy(entry.gyro, m_mpu->getRawGyro(), 3 * sizeof(int16_t));
        memcpy(entry.acceleration, m_mpu->getRawAccel(), 3 * sizeof(int16_t));
        entry.gyro[0] -= m_gyroCalibration[0];
        entry.gyro[1] -= m_gyroCalibration[1];
        entry.gyro[2] -= m_gyroCalibration[2];
        entry.us = timer_2.getTicks();
        
        if (entry.us >= 30000000) {
            entry.process = false;
        }
        m_sdCardWorker.pushData(entry);

        count++;
        if (timer_3.getTicks() >= 1000000) {
            printf("%d\n", count);
            printf("%llu,%d,%d,%d,%d,%d,%d\n", entry.us, entry.gyro[0], entry.gyro[1], entry.gyro[2], entry.acceleration[0], entry.acceleration[1], entry.acceleration[2]);
            count = 0;
            timer_3.start();
        }
        int64_t delay_us = m_rateUs - timer_1.getTicks();
        if (delay_us > 0) {
            busy_wait_us(delay_us);
        }
    }
}
