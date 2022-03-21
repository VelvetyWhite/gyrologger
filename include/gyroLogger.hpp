#ifndef __GYRO_LOGGER__
#define __GYRO_LOGGER__

#include "mpuBase.hpp"
#include "mpu6500.hpp"
#include "mpu6050.hpp"
#include "sdCardWorker.hpp"

#include <memory>

class GyroLogger {
public:
    void init(std::unique_ptr<MpuBase> &&mpu, std::unique_ptr<SdCardWorker> &&sdCardWorker, uint16_t rateUs);
    void run();

private:
    static void sdWorkerLauncher();

    int16_t m_acceleration[3];
    int16_t m_gyro[3];
    int16_t m_gyroCalibration[3];

    uint16_t m_rateUs;

    std::unique_ptr<MpuBase> m_mpu;
    std::unique_ptr<SdCardWorker> m_sdCardWorker;
};

#endif