#include "gyroLogger.hpp"
#include "timer.hpp"
#include "oneButton.hpp"
#include <cstring>

#define BIAS_LOOPS 1000

void GyroLogger::init(std::unique_ptr<MpuBase> &&mpu, std::unique_ptr<SdCardWorker> &&sdCardWorker, uint16_t rateUs) {
    m_mpu = std::move(mpu);
    m_sdCardWorker = std::move(sdCardWorker);
    m_rateUs = rateUs;
    m_sdCardWorker->getGyroBiasConfig(m_mpu->getGyroBias(), m_mpu->getAccelBias());
    m_sdCardWorker->setLoggingFileHeaderData(m_mpu->getAScale(), m_mpu->getGScale());
}

void GyroLogger::run() {
    uint32_t count = 0;
    absolute_time_t loopDurationOverOneSecond = 0;

    queue_entry_t entry;

    Timer timer_1;
    Timer timer_2;
    Timer timer_3;

    bool shouldWrite = false;

    OneButton button(26);
    button.attachClick([&shouldWrite]() {
        printf("Start logging\n");
        shouldWrite = !shouldWrite;
    });
    button.attachDoubleClick([this, &shouldWrite]() {
        if (!shouldWrite) {
            sleep_ms(1000);
            m_mpu->calculateBias(BIAS_LOOPS);
            m_sdCardWorker->createGyroBiasConfig(m_mpu->getGyroBias(), m_mpu->getAccelBias());
        } else {
            printf("Can't calculate bias while logging...\n");
        }
    });

    timer_2.start();
    timer_3.start();
    while (1) {
        timer_1.start();
        button.tick();

        memcpy(entry.gyro, m_mpu->getUnBiasedGyro(), 3 * sizeof(int16_t));
        memcpy(entry.acceleration, m_mpu->getUnBiasedAccel(), 3 * sizeof(int16_t));
        entry.us = timer_2.getTicks();
        entry.process = shouldWrite;

        m_sdCardWorker->pushData(entry);

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
