#include "sdCardWorker.hpp"
#include "hardware/rtc.h"
#include <cstdio>
#include <cstring>
#include <functional>

#define MPU_BIAS_FILE_NAME "bias"

bool SdCardWorker::init(uint8_t queueSize) {
    spi_init(SD_SPI_PORT, SD_SPI_INITIAL_BAUD);
    
    m_sd = sd_get_by_num(0);
    FRESULT result = f_mount(&m_sd->fatfs, m_sd->pcName, 1);
    if (result != FR_OK) {
        printf("Failed to mount sd card %s (%d)\n", FRESULT_str(result), result);
        return false;
    } else {
        printf("Mounted sd card\n");
    }

    queue_init(&m_queue, sizeof(queue_entry_t), queueSize);

    auto runnerWrapper = []() {
        SdCardWorker *sdCardWorker = (SdCardWorker*) multicore_fifo_pop_blocking();
        sdCardWorker->runnerFunction();
    };

    multicore_launch_core1(runnerWrapper);
    multicore_fifo_push_blocking((uintptr_t)this);

    return true;
}

bool SdCardWorker::createNewFile(FIL &file, const char *name) {
    FRESULT result = f_open(&file, name, FA_WRITE | FA_CREATE_ALWAYS);
    if (result != FR_OK) {
        printf("Failed to open %s for writing %s (%d)\n", name, FRESULT_str(result), result);
        return false;
    } else {
        printf("Successfully opened %s for writing\n", name);
    }
    return true;
}

bool SdCardWorker::closeFile(FIL &file, const char *name) {
    FRESULT result = f_close(&file);
    if (result != FR_OK) {
        printf("f_close error: %s (%d)\n", FRESULT_str(result), result);
        return false;
    } else {
        printf("Successfully closed %s\n", name);
    }
    return true;
}

bool SdCardWorker::writeFileHeader(FIL &file) {
    snprintf(m_buffer, 128, "GYROFLOW IMU LOG\n"
                    "version,1.1\n"
                    "id,gyrologger\n"
                    "orientation,Zxy\n"
                    "tscale,0.000001\n"
                    "gscale,%.9f\n"
                    "ascale,%.9f\n"
                    "t,gx,gy,gz,ax,ay,az\n", fileHeaderData[1], fileHeaderData[0]);
    int result = f_printf(&file, m_buffer);
    if (result < 0) { 
        printf("f_printf failed\n"); 
        return false;
    }
    return true;
}

void SdCardWorker::pushData(queue_entry_t &entry) {
    queue_add_blocking(&m_queue, &entry);
}

void SdCardWorker::setLoggingFileHeaderData(float aScale, float gScale) {
    fileHeaderData[0] = aScale;
    fileHeaderData[1] = gScale;
}

void SdCardWorker::runnerFunction() {
    FIL file;
    datetime_t dateTime;

    bool finished = false;
    bool fileOpen = false;

    while(true) {
        queue_remove_blocking(&m_queue, &m_entry);
        if (m_entry.process) {
            if (!fileOpen) {
                rtc_get_datetime(&dateTime);
                snprintf(m_dateTimeBuffer, sizeof(m_dateTimeBuffer), "%d-%d-%d-%d-%d-%d%s", dateTime.year, dateTime.month, dateTime.day, dateTime.hour, dateTime.min, dateTime.sec, ".csv");
                createNewFile(file, m_dateTimeBuffer);
                writeFileHeader(file);
                fileOpen = true;
            }
            snprintf(m_buffer, 128, "%llu,%d,%d,%d,%d,%d,%d\n", m_entry.us, 
                m_entry.gyro[0], m_entry.gyro[1], m_entry.gyro[2], 
                m_entry.acceleration[0], m_entry.acceleration[1], m_entry.acceleration[2]);
            f_write(&file, m_buffer, strlen(m_buffer), NULL);
        } else {
            if (fileOpen) {
                closeFile(file, m_dateTimeBuffer);
                fileOpen = false;
            }
        }
    }
    f_unmount(m_sd->pcName);
}

void SdCardWorker::runnerFunctionWrapper() {
    SdCardWorker &sdCardWorker = *(SdCardWorker*) multicore_fifo_pop_blocking();
    sdCardWorker.runnerFunction();
}

void SdCardWorker::createGyroBiasConfig(const int16_t *gyroBias, const int16_t *accelBias) {
    FIL file;
    createNewFile(file, MPU_BIAS_FILE_NAME);
    f_write(&file, gyroBias, 3 * sizeof(int16_t), NULL);
    f_write(&file, accelBias, 3 * sizeof(int16_t), NULL);
    closeFile(file, MPU_BIAS_FILE_NAME);
}

void SdCardWorker::getGyroBiasConfig(int16_t *gyroBias, int16_t *accelBias) {
    FIL file;
    FRESULT result = f_open(&file, MPU_BIAS_FILE_NAME, FA_READ);
    if (result != FR_OK) {
        printf("Failed to open %s for reading %s (%d)\n", MPU_BIAS_FILE_NAME, FRESULT_str(result), result);
        return;
    } else {
        printf("Successfully opened %s for reading\n", MPU_BIAS_FILE_NAME);
    }
    f_read(&file, gyroBias, 3 * sizeof(int16_t), NULL);
    f_read(&file, accelBias, 3 * sizeof(int16_t), NULL);
    closeFile(file, MPU_BIAS_FILE_NAME);
}
