#include "sdCardWorker.hpp"
#include <cstdio>
#include <cstring>
#include <functional>

bool SdCardWorker::init(uint8_t queueSize) {
    spi_init(spi0, SD_SPI_INITIAL_BAUD);
    
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

bool SdCardWorker::createNewFile(FIL &file, char *name) {
    FRESULT result = f_open(&file, name, FA_WRITE | FA_CREATE_ALWAYS);
    if (result != FR_OK) {
        printf("Failed to open %s for writing %s (%d)\n", name, FRESULT_str(result), result);
        return false;
    } else {
        printf("Successfully opened %s for writing\n", name);
    }
    return true;
}

bool SdCardWorker::writeFileHeader(FIL &file) {
    int result = f_printf(&file, "GYROFLOW IMU LOG\n"
                    "version,1.1\n"
                    "id,gyrologger\n"
                    "orientation,Zxy\n"
                    "tscale,0.000001\n"
                    "gscale,0.00106422515\n"
                    "ascale,0.00048828125\n"
                    "t,gx,gy,gz,ax,ay,az\n");
    if (result < 0) { 
        printf("f_printf faield\n"); 
        return false;
    }
    return true;
}

void SdCardWorker::pushData(queue_entry_t &entry) {
    queue_add_blocking(&m_queue, &entry);
}

void SdCardWorker::runnerFunction() {
    FIL file;
    char *name = "log.csv";

    bool finished = false;
    bool fileOpen = false;
    char buffer[100];

    while(true) {
        queue_remove_blocking(&m_queue, &m_entry);
        if (m_entry.process) {
            if (!fileOpen) {
                createNewFile(file, name);
                writeFileHeader(file);
                fileOpen = true;
            }
            snprintf(buffer, 100, "%llu,%d,%d,%d,%d,%d,%d\n", m_entry.us, 
                m_entry.gyro[0], m_entry.gyro[1], m_entry.gyro[2], 
                m_entry.acceleration[0], m_entry.acceleration[1], m_entry.acceleration[2]);
            f_write(&file, buffer, strlen(buffer), NULL);
        } else {
            if (fileOpen) {
                FRESULT result = f_close(&file);
                if (result != FR_OK) {
                    printf("f_close error: %s (%d)\n", FRESULT_str(result), result);
                } else {
                    printf("Successfully closed %s for writing\n", name);
                }
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
