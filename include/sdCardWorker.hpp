#ifndef __SD_CARD_WORKER__
#define __SD_CARD_WORKER__

#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "rtc.h"

#define SD_PIN_MISO 4
#define SD_PIN_CS   5
#define SD_PIN_SCK  2
#define SD_PIN_MOSI 3

#define SD_SPI_PORT spi0
#define SD_SPI_BAUD 20000000

#define SD_SPI_INITIAL_BAUD 1000000

typedef struct
{
    int16_t gyro[3] = { 0, 0, 0 };
    int16_t acceleration[3] = { 0, 0, 0 };
    uint64_t us = 0;
    bool process = true;
} queue_entry_t;

class SdCardWorker {
public:
    bool init(uint8_t queueSize);
    void pushData(queue_entry_t &entry);
    void setLoggingFileHeaderData(float aScale, float gScale);

    void createGyroBiasConfig(const int16_t *gyroBias, const int16_t *accelBias);
    void getGyroBiasConfig(int16_t *gyroBias, int16_t *accelBias);

private:
    bool createNewFile(FIL &file, const char *name);
    bool closeFile(FIL &file, const char *name);
    bool writeFileHeader(FIL &file);
    void runnerFunction();
    static void runnerFunctionWrapper();

    float fileHeaderData[2] = { 0.0f };

    sd_card_t *m_sd;
    queue_t m_queue;
    queue_entry_t m_entry;
};

#endif