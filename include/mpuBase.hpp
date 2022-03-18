#ifndef __mpu_base__
#define __mpu_base__

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#include <cstdio>

class MpuBase {
public:
    virtual void init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs);
    virtual void init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl);
    virtual int16_t* const getRawGyro() = 0;
    virtual int16_t* const getRawAccel() = 0;
protected:
    int16_t m_rawGyro[3];
    int16_t m_rawAccel[3];
};

#endif