#ifndef __mpu_base__
#define __mpu_base__

#include "hardware/spi.h"
#include "hardware/i2c.h"

#include <cstdio>

typedef enum {
    RANGE_2_G = 0b00,
    RANGE_4_G = 0b01,
    RANGE_8_G = 0b10,
    RANGE_16_G = 0b11,
} MPU_ACCELEROMETER_RANGE;

typedef enum {
    RANGE_250_DEG = 0b00,
    RANGE_500_DEG = 0b01,
    RANGE_1000_DEG = 0b10,
    RANGE_2000_DEG = 0b11,
} MPU_GYRO_RANGE;

typedef enum {
    BAND_260_256_HZ = 0b000,
    BAND_184_188_HZ = 0b001,
    BAND_94_98_HZ = 0b010,
    BAND_44_42_HZ = 0b011,
    BAND_21_20_HZ = 0b100,
    BAND_10_HZ = 0b101,
    BAND_5_HZ = 0b110,
} MPU_DLPF_BANDWIDTH;

class MpuBase {
public:
    virtual void init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs);
    virtual void init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl);

    virtual int16_t* const getRawGyro() = 0;
    virtual int16_t* const getRawAccel() = 0;

    int16_t* const getUnBiasedGyro();
    int16_t* const getUnBiasedAccel();

    float getAScale() const { return m_aScale[m_accelerometerRange]; }
    float getGScale() const { return m_gScale[m_gyroRange]; }

    virtual void setAccelerometerRange(MPU_ACCELEROMETER_RANGE accelerometerRange);
    virtual void setGyroRange(MPU_GYRO_RANGE gyroRange);
    virtual void setDlpfBandwidth(MPU_DLPF_BANDWIDTH dlpfBandwidth);

    void calculateBias(uint16_t count);
    void adjustConfig(uint8_t configRegister, uint8_t configValue, uint8_t bits, uint8_t shift);
    void setConfig(uint8_t configRegister, uint8_t configValue);
protected:
    int16_t *convertAxisFromHiLowBuffer(uint8_t *buffer, int16_t *axisData);

    virtual void readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) = 0;
    virtual void writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len) = 0;

    int16_t m_rawGyro[3];
    int16_t m_rawAccel[3];
    int16_t m_gyroBias[3] = { 0, 0, 0 };
    int16_t m_accelBias[3] = { 0, 0, 0 };

    MPU_ACCELEROMETER_RANGE m_accelerometerRange;
    MPU_GYRO_RANGE m_gyroRange;
    MPU_DLPF_BANDWIDTH m_dlpfBandwidth;

    const float m_aScale[4] = { 0.00006103515625f, 0.0001220703125f, 0.000244140625f, 0.00048828125f };
    const float m_gScale[4] = { 0.000133231240458f, 0.000266462480916f, 0.00053211257622f, 0.001064225152439f };

    const uint16_t m_accelLsbSensitivity[4] = { 16384, 8192, 4096, 2048 };
};

#endif