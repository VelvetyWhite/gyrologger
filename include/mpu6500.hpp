#ifndef __mpu_6500__
#define __mpu_6500__

#include "mpuBase.hpp"

#define READ_BIT 0x80

#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_ACCEL_CONFIG_2   0x1D

#define MPU6500_GYRO_DATA           0x43
#define MPU6500_ACCEL_DATA          0x3B

#define MPU6500_CUSTOM_CONFIG           0b00000010
#define MPU6500_CUSTOM_GYRO_CONFIG      0b00011000
#define MPU6500_CUSTOM_ACCEL_CONFIG     0b00011000
#define MPU6500_CUSTOM_ACCEL_CONFIG_2   0b00000010

class Mpu6500 : public MpuBase {
public:
    void init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs);
    void init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl);
    void reset();
    void setCustomConfig();
    int16_t* const getRawGyro();
    int16_t* const getRawAccel();

    virtual void setAccelerometerRange(MPU_ACCELEROMETER_RANGE accelerometerRange);
    virtual void setGyroRange(MPU_GYRO_RANGE gyroRange);
    virtual void setDlpfBandwidth(MPU_DLPF_BANDWIDTH dlpfBandwidth);

private:
    void readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
    void writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len);

    static inline void cs_select(uint8_t cs);
    static inline void cs_deselect(uint8_t cs);

    spi_inst_t *m_spi;
    uint32_t m_miso;
    uint32_t m_mosi;
    uint32_t m_sck;
    uint32_t m_cs;
};

#endif