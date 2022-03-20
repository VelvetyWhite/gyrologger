#ifndef __mpu_6500__
#define __mpu_6500__

#include "mpuBase.hpp"

#define READ_BIT 0x80

#define MPU6500_RA_CONFIG           0x1A
#define MPU6500_RA_GYRO_CONFIG      0x1B
#define MPU6500_RA_ACCEL_CONFIG     0x1C
#define MPU6500_RA_ACCEL_CONFIG_2   0x1D

#define MPU6500_GYRO_FS_250         0x00
#define MPU6500_GYRO_FS_500         0x01
#define MPU6500_GYRO_FS_1000        0x02
#define MPU6500_GYRO_FS_2000        0x03

#define MPU6500_ACCEL_FS_2          0x00
#define MPU6500_ACCEL_FS_4          0x01
#define MPU6500_ACCEL_FS_8          0x02
#define MPU6500_ACCEL_FS_16         0x03

#define MPU6500_DLPF_BW_256         0x00
#define MPU6500_DLPF_BW_188         0x01
#define MPU6500_DLPF_BW_98          0x02
#define MPU6500_DLPF_BW_42          0x03
#define MPU6500_DLPF_BW_20          0x04
#define MPU6500_DLPF_BW_10          0x05
#define MPU6500_DLPF_BW_5           0x06

#define MPU6500_GYRO_DATA           0x43
#define MPU6500_ACCEL_DATA          0x3B

#define MPU6500_CUSTOM_CONFIG           0b00000010
#define MPU6500_CUSTOM_GYRO_CONFIG      0b00011000
#define MPU6500_CUSTOM_ACCEL_CONFIG     0b00011000
#define MPU6500_CUSTOM_ACCEL_CONFIG_2   0b00000010

#define MPU6500_GYRO_CALIBRATION_X 2186
#define MPU6500_GYRO_CALIBRATION_Y 2408
#define MPU6500_GYRO_CALIBRATION_Z -8

class Mpu6500 : public MpuBase {
public:
    void init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs);
    void init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl);
    void reset();
    void setCustomConfig();
    int16_t* const getRawGyro();
    int16_t* const getRawAccel();

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