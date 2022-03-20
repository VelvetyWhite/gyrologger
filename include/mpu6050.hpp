#ifndef __mpu_6050__
#define __mpu_6050__

#include "mpuBase.hpp"

#define MPU6050_I2CADDR_DEFAULT             0x68 ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_DEVICE_ID                   0x68 ///< The correct MPU6050_WHO_AM_I value

#define MPU6050_SELF_TEST_X                 0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y                 0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z                 0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A                 0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV                  0x19 ///< sample rate divisor register

#define MPU6050_CONFIG                      0x1A ///< General configuration register
#define MPU6050_GYRO_CONFIG                 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG                0x1C ///< Accelerometer specific configration register

#define MPU6050_INT_PIN_CONFIG              0x37 ///< Interrupt pin configuration register
#define MPU6050_WHO_AM_I                    0x75 ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET           0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL                   0x6A ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1                  0x6B ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2                  0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H                      0x41 ///< Temperature data high byte register
#define MPU6050_TEMP_L                      0x42 ///< Temperature data low byte register

#define MPU6050_ACCEL_DATA                   0x3B ///< base address for sensor data reads
#define MPU6050_GYRO_DATA                    0x43 ///< base address for sensor data reads

#define MPU6050_GYRO_FS_250                 0x00
#define MPU6050_GYRO_FS_500                 0x01
#define MPU6050_GYRO_FS_1000                0x02
#define MPU6050_GYRO_FS_2000                0x03

#define MPU6050_ACCEL_FS_2                  0x00
#define MPU6050_ACCEL_FS_4                  0x01
#define MPU6050_ACCEL_FS_8                  0x02
#define MPU6050_ACCEL_FS_16                 0x03

#define MPU6050_DLPF_BW_256                 0x00
#define MPU6050_DLPF_BW_188                 0x01
#define MPU6050_DLPF_BW_98                  0x02
#define MPU6050_DLPF_BW_42                  0x03
#define MPU6050_DLPF_BW_20                  0x04
#define MPU6050_DLPF_BW_10                  0x05
#define MPU6050_DLPF_BW_5                   0x06

#define MPU6050_CUSTOM_CONFIG               0b00000010 //dlpf 94/98
#define MPU6050_CUSTOM_GYRO_CONFIG          0b00011000 //2000 deg
#define MPU6050_CUSTOM_ACCEL_CONFIG         0b00011000 //16g

#define MPU6050_GYRO_CALIBRATION_X -113
#define MPU6050_GYRO_CALIBRATION_Y 3
#define MPU6050_GYRO_CALIBRATION_Z -56

class Mpu6050 : public MpuBase {
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

    i2c_inst_t *m_i2c;
    uint32_t m_sda;
    uint32_t m_scl;
};

#endif
