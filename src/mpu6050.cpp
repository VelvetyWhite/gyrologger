#include "mpu6050.hpp"
#include "pico/binary_info.h"
#include <cstring>
#include <cstdio>

void Mpu6050::init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl) {
    m_i2c = i2c;
    m_sda = sda;
    m_scl = scl;

    printf("Initialising MPU6050\n");

    i2c_init(m_i2c, 400 * 1000);
    gpio_set_function(m_sda, GPIO_FUNC_I2C);
    gpio_set_function(m_scl, GPIO_FUNC_I2C);
    gpio_pull_up(m_sda);
    gpio_pull_up(m_scl);
    
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    
    reset();
}

void Mpu6050::init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs) {
    printf("MPU6050 running on i2c only\n");
}

void Mpu6050::reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x00};
    writeRegisters(0x6B, buf, 1);
    setCustomConfig();
}

int16_t* const Mpu6050::getRawGyro() {
    uint8_t buffer[6];
    
    readRegisters(MPU6050_GYRO_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        m_rawGyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
    return m_rawGyro;
}
int16_t* const Mpu6050::getRawAccel() {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    readRegisters(MPU6050_ACCEL_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        m_rawAccel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    return m_rawAccel;
}

void Mpu6050::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    i2c_write_blocking(m_i2c, MPU6050_DEVICE_ID, &reg, 1, true);
    busy_wait_us(10);
    i2c_read_blocking(m_i2c, MPU6050_DEVICE_ID, buf, len, false);
    busy_wait_us(10);
}

void Mpu6050::writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t aux[len + 1];
    aux[0] = reg;
    std::memcpy(&aux[1], buf, len);
    len++;
    i2c_write_blocking(m_i2c, MPU6050_DEVICE_ID, aux, len, false);
    busy_wait_us(10);
}

void Mpu6050::adjustConfig(uint8_t configRegister, uint8_t configValue, uint8_t bits, uint8_t shift) {
    uint8_t actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);

    // mask off the data before writing
    uint32_t mask = (1 << (bits)) - 1;
    configValue &= mask;

    mask <<= shift; //shift position taken from datasheet
    actualConfig &= ~mask;          // remove the current data at that spot
    actualConfig |= configValue << shift; // and add in the new data

    writeRegisters(configRegister, &actualConfig, 1);

    actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

void Mpu6050::setConfig(uint8_t configRegister, uint8_t configValue) {
    uint8_t actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);
    
    writeRegisters(configRegister, &configValue, 1);

    actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

void Mpu6050::setCustomConfig() {
    setConfig(MPU6050_CONFIG, MPU6050_CUSTOM_CONFIG);
    setConfig(MPU6050_GYRO_CONFIG, MPU6050_CUSTOM_GYRO_CONFIG);
    setConfig(MPU6050_ACCEL_CONFIG, MPU6050_CUSTOM_ACCEL_CONFIG);
}

