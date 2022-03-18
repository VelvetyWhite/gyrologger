#include "mpu6500.hpp"
#include "pico/binary_info.h"
#include <cstring>
#include <cstdio>

void Mpu6500::init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl) {
    printf("MPU6500 running on spi only\n");
}

void Mpu6500::init(spi_inst_t *spi, uint32_t miso, uint32_t mosi, uint32_t sck, uint32_t cs) {
    m_spi = spi;
    m_miso = miso;
    m_mosi = mosi;
    m_sck = sck;
    m_cs = cs;
    printf("Initialising MPU6500\n");
    spi_init(spi, 1000000);
    gpio_set_function(m_miso, GPIO_FUNC_SPI);
    gpio_set_function(m_sck, GPIO_FUNC_SPI);
    gpio_set_function(m_mosi, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(m_miso, m_mosi, m_sck, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(m_cs);
    gpio_set_dir(m_cs, GPIO_OUT);
    gpio_put(m_cs, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(m_cs, "SPI CS"));

    reset();

    // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    readRegisters(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);

    uint baudrate = spi_set_baudrate(m_spi, 20000000);
    printf("Baudrate: %d\n", baudrate);
    busy_wait_us(5);
}

void Mpu6500::reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x00};
    writeRegisters(0x6B, buf, 1);
    setCustomConfig();
}

int16_t* const Mpu6500::getRawGyro() {
    uint8_t buffer[6];
    
    readRegisters(MPU6500_GYRO_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        m_rawGyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
    return m_rawGyro;
}
int16_t* const Mpu6500::getRawAccel() {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    readRegisters(MPU6500_ACCEL_DATA, buffer, 6);

    for (int i = 0; i < 3; i++) {
        m_rawAccel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    return m_rawAccel;
}

void Mpu6500::readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select(m_cs);
    spi_write_blocking(m_spi, &reg, 1);
    busy_wait_us(10);
    spi_read_blocking(m_spi, reg, buf, len);
    cs_deselect(m_cs);
    busy_wait_us(10);
}

void Mpu6500::writeRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t aux[len + 1];
    aux[0] = reg;
    std::memcpy(&aux[1], buf, len);
    len++;
    cs_select(m_cs);
    spi_write_blocking(m_spi, aux, len);
    cs_deselect(m_cs);
    busy_wait_us(10);
}

void Mpu6500::adjustConfig(uint8_t configRegister, uint8_t configValue, uint8_t bits, uint8_t shift) {
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

void Mpu6500::setConfig(uint8_t configRegister, uint8_t configValue) {
    uint8_t actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);
    
    writeRegisters(configRegister, &configValue, 1);

    actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

void Mpu6500::setCustomConfig() {
    setConfig(MPU6500_RA_CONFIG, MPU6500_CUSTOM_CONFIG);
    setConfig(MPU6500_RA_GYRO_CONFIG, MPU6500_CUSTOM_GYRO_CONFIG);
    setConfig(MPU6500_RA_ACCEL_CONFIG, MPU6500_CUSTOM_ACCEL_CONFIG);
    setConfig(MPU6500_RA_ACCEL_CONFIG_2, MPU6500_CUSTOM_ACCEL_CONFIG_2);
}
