/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

/* Example code to talk to a MPU9250 MEMS accelerometer and gyroscope.
   Ignores the magnetometer, that is left as a exercise for the reader.

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor SPI) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic MPU9250 board, other
   boards may vary.

   GPIO 4 (pin 6) MISO/spi0_rx-> ADO on MPU9250 board
   GPIO 5 (pin 7) Chip select -> NCS on MPU9250 board
   GPIO 6 (pin 9) SCK/spi0_sclk -> SCL on MPU9250 board
   GPIO 7 (pin 10) MOSI/spi0_tx -> SDA on MPU9250 board
   3.3v (pin 36) -> VCC on MPU9250 board
   GND (pin 38)  -> GND on MPU9250 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
   The particular device used here uses the same pins for I2C and SPI, hence the
   using of I2C names
*/

#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

#define SPI_PORT spi1
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

#define MPU6500_CUSTOM_CONFIG           0b00000010
#define MPU6500_CUSTOM_GYRO_CONFIG      0b11111011
#define MPU6500_CUSTOM_ACCEL_CONFIG     0b11111000
#define MPU6500_CUSTOM_ACCEL_CONFIG_2   0b00000010

typedef struct
{
    int16_t gyro[3];
    int16_t acceleration[3];
} queue_entry_t;

queue_t call_queue;
queue_t results_queue;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    busy_wait_us(1);
    spi_read_blocking(SPI_PORT, reg, buf, len);
    cs_deselect();
    busy_wait_us(1);
}

static void write_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.;
    uint8_t aux[len + 1];
    aux[0] = reg;
    memcpy(&aux[1], buf, len);
    len++;
    cs_select();
    spi_write_blocking(SPI_PORT, aux, len);
    cs_deselect();
    busy_wait_us(1);
}

static void adjustConfig(uint8_t configRegister, uint8_t configValue, uint8_t bits, uint8_t shift) {
    uint8_t actualConfig = 0;
    read_registers(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);

    // mask off the data before writing
    uint32_t mask = (1 << (bits)) - 1;
    configValue &= mask;

    mask <<= shift; //shift position taken from datasheet
    actualConfig &= ~mask;          // remove the current data at that spot
    actualConfig |= configValue << shift; // and add in the new data

    write_registers(configRegister, &actualConfig, 1);

    actualConfig = 0;
    read_registers(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

static void setConfig(uint8_t configRegister, uint8_t configValue) {
    uint8_t actualConfig = 0;
    read_registers(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);
    
    write_registers(configRegister, &configValue, 1);

    actualConfig = 0;
    read_registers(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

static void setCustomConfig() {
    setConfig(MPU6500_RA_CONFIG, MPU6500_CUSTOM_CONFIG);
    setConfig(MPU6500_RA_GYRO_CONFIG, MPU6500_CUSTOM_GYRO_CONFIG);
    setConfig(MPU6500_RA_ACCEL_CONFIG, MPU6500_CUSTOM_ACCEL_CONFIG);
    setConfig(MPU6500_RA_ACCEL_CONFIG_2, MPU6500_CUSTOM_ACCEL_CONFIG_2);
}

static void mpu9250_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x00};
    write_registers(0x6B, buf, 1);
    setCustomConfig();
}

static void mpu9250_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    read_registers(0x41, buffer, 2);

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu9250_read_raw_accel(int16_t accel[3]) { //Used to get the raw acceleration values from the mpu
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu9250_read_raw_gyro(int16_t gyro[3]) {  //Used to get the raw gyro values from the mpu
    uint8_t buffer[6];
    
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

void calibrate_gyro(int16_t gyroCal[3], int count)  //Used to calibrate the gyro. The gyro must be still while calibration happens
{
    int16_t dataReading[3];
    int64_t temp[3] = {0};
    for (int i = 0; i < count; i++)
    {
        mpu9250_read_raw_gyro(dataReading);
        temp[0] += dataReading[0];
        temp[1] += dataReading[1];
        temp[2] += dataReading[2];
    }
    gyroCal[0] = temp[0] / count;
    gyroCal[1] = temp[1] / count;
    gyroCal[2] = temp[2] / count;
}

void core1_entry() {
    printf("Started second core\n");

    while(1) {
        queue_entry_t entry;
        queue_remove_blocking(&call_queue, &entry);

        //printf("Acc. X = %d, Y = %d, Z = %d\n", entry.acceleration[0], entry.acceleration[1], entry.acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", entry.gyro[0], entry.gyro[1], entry.gyro[2]);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");

    spi_init(SPI_PORT, 1000000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

    mpu9250_reset();

    // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);

    uint baudrate = spi_set_baudrate(SPI_PORT, 20000000);
    printf("Baudrate: %d\n", baudrate);
    busy_wait_us(5);

    int16_t acceleration[3], gyro[3], gyroCalibration[3];

    calibrate_gyro(gyroCalibration, 1000);
    printf("Calibration: %d %d %d\n", gyroCalibration[0], gyroCalibration[1], gyroCalibration[2]);

    queue_init(&call_queue, sizeof(queue_entry_t), 10);
    queue_entry_t entry;

    multicore_launch_core1(core1_entry);

    absolute_time_t loopStart = get_absolute_time();
    absolute_time_t loopEnd;
    uint64_t elapsedUs = 0;
    uint16_t count = 0;
    while (1) {
        //mpu9250_read_raw(acceleration, gyro, &temp);
        mpu9250_read_raw_gyro(entry.gyro);
        mpu9250_read_raw_accel(entry.acceleration);
        entry.gyro[0] -= gyroCalibration[0];
        entry.gyro[1] -= gyroCalibration[1];
        entry.gyro[2] -= gyroCalibration[2];

        queue_add_blocking(&call_queue, &entry);

        count++;
        loopEnd = get_absolute_time();
        elapsedUs = loopEnd - loopStart;
        if (elapsedUs >= 100000) {
            printf("%d\n", count);
            count = 0;
            loopStart = loopEnd;
        }

        //sleep_ms(1000);
    }

    return 0;
}
