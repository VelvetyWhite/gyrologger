/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <memory>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "rtc.h"

#include "mpuBase.hpp"
#include "mpu6500.hpp"
#include "mpu6050.hpp"
#include "timer.hpp"

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

#define PIN_SDA 14
#define PIN_SCL 15

#define SPI_PORT spi1
#define I2C_PORT i2c1

//#define RATE_US 2000 //500hz
#define RATE_US 1000 //1000hz
//#define RATE_US 500 //2000hz

typedef struct
{
    int16_t gyro[3] = { 0, 0, 0 };
    int16_t acceleration[3] = { 0, 0, 0 };
    uint64_t us = 0;
} queue_entry_t;

queue_t call_queue;
queue_t results_queue;

void calibrate_gyro(MpuBase *mpu, int16_t gyroCal[3], int count)  //Used to calibrate the gyro. The gyro must be still while calibration happens
{
    int16_t *dataReading;
    int64_t temp[3] = {0};
    for (int i = 0; i < count; i++)
    {
        dataReading = mpu->getRawGyro();
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
    spi_init(spi0, 1000000);
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr == FR_OK) {
        printf("Mounted sd card\n");
    } else {
        printf("Failed to mount sd card %s (%d)\n", FRESULT_str(fr), fr);
    }
    FIL fil;
    const char* const filename = "log.csv";
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr == FR_OK) {
        printf("Successfully opened %s for writing\n", filename);
    } else {
        printf("Failed to open %s for writing %s (%d)\n", filename, FRESULT_str(fr), fr);
    }

    int res = f_printf(&fil, "GYROFLOW IMU LOG\n"
                    "version,1.1\n"
                    "id,custom_logger_name\n"
                    "orientation,Zxy\n"
                    "tscale,0.000001\n"
                    "gscale,0.00106422515\n"
                    "ascale,0.00048828125\n"
                    "t,gx,gy,gz,ax,ay,az\n");
    if (res < 0) {
        printf("f_printf faield\n");
    }
    uint64_t start = get_absolute_time();
    bool finished = false;
    char buffer[100];
    while(1) {
        queue_entry_t entry;
        queue_remove_blocking(&call_queue, &entry);
        if (get_absolute_time() - start <= 30000000) {
            snprintf(buffer, 100, "%llu,%d,%d,%d,%d,%d,%d\n", entry.us, entry.gyro[0], entry.gyro[1], entry.gyro[2], entry.acceleration[0], entry.acceleration[1], entry.acceleration[2]);
            f_write(&fil, buffer, strlen(buffer), NULL);
        } else if(!finished) {
            finished = true;
            fr = f_close(&fil);
            if (FR_OK != fr) {
                printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
            }
            f_unmount(pSD->pcName);
            printf("Finished writing...\n");
        }
        //printf("Acc. X = %d, Y = %d, Z = %d\n", entry.acceleration[0], entry.acceleration[1], entry.acceleration[2]);
        //printf("Gyro. X = %d, Y = %d, Z = %d\n", entry.gyro[0], entry.gyro[1], entry.gyro[2]);
    }

    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    f_unmount(pSD->pcName);
}

int main() {
    stdio_init_all();
    time_init();
    sleep_ms(1000);

    printf("Init mpu\n");
    
    std::unique_ptr<MpuBase> mpu = std::make_unique<Mpu6050>();
    //mpu->init(SPI_PORT, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS);
    mpu->init(I2C_PORT, PIN_SDA, PIN_SCL);
    
    printf("done init mpu\n");
    
    int16_t acceleration[3], gyro[3];
    //int16_t gyroCalibration[3] = { MPU6500_GYRO_CALIBRATION_X, MPU6500_GYRO_CALIBRATION_Y, MPU6500_GYRO_CALIBRATION_Z };
    int16_t gyroCalibration[3] = { MPU6050_GYRO_CALIBRATION_X, MPU6050_GYRO_CALIBRATION_Y, MPU6050_GYRO_CALIBRATION_Z };

    //calibrate_gyro(mpu.get(), gyroCalibration, 1000);
    printf("Calibration: %d %d %d\n", gyroCalibration[0], gyroCalibration[1], gyroCalibration[2]);

    queue_init(&call_queue, sizeof(queue_entry_t), 100);
    queue_entry_t entry;

    multicore_launch_core1(core1_entry);

    uint32_t count = 0;
    absolute_time_t loopDurationOverOneSecond = 0;

    Timer timer_1;
    Timer timer_2;
    Timer timer_3;

    timer_2.start();
    timer_3.start();
    while (1) {
        timer_1.start();
        memcpy(entry.gyro, mpu->getRawGyro(), 3);
        memcpy(entry.acceleration, mpu->getRawAccel(), 3);
        entry.gyro[0] -= gyroCalibration[0];
        entry.gyro[1] -= gyroCalibration[1];
        entry.gyro[2] -= gyroCalibration[2];
        entry.us = timer_2.getTicks();
        queue_add_blocking(&call_queue, &entry);

        count++;
        if (timer_3.getTicks() >= 1000000) {
            printf("%d\n", count);
            //printf("%llu,%d,%d,%d,%d,%d,%d\n", entry.us, entry.gyro[0], entry.gyro[1], entry.gyro[2], entry.acceleration[0], entry.acceleration[1], entry.acceleration[2]);
            count = 0;
            timer_3.start();
        }
        int64_t delay_us = RATE_US - timer_1.getTicks();
        if (delay_us > 0) {
            busy_wait_us(delay_us);
        }
    }

    return 0;
}
