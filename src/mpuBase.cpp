#include "mpuBase.hpp"
#include "pico/time.h"
#include <cmath>

void MpuBase::adjustConfig(uint8_t configRegister, uint8_t configValue, uint8_t bits, uint8_t shift) {
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

void MpuBase::setConfig(uint8_t configRegister, uint8_t configValue) {
    uint8_t actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data before changes: %d\n", actualConfig);
    
    writeRegisters(configRegister, &configValue, 1);

    actualConfig = 0;
    readRegisters(configRegister, &actualConfig, 1);
    printf("Config data after changes: %d\n", actualConfig);
}

int16_t *MpuBase::convertAxisFromHiLowBuffer(uint8_t *buffer, int16_t *axisData) {
    for (int i = 0; i < 3; i++) {
        axisData[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
    return axisData;
}

void MpuBase::calculateBias(uint16_t count) {
    int16_t *dataReading;
    int64_t temp[6] = {0};
    for (int i = 0; i < count; i++) {
        dataReading = getRawGyro();
        temp[0] += dataReading[0];
        temp[1] += dataReading[1];
        temp[2] += dataReading[2];
        dataReading = getRawAccel();
        temp[3] += dataReading[0];
        temp[4] += dataReading[1];
        temp[5] += dataReading[2];
        sleep_ms(1);
    }
    m_gyroBias[0] = temp[0] / count;
    m_gyroBias[1] = temp[1] / count;
    m_gyroBias[2] = temp[2] / count;
    m_accelBias[0] = temp[3] / count;
    m_accelBias[1] = temp[4] / count;
    m_accelBias[2] = temp[5] / count;

    for (uint8_t i = 0; i < 3; i++) { // compensate for gravity by leaving 1g on the corresponding axis
        if (std::abs(m_accelBias[i]) >= m_accelLsbSensitivity[m_accelerometerRange]) {
            m_accelBias[i] > 0 ? m_accelBias[i] -= m_accelLsbSensitivity[m_accelerometerRange] : 
                                                m_accelBias[i] += m_accelLsbSensitivity[m_accelerometerRange];
        }
    }

    printf("Gyro bias: %d %d %d | Accel bias:  %d %d %d\n", m_gyroBias[0], m_gyroBias[1], m_gyroBias[2], 
        m_accelBias[0], m_accelBias[1], m_accelBias[2]);
}

int16_t* const MpuBase::getUnBiasedGyro() {
    int16_t *gyroData = getRawGyro();
    gyroData[0] -= m_gyroBias[0];
    gyroData[1] -= m_gyroBias[1];
    gyroData[2] -= m_gyroBias[2];
    return gyroData;
}
int16_t* const MpuBase::getUnBiasedAccel() {
    int16_t *accelData = getRawAccel();
    accelData[0] -= m_accelBias[0];
    accelData[1] -= m_accelBias[1];
    accelData[2] -= m_accelBias[2];
    return accelData;
}

void MpuBase::setAccelerometerRange(MPU_ACCELEROMETER_RANGE accelerometerRange) {
    m_accelerometerRange = accelerometerRange;
}
void MpuBase::setGyroRange(MPU_GYRO_RANGE gyroRange) {
    m_gyroRange = gyroRange;
}
void MpuBase::setDlpfBandwidth(MPU_DLPF_BANDWIDTH dlpfBandwidth) {
    m_dlpfBandwidth = dlpfBandwidth;
}
