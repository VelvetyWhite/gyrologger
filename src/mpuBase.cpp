#include "mpuBase.hpp"

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
        m_rawGyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
    return m_rawGyro;
}
