#pragma once
#include "I2CInterface.hpp"
#include <esp_err.h>

class I2CSensor
{
public:
    I2CSensor(I2CInterface &i2c, uint8_t addr, uint8_t reg_id = 0x00, uint8_t reg_reset = 0x00) : i2c(i2c), addr(addr), REG_ID(reg_id), REG_RESET(reg_reset) {}
    virtual ~I2CSensor() = default;

    virtual esp_err_t init() { return i2c.writeRegister(addr, REG_RESET, 0x00); };
    virtual esp_err_t whoami(uint8_t &id) { return i2c.readRegister(addr, REG_ID, id); };

protected:
    I2CInterface &i2c;
    uint8_t addr;

    const uint8_t REG_ID;
    const uint8_t REG_RESET;
};