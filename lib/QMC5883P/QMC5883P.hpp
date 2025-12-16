#pragma once
#include "I2CInterface.hpp"
#include <esp_err.h>

enum class QMC_ODR : uint8_t
{
    ODR_10Hz = 0x00,
    ODR_50Hz = 0x04,
    ODR_100Hz = 0x08,
    ODR_200Hz = 0x0C
};

enum class QMC_RNG : uint8_t
{
    RNG_2G = 0x00,
    RNG_8G = 0x10
};

enum class QMC_OSR : uint8_t
{
    OSR_512 = 0x00,
    OSR_256 = 0x40,
    OSR_128 = 0x80,
    OSR_64 = 0xC0
};

class QMC5883P
{
public:
    explicit QMC5883P(I2CInterface &i2c, uint8_t addr = 0x2C) : i2c(i2c), addr(addr) {}

    esp_err_t init();
    esp_err_t whoami(uint8_t &id);

    esp_err_t setConfig(QMC_ODR odr, QMC_RNG rng, QMC_OSR osr);

    esp_err_t readMagnetoRaw(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z);
    esp_err_t readMagneto(float &mag_x, float &mag_y, float &mag_z);

private:
    esp_err_t readStatus(bool &rdy);

    I2CInterface &i2c;
    uint8_t addr;

    float scale = 0.0f;

    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_DATA_X_LSB = 0x01;
    static constexpr uint8_t REG_STATUS = 0x09;
    static constexpr uint8_t REG_CTRL_1 = 0x0A;
    static constexpr uint8_t REG_CTRL_2 = 0x0B;
};