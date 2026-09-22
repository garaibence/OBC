#pragma once
#include "I2CSensor.hpp"
#include "I2CInterface.hpp"
#include <esp_err.h>

enum class QMC_MODE : uint8_t
{
    MODE_SUSPEND = 0x00,
    MODE_NORMAL = 0x01,
    MODE_SINGLE = 0x02,
    MODE_CONTINUOUS = 0x03
};

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
    RNG_8G = 0x04,
    RNG_12G = 0x08,
    RNG_30G = 0x0C
};

enum class QMC_OSR : uint8_t
{
    OSR_512 = 0x00,
    OSR_256 = 0x40,
    OSR_128 = 0x80,
    OSR_64 = 0xC0
};

class QMC5883P : public I2CSensor
{
public:
    explicit QMC5883P(I2CInterface &i2c, uint8_t addr = 0x2C) : I2CSensor(i2c, addr, REG_ID) {}

    esp_err_t init() override;
    esp_err_t whoami(uint8_t &id);

    esp_err_t setConfig(QMC_ODR odr, QMC_RNG rng, QMC_OSR osr, QMC_MODE mode);
    esp_err_t setMode(QMC_MODE mode);
    esp_err_t setOutputDataRate(QMC_ODR odr);
    esp_err_t setRange(QMC_RNG rng);
    esp_err_t setOversamplingRatio(QMC_OSR osr);

    float getScale() { return scale; }

    esp_err_t readMagnetoRaw(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z);
    esp_err_t readMagneto(float &mag_x, float &mag_y, float &mag_z);

private:
    esp_err_t readStatus(bool &rdy);
    static float rngToScale(QMC_RNG rng)
    {
        switch (rng)
        {
        case QMC_RNG::RNG_2G:
            return 1.0f / 12000.0f;
        case QMC_RNG::RNG_8G:
            return 1.0f / 3000.0f;
        case QMC_RNG::RNG_12G:
            return 1.0f / 2000.0f;
        case QMC_RNG::RNG_30G:
            return 1.0f / 800.0f;
        default:
            return 0.0f;
        }
    }

    float scale = 0.0f;

    static constexpr uint8_t REG_ID = 0x00;

    static constexpr uint8_t REG_DATA_X_LSB = 0x01;
    static constexpr uint8_t REG_STATUS = 0x09;
    
    static constexpr uint8_t REG_CTRL_1 = 0x0A;
    static constexpr uint8_t REG_CTRL_2 = 0x0B;
};