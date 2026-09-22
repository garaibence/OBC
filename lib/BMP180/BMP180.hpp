#pragma once
#include "I2CSensor.hpp"
#include "I2CInterface.hpp"
#include <esp_err.h>

enum class Oversampling : uint8_t
{
    OSS_0 = 0, // ultra low power
    OSS_1 = 1, // standard
    OSS_2 = 2, // high resolution
    OSS_3 = 3  // ultra high resolution
};

class BMP180 : public I2CSensor
{
public:
    explicit BMP180(I2CInterface &i2c, uint8_t addr = 0x77) : I2CSensor(i2c, addr, REG_ID, REG_RESET) {}
    
    esp_err_t init() override;
    
    esp_err_t readTemperatureRaw(uint16_t &ut);
    esp_err_t readTemperature(float &temp);

    esp_err_t readPressureRaw(uint32_t &up);
    esp_err_t readPressure(float &pres);

    esp_err_t readAltitude(float &alt, float referencePres = 101325.0f);

    void setOversampling(Oversampling oss);

private:
    esp_err_t readCalibration();
    esp_err_t waitForConversion();
    uint8_t pressureCommand() const
    {
        return CMD_PRES | (static_cast<uint8_t>(oss) << 6);
    }

    bool hasTemp = false;

    Oversampling oss = Oversampling::OSS_0;

    int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;

    int32_t B5;

    static constexpr uint8_t REG_ID = 0xD0;
    static constexpr uint8_t REG_RESET = 0xE0;

    static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t REG_OUT_MSB = 0xF6;
    static constexpr uint8_t REG_CALIB = 0xAA;

    static constexpr uint8_t CMD_TEMP = 0x2E;
    static constexpr uint8_t CMD_PRES = 0x34;
};