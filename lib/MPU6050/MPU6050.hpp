#pragma once
#include "I2CInterface.hpp"
#include <cstdint>
#include <esp_err.h>

class MPU6050
{
public:
    explicit MPU6050(I2CInterface &i2c, uint8_t addr = 0x68) : i2c(i2c), addr(addr) {}

    esp_err_t init();

    esp_err_t whoami(uint8_t &id);

    esp_err_t readAccelerometerRaw(int16_t &raw_ax, int16_t &raw_ay, int16_t &raw_az);
    esp_err_t readAccelerometer(float &ax, float &ay, float &az);

    esp_err_t readGyroscopeRaw(int16_t &raw_gx, int16_t &raw_gy, int16_t &raw_gz);
    esp_err_t readGyroscope(float &gx, float &gy, float &gz);

    esp_err_t readTemperatureRaw(int16_t &raw_temp);
    esp_err_t readTemperatureC(float &temp);

private:
    I2CInterface &i2c;
    uint8_t addr;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
};