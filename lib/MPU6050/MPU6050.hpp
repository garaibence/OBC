#pragma once
#include "I2CInterface.hpp"
#include <cstdint>
#include <esp_err.h>

enum class DlpfBandwidth : uint8_t
{
    BW_260HZ = 0, // Accel: 260Hz - 0ms,    Gyro: 256Hz - 0.98ms
    BW_184HZ = 1, // Accel: 184Hz - 2ms,    Gyro: 188Hz - 1.9ms
    BW_94HZ = 2,  // Accel: 94Hz  - 3ms,    Gyro: 98Hz  - 2.8ms
    BW_44HZ = 3,  // Accel: 44Hz  - 4.9ms,  Gyro: 42Hz  - 4.8ms
    BW_21HZ = 4,  // Accel: 21Hz  - 8.5ms,  Gyro: 20Hz  - 8.3ms
    BW_10HZ = 5,  // Accel: 10Hz  - 13.8ms, Gyro: 10Hz  - 13.4ms
    BW_5HZ = 6    // Accel: 5Hz   - 19ms,   Gyro: 5Hz   - 18.6ms
};

enum class GyroRange : uint8_t
{
    RANGE_250_DEG = 0,  // +/- 250 °/s
    RANGE_500_DEG = 1,  // +/- 500 °/s
    RANGE_1000_DEG = 2, // +/- 1000 °/s
    RANGE_2000_DEG = 3  // +/- 2000 °/s
};

enum class AccelRange : uint8_t
{
    RANGE_2G = 0, // +/- 2g
    RANGE_4G = 1, // +/- 4g
    RANGE_8G = 2, // +/- 8g
    RANGE_16G = 3 // +/- 16g
};

class MPU6050
{
public:
    explicit MPU6050(I2CInterface &i2c, uint8_t addr = 0x68) : i2c(i2c), addr(addr) {}

    esp_err_t init();

    esp_err_t whoami(uint8_t &id);

    esp_err_t setDlpfBandwidth(DlpfBandwidth bw);
    esp_err_t setGyroRange(GyroRange range);
    esp_err_t setAccelRange(AccelRange range);

    esp_err_t readAccelerometerRaw(int16_t &raw_ax, int16_t &raw_ay, int16_t &raw_az);
    esp_err_t readAccelerometer(float &ax, float &ay, float &az);

    esp_err_t readGyroscopeRaw(int16_t &raw_gx, int16_t &raw_gy, int16_t &raw_gz);
    esp_err_t readGyroscope(float &gx, float &gy, float &gz);

    esp_err_t readTemperatureRaw(int16_t &raw_temp);
    esp_err_t readTemperatureC(float &temp);

    esp_err_t setI2CBypass(bool bypass);

private:
    esp_err_t readConfig(uint8_t &conf);
    esp_err_t writeConfig(uint8_t conf);

    I2CInterface &i2c;
    uint8_t addr;

    float accel_lsb_sensitivity = 16384.0f; // Default +/- 2g
    float gyro_lsb_sensitivity = 131.0f;    // Default +/- 250 °/s

    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_PIN_CFG = 0x37;
    static constexpr uint8_t REG_USER_CTRL = 0x6A;

    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
};