#include "MPU6050.hpp"
#include <cstring>

static auto be16 = [](const uint8_t *b) -> int16_t
{ return (int16_t)((b[0] << 8) | b[1]); };

esp_err_t MPU6050::init()
{
    uint8_t wake = 0x00;
    return i2c.writeRegister(addr, REG_PWR_MGMT_1, wake);
}

esp_err_t MPU6050::whoami(uint8_t &id)
{
    return i2c.readRegister(addr, REG_WHO_AM_I, id);
}

esp_err_t MPU6050::readAccelerometerRaw(int16_t &raw_ax, int16_t &raw_ay, int16_t &raw_az)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_ax = be16(&buf[0]);
    raw_ay = be16(&buf[2]);
    raw_az = be16(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readAccelerometer(float &ax, float &ay, float &az)
{
    int16_t raw_ax, raw_ay, raw_az;
    esp_err_t err = readAccelerometerRaw(raw_ax, raw_ay, raw_az);
    if (err != ESP_OK)
        return err;

    // accel = raw / 16384
    ax = raw_ax / 16384.0f;
    ay = raw_ay / 16384.0f;
    az = raw_az / 16384.0f;
    return ESP_OK;
}

esp_err_t MPU6050::readGyroscopeRaw(int16_t &raw_gx, int16_t &raw_gy, int16_t &raw_gz)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_GYRO_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_gx = be16(&buf[0]);
    raw_gy = be16(&buf[2]);
    raw_gz = be16(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readGyroscope(float &gx, float &gy, float &gz)
{
    int16_t raw_gx, raw_gy, raw_gz;
    esp_err_t err = readGyroscopeRaw(raw_gx, raw_gy, raw_gz);
    if (err != ESP_OK)
        return err;
    
    // rot = raw / 131
    gx = raw_gx / 131.0f;
    gy = raw_gy / 131.0f;
    gz = raw_gz / 131.0f;
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureRaw(int16_t &raw_temp)
{
    uint8_t buf[2];
    esp_err_t err = i2c.readRegisters(addr, REG_TEMP_OUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_temp = be16(&buf[0]);
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureC(float &temp)
{
    int16_t raw;
    esp_err_t err = readTemperatureRaw(raw);
    if (err != ESP_OK)
        return err;
    // temp = (raw / 340) + 36.53
    temp = raw / 340.0f + 36.53f;
    return ESP_OK;
}