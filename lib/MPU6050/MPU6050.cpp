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

esp_err_t MPU6050::readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    ax = be16(&buf[0]);
    ay = be16(&buf[2]);
    az = be16(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readGyroscope(int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_GYRO_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    gx = be16(&buf[0]);
    gy = be16(&buf[2]);
    gz = be16(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureRaw(int16_t &raw)
{
    uint8_t buf[2];
    esp_err_t err = i2c.readRegisters(addr, REG_TEMP_OUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw = be16(&buf[0]);
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureC(float &temp_c)
{
    int16_t raw;
    esp_err_t err = readTemperatureRaw(raw);
    if (err != ESP_OK)
        return err;
    // temp = (raw / 340) + 36.53
    temp_c = (float)raw / 340.0f + 36.53f;
    return ESP_OK;
}