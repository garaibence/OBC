#include "MPU6050.hpp"
#include "ByteUtils.hpp"

esp_err_t MPU6050::init()
{
    uint8_t wake = 0x00;
    esp_err_t err = i2c.writeRegister(addr, REG_PWR_MGMT_1, wake);
    if (err != ESP_OK)
        return err;
    return ESP_OK;
}

esp_err_t MPU6050::whoami(uint8_t &id)
{
    return i2c.readRegister(addr, REG_WHO_AM_I, id);
}

esp_err_t MPU6050::setDlpfBandwidth(DlpfBandwidth bw)
{
    uint8_t current_conf;
    esp_err_t err = i2c.readRegister(addr, REG_CONFIG, current_conf);
    if (err != ESP_OK)
        return err;

    current_conf &= 0xF8;
    current_conf |= static_cast<uint8_t>(bw);

    return i2c.writeRegister(addr, REG_CONFIG, current_conf);
}

esp_err_t MPU6050::setGyroRange(GyroRange range)
{
    uint8_t current_conf;
    esp_err_t err = i2c.readRegister(addr, REG_GYRO_CONFIG, current_conf);
    if (err != ESP_OK)
        return err;

    current_conf &= ~0x18;
    current_conf |= (static_cast<uint8_t>(range) << 3);

    err = i2c.writeRegister(addr, REG_GYRO_CONFIG, current_conf);
    if (err != ESP_OK)
        return err;

    switch (range)
    {
    case GyroRange::RANGE_250_DEG:
        gyro_lsb_sensitivity = 131.0f;
        break;
    case GyroRange::RANGE_500_DEG:
        gyro_lsb_sensitivity = 65.5f;
        break;
    case GyroRange::RANGE_1000_DEG:
        gyro_lsb_sensitivity = 32.8f;
        break;
    case GyroRange::RANGE_2000_DEG:
        gyro_lsb_sensitivity = 16.4f;
        break;
    }
    return ESP_OK;
}

esp_err_t MPU6050::setAccelRange(AccelRange range)
{
    uint8_t current_conf;
    esp_err_t err = i2c.readRegister(addr, REG_ACCEL_CONFIG, current_conf);
    if (err != ESP_OK)
        return err;

    current_conf &= ~0x18;
    current_conf |= (static_cast<uint8_t>(range) << 3);

    err = i2c.writeRegister(addr, REG_ACCEL_CONFIG, current_conf);
    if (err != ESP_OK)
        return err;

    switch (range)
    {
    case AccelRange::RANGE_2G:
        accel_lsb_sensitivity = 16384.0f;
        break;
    case AccelRange::RANGE_4G:
        accel_lsb_sensitivity = 8192.0f;
        break;
    case AccelRange::RANGE_8G:
        accel_lsb_sensitivity = 4096.0f;
        break;
    case AccelRange::RANGE_16G:
        accel_lsb_sensitivity = 2048.0f;
        break;
    }
    return ESP_OK;
}

esp_err_t MPU6050::readAccelerometerRaw(int16_t &raw_ax, int16_t &raw_ay, int16_t &raw_az)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_ax = be16s(&buf[0]);
    raw_ay = be16s(&buf[2]);
    raw_az = be16s(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readAccelerometer(float &ax, float &ay, float &az)
{
    int16_t raw_ax, raw_ay, raw_az;
    esp_err_t err = readAccelerometerRaw(raw_ax, raw_ay, raw_az);
    if (err != ESP_OK)
        return err;

    ax = raw_ax / accel_lsb_sensitivity;
    ay = raw_ay / accel_lsb_sensitivity;
    az = raw_az / accel_lsb_sensitivity;
    return ESP_OK;
}

esp_err_t MPU6050::readGyroscopeRaw(int16_t &raw_gx, int16_t &raw_gy, int16_t &raw_gz)
{
    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_GYRO_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_gx = be16s(&buf[0]);
    raw_gy = be16s(&buf[2]);
    raw_gz = be16s(&buf[4]);
    return ESP_OK;
}

esp_err_t MPU6050::readGyroscope(float &gx, float &gy, float &gz)
{
    int16_t raw_gx, raw_gy, raw_gz;
    esp_err_t err = readGyroscopeRaw(raw_gx, raw_gy, raw_gz);
    if (err != ESP_OK)
        return err;

    gx = raw_gx / gyro_lsb_sensitivity;
    gy = raw_gy / gyro_lsb_sensitivity;
    gz = raw_gz / gyro_lsb_sensitivity;
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureRaw(int16_t &raw_temp)
{
    uint8_t buf[2];
    esp_err_t err = i2c.readRegisters(addr, REG_TEMP_OUT_H, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    raw_temp = be16s(&buf[0]);
    return ESP_OK;
}

esp_err_t MPU6050::readTemperatureC(float &temp)
{
    int16_t raw;
    esp_err_t err = readTemperatureRaw(raw);
    if (err != ESP_OK)
        return err;
    temp = raw / 340.0f + 36.53f;
    return ESP_OK;
}

esp_err_t MPU6050::readConfig(uint8_t &conf)
{
    return i2c.readRegister(addr, REG_CONFIG, conf);
}

esp_err_t MPU6050::writeConfig(uint8_t conf)
{
    return i2c.writeRegister(addr, REG_CONFIG, conf);
}

esp_err_t MPU6050::setI2CBypass(bool enable)
{
    uint8_t current_user_ctrl;
    esp_err_t err = i2c.readRegister(addr, REG_USER_CTRL, current_user_ctrl);
    if (err != ESP_OK)
        return err;

    current_user_ctrl &= ~(1 << 5);
    err = i2c.writeRegister(addr, REG_USER_CTRL, current_user_ctrl);
    if (err != ESP_OK)
        return err;

    uint8_t current_int_cfg;
    err = i2c.readRegister(addr, REG_INT_PIN_CFG, current_int_cfg);
    if (err != ESP_OK)
        return err;

    if (enable)
        current_int_cfg |= (1 << 1);
    else
        current_int_cfg &= ~(1 << 1);

    return i2c.writeRegister(addr, REG_INT_PIN_CFG, current_int_cfg);
}