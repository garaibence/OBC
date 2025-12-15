#include "BMP180.hpp"
#include "ByteUtils.hpp"
#include <cmath>

esp_err_t BMP180::init()
{
    uint8_t reset = 0xB6;
    esp_err_t err = i2c.writeRegister(addr, REG_RESET, reset);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));

    return readCalibration();
}

esp_err_t BMP180::whoami(uint8_t &id)
{
    return i2c.readRegister(addr, REG_ID, id);
}

esp_err_t BMP180::readCalibration()
{
    uint8_t buf[22];
    esp_err_t err = i2c.readRegisters(addr, 0xAA, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;

    AC1 = be16s(&buf[0]);
    AC2 = be16s(&buf[2]);
    AC3 = be16s(&buf[4]);
    AC4 = be16u(&buf[6]);
    AC5 = be16u(&buf[8]);
    AC6 = be16u(&buf[10]);
    B1 = be16s(&buf[12]);
    B2 = be16s(&buf[14]);
    MB = be16s(&buf[16]);
    MC = be16s(&buf[18]);
    MD = be16s(&buf[20]);

    return ESP_OK;
}

esp_err_t BMP180::waitForConversion()
{
    uint8_t ctrl;
    do
    {
        esp_err_t err = i2c.readRegister(addr, REG_CTRL_MEAS, ctrl);
        if (err != ESP_OK)
            return err;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (ctrl & 0x20); // SCO bit

    return ESP_OK;
}

void BMP180::setOversampling(Oversampling newOss)
{
    oss = newOss;
}

esp_err_t BMP180::readTemperatureRaw(uint16_t &ut)
{
    esp_err_t err = i2c.writeRegister(addr, REG_CTRL_MEAS, CMD_TEMP);
    if (err != ESP_OK)
        return err;

    err = waitForConversion();
    if (err != ESP_OK)
        return err;

    uint8_t buf[2];
    err = i2c.readRegisters(addr, REG_OUT_MSB, buf, 2);
    if (err != ESP_OK)
        return err;

    ut = (uint16_t(buf[0]) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t BMP180::readTemperature(float &temp_c)
{
    uint16_t ut;
    esp_err_t err = readTemperatureRaw(ut);
    if (err != ESP_OK)
        return err;

    int32_t X1 = ((int32_t)ut - (int32_t)AC6) * (int32_t)AC5 >> 15;
    int32_t X2 = ((int32_t)MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    int32_t T = (B5 + 8) >> 4;
    temp_c = T / 10.0f;

    hasTemp = true;
    return ESP_OK;
}

esp_err_t BMP180::readPressureRaw(uint32_t &up)
{
    uint8_t cmd = pressureCommand();

    esp_err_t err = i2c.writeRegister(addr, REG_CTRL_MEAS, cmd);
    if (err != ESP_OK)
        return err;

    err = waitForConversion();
    if (err != ESP_OK)
        return err;

    uint8_t buf[3] = {};
    err = i2c.readRegisters(addr, REG_OUT_MSB, buf, 3);
    if (err != ESP_OK)
        return err;

    up = ((uint32_t)buf[0] << 16 |
          (uint32_t)buf[1] << 8 |
          (uint32_t)buf[2]) >>
         (8 - static_cast<uint8_t>(oss));

    return ESP_OK;
}

esp_err_t BMP180::readPressure(float &pressure_pa)
{
    if (!hasTemp)
    {
        float dummy;
        esp_err_t err = readTemperature(dummy);
        if (err != ESP_OK)
            return err;
    }

    uint32_t up;
    esp_err_t err = readPressureRaw(up);
    if (err != ESP_OK)
        return err;

    int32_t B6 = B5 - 4000;

    int32_t X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;

    uint8_t oss_val = static_cast<uint8_t>(oss);

    int32_t B3 = (((AC1 * 4 + X3) << oss_val) + 2) >> 2;
    uint32_t B7 = (up - B3) * (50000 >> oss_val);

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;

    int32_t p;
    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p += (X1 + X2 + 3791) >> 4;

    pressure_pa = (float)p;
    return ESP_OK;
}

esp_err_t BMP180::readAltitude(float &alt, float seaLvlPres)
{
    float pres;
    esp_err_t err = readPressure(pres);
    if (err != ESP_OK)
        return err;

    alt = 44330.0f * (1.0f - powf(pres / seaLvlPres, 0.19029495f));

    return ESP_OK;
}