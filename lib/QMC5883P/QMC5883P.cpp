#include "QMC5883P.hpp"
#include "ByteUtils.hpp"

esp_err_t QMC5883P::init()
{
    uint8_t reset = 0x80;
    esp_err_t err = i2c.writeRegister(addr, REG_CTRL_2, reset);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(100));

    err = i2c.writeRegister(addr, REG_CTRL_2, 0x00);
    if (err != ESP_OK)
        return err;

    return setConfig(QMC_ODR::ODR_200Hz, QMC_RNG::RNG_8G, QMC_OSR::OSR_512, QMC_MODE::MODE_CONTINUOUS);
}

esp_err_t QMC5883P::whoami(uint8_t &id)
{
    return i2c.readRegister(addr, REG_ID, id);
}

esp_err_t QMC5883P::setConfig(QMC_ODR odr, QMC_RNG rng, QMC_OSR osr, QMC_MODE mode)
{
    uint8_t ctrl1_val = static_cast<uint8_t>(osr) |
                        static_cast<uint8_t>(odr) |
                        static_cast<uint8_t>(mode);

    esp_err_t err = i2c.writeRegister(addr, REG_CTRL_1, ctrl1_val);
    if (err != ESP_OK)
        return err;

    uint8_t ctrl2_val = static_cast<uint8_t>(rng);

    err = i2c.writeRegister(addr, REG_CTRL_2, ctrl2_val);
    if (err != ESP_OK)
        return err;


    scale = rngToScale(rng);

    return ESP_OK;
}

esp_err_t QMC5883P::setMode(QMC_MODE mode)
{
    uint8_t ctrl;
    esp_err_t err = i2c.readRegister(addr, REG_CTRL_1, ctrl);
    if (err != ESP_OK)
        return err;

    ctrl &= 0xFC;
    ctrl |= static_cast<uint8_t>(mode);

    return i2c.writeRegister(addr, REG_CTRL_1, ctrl);
}

esp_err_t QMC5883P::setOutputDataRate(QMC_ODR odr)
{
    uint8_t ctrl;
    esp_err_t err = i2c.readRegister(addr, REG_CTRL_1, ctrl);
    if (err != ESP_OK)
        return err;

    ctrl &= 0xF3;
    ctrl |= static_cast<uint8_t>(odr);

    return i2c.writeRegister(addr, REG_CTRL_1, ctrl);
}

esp_err_t QMC5883P::setOversamplingRatio(QMC_OSR osr)
{
    uint8_t ctrl;
    esp_err_t err = i2c.readRegister(addr, REG_CTRL_1, ctrl);
    if (err != ESP_OK)
        return err;

    ctrl &= 0x3F;
    ctrl |= static_cast<uint8_t>(osr);

    return i2c.writeRegister(addr, REG_CTRL_1, ctrl);
}

esp_err_t QMC5883P::setRange(QMC_RNG rng)
{
    uint8_t ctrl;
    esp_err_t err = i2c.readRegister(addr, REG_CTRL_2, ctrl);
    if (err != ESP_OK)
        return err;

    ctrl &= 0xF3;
    ctrl |= static_cast<uint8_t>(rng);

    scale = rngToScale(rng);

    return i2c.writeRegister(addr, REG_CTRL_2, ctrl);
}

esp_err_t QMC5883P::readStatus(bool &rdy)
{
    uint8_t status;
    esp_err_t err = i2c.readRegister(addr, REG_STATUS, status);
    if (err != ESP_OK)
        return err;

    rdy = (status & 0x01);
    return ESP_OK;
}

esp_err_t QMC5883P::readMagnetoRaw(int16_t &raw_x, int16_t &raw_y, int16_t &raw_z)
{
    bool ready;
    do
    {
        esp_err_t err = readStatus(ready);
        if (err != ESP_OK)
            return err;

        vTaskDelay(pdMS_TO_TICKS(1));

    } while (!ready);

    uint8_t buf[6];
    esp_err_t err = i2c.readRegisters(addr, REG_DATA_X_LSB, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;

    raw_x = le16s(&buf[0]);
    raw_y = le16s(&buf[2]);
    raw_z = le16s(&buf[4]);

    return ESP_OK;
}

esp_err_t QMC5883P::readMagneto(float &mag_x, float &mag_y, float &mag_z)
{
    int16_t rx, ry, rz;
    esp_err_t err = readMagnetoRaw(rx, ry, rz);
    if (err != ESP_OK)
        return err;

    mag_x = rx * scale * 100.0f;
    mag_y = ry * scale * 100.0f;
    mag_z = rz * scale * 100.0f;

    return ESP_OK;
}