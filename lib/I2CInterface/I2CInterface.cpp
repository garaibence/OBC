#include "I2CInterface.hpp"
#include <esp_log.h>
#include <cstring>

static const char *TAG = "I2C_IF";

esp_err_t I2CInterface::init()
{
    if (installed)
        return ESP_OK;

    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = clk;

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_param_config failed: %d", err);
        return err;
    }
    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_driver_install failed: %d", err);
        return err;
    }
    installed = true;
    return ESP_OK;
}

esp_err_t I2CInterface::deinit()
{
    if (!installed)
        return ESP_OK;
    esp_err_t err = i2c_driver_delete(port);
    if (err == ESP_OK)
        installed = false;
    return err;
}

esp_err_t I2CInterface::writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return writeRegisters(dev_addr, reg_addr, &data, 1);
}

esp_err_t I2CInterface::writeRegisters(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    if (len)
    {
        i2c_master_write(cmd, const_cast<uint8_t *>(data), len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout_ticks);
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t I2CInterface::readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t &data)
{
    return readRegisters(dev_addr, reg_addr, &data, 1);
}

esp_err_t I2CInterface::readRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;
    if (len == 0)
        return ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + (len - 1), I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout_ticks);
    i2c_cmd_link_delete(cmd);
    return err;
}

std::vector<uint8_t> I2CInterface::scan(uint8_t start_addr, uint8_t end_addr)
{
    std::vector<uint8_t> found;
    if (!installed)
        return found;
    for (uint8_t addr = start_addr; addr <= end_addr; ++addr)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, timeout_ticks);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK)
        {
            found.push_back(addr);
        }
    }
    return found;
}