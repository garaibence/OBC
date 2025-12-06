#pragma once

#include <driver/i2c.h>
#include <esp_err.h>
#include <cstdint>
#include <vector>

class I2CInterface
{
public:
    I2CInterface(i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio,
                 uint32_t clk_speed_hz = 100000, int timeout_ms = 1000)
        : port(port), sda(sda_gpio), scl(scl_gpio),
          clk(clk_speed_hz), timeout_ticks(pdMS_TO_TICKS(timeout_ms)),
          installed(false) {}

    ~I2CInterface() { deinit(); }

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
    esp_err_t writeRegisters(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);
    esp_err_t readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t &data);
    esp_err_t readRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

    std::vector<uint8_t> scan(uint8_t start_addr = 1, uint8_t end_addr = 127);

private:
    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t clk;
    TickType_t timeout_ticks;
    bool installed;
};