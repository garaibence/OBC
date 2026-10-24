#pragma once

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_err.h>
#include <cstdint>
#include <cstddef>

class SPIInterface
{
public:
    SPIInterface(spi_host_device_t host, gpio_num_t cs_gpio,
                 uint32_t clk_speed_hz = 1000000)
        : host_id(host), cs(cs_gpio), clk(clk_speed_hz), device(nullptr), installed(false) {}

    ~SPIInterface() { deinit(); }

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t writeRegister(uint8_t reg_addr, uint8_t data);
    esp_err_t readRegister(uint8_t reg_addr, uint8_t &data);
    esp_err_t writeRegisters(uint8_t reg_addr, const uint8_t *data, size_t len);
    esp_err_t readRegisters(uint8_t reg_addr, uint8_t *data, size_t len);

private:
    spi_host_device_t host_id;
    gpio_num_t cs;
    uint32_t clk;
    spi_device_handle_t device;
    bool installed;
};
