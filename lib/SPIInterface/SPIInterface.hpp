#pragma once

#include <driver/spi_master.h>
#include <soc/gpio_num.h>
#include <esp_err.h>
#include <cstdint>
#include <vector>

class SPIInterface
{
public:
    SPIInterface(spi_host_device_t host, gpio_num_t mosi_gpio, gpio_num_t miso_gpio,
                 gpio_num_t clk_gpio, gpio_num_t cs_gpio, uint32_t freq_hz = 10000000,
                 int timeout_ms = 1000)
        : host(host), mosi(mosi_gpio), miso(miso_gpio), clk(clk_gpio), cs(cs_gpio),
          freq(freq_hz), timeout_ticks(pdMS_TO_TICKS(timeout_ms)),
          spi_handle(nullptr), installed(false) {}

    ~SPIInterface() { deinit(); }

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t transmit(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
    esp_err_t transmitOnly(const uint8_t *tx_data, size_t len);
    esp_err_t receiveOnly(uint8_t *rx_data, size_t len);

private:
    spi_host_device_t host;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t clk;
    gpio_num_t cs;
    uint32_t freq;
    TickType_t timeout_ticks;
    spi_device_handle_t spi_handle;
    bool installed;
};