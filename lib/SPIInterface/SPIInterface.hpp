#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

class SPIInterface
{
public:
    SPIInterface(spi_host_device_t host_id, int mosi, int miso, int sclk);
    ~SPIInterface();

    esp_err_t init();
    esp_err_t deinit();

    spi_host_device_t get_host_id() const { return host_id; }

private:
    spi_host_device_t host_id;
    int mosi, miso, sclk;
    bool is_initialized;
};