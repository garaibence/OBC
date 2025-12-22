#include "SPIInterface.hpp"
#include "esp_log.h"

static const char *TAG = "SPIInterface";

SPIInterface::SPIInterface(spi_host_device_t host_id, int mosi, int miso, int sclk)
    : host_id(host_id), mosi(mosi), miso(miso), sclk(sclk), is_initialized(false) {}

SPIInterface::~SPIInterface()
{
    deinit();
}

esp_err_t SPIInterface::init()
{
    if (is_initialized)
        return ESP_OK;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = mosi;
    bus_cfg.miso_io_num = miso;
    bus_cfg.sclk_io_num = sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    esp_err_t ret = spi_bus_initialize(host_id, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK)
    {
        is_initialized = true;
        ESP_LOGI(TAG, "SPI bus initialized successfully");
    }
    else
        ESP_LOGE(TAG, "SPI bus failed to initialize");
    return ret;
}

esp_err_t SPIInterface::deinit()
{
    if (!is_initialized)
        return ESP_OK;

    esp_err_t ret = spi_bus_free(host_id);
    if (ret == ESP_OK)
    {
        is_initialized = false;
        ESP_LOGI(TAG, "SPI bus freed");
    }
    else
        ESP_LOGE(TAG, "Failed to free SPI bus");
    return ret;
}