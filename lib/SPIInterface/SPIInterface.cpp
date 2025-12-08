#include "SPIInterface.hpp"
#include <esp_log.h>

static const char *TAG = "SPI_IF";

esp_err_t SPIInterface::init()
{
    if (installed)
        return ESP_OK;

    spi_bus_config_t bus_cfg{};
    bus_cfg.mosi_io_num = mosi;
    bus_cfg.miso_io_num = miso;
    bus_cfg.sclk_io_num = clk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4092;
    bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;
    bus_cfg.intr_flags = 0;

    esp_err_t err = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg{};
    dev_cfg.command_bits = 0;
    dev_cfg.address_bits = 0;
    dev_cfg.dummy_bits = 0;
    dev_cfg.mode = 0;
    dev_cfg.duty_cycle_pos = 128;
    dev_cfg.cs_ena_pretrans = 0;
    dev_cfg.cs_ena_posttrans = 0;
    dev_cfg.clock_speed_hz = freq;
    dev_cfg.input_delay_ns = 0;
    dev_cfg.spics_io_num = cs;
    dev_cfg.flags = 0;
    dev_cfg.queue_size = 7;
    dev_cfg.pre_cb = nullptr;
    dev_cfg.post_cb = nullptr;

    err = spi_bus_add_device(host, &dev_cfg, &spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(err));
        spi_bus_free(host);
        return err;
    }

    installed = true;
    ESP_LOGI(TAG, "SPI interface initialized successfully");
    return ESP_OK;
}

esp_err_t SPIInterface::deinit()
{
    if (!installed)
        return ESP_OK;

    esp_err_t err = spi_bus_remove_device(spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(err));
        return err;
    }

    err = spi_bus_free(host);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(err));
        return err;
    }

    installed = false;
    spi_handle = nullptr;
    ESP_LOGI(TAG, "SPI interface deinitialized");
    return ESP_OK;
}

esp_err_t SPIInterface::transmit(const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    if (!installed) {
        ESP_LOGE(TAG, "SPI interface not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t trans{};
    trans.flags = 0;
    trans.cmd = 0;
    trans.addr = 0;
    trans.length = len * 8;
    trans.rxlength = 0;
    trans.user = nullptr;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    esp_err_t err = spi_device_transmit(spi_handle, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t SPIInterface::transmitOnly(const uint8_t *tx_data, size_t len)
{
    return transmit(tx_data, nullptr, len);
}

esp_err_t SPIInterface::receiveOnly(uint8_t *rx_data, size_t len)
{
    return transmit(nullptr, rx_data, len);
}