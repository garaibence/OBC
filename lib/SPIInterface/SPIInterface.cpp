#include "SPIInterface.hpp"
#include <esp_log.h>
#include <cstring>
#include <vector>

static const char *TAG = "SPI_IF";

// LoRa-style register convention: bit 7 of the address byte is set for writes, clear for reads
static constexpr uint8_t ADDR_WRITE_FLAG = 0x80;
// The LoRa module does not need a fast SPI clock, and the driver only uses short bursts
static constexpr uint32_t CLK_SPEED_MAX_HZ = 8000000;

esp_err_t SPIInterface::init()
{
    if (installed)
        return ESP_OK;

    if (clk > CLK_SPEED_MAX_HZ)
    {
        ESP_LOGW(TAG, "Clamping SPI clock to %d Hz", static_cast<int>(CLK_SPEED_MAX_HZ));
        clk = CLK_SPEED_MAX_HZ;
    }

    spi_device_interface_config_t dev_cfg{};
    dev_cfg.clock_speed_hz = static_cast<int>(clk);
    dev_cfg.spics_io_num = cs;
    dev_cfg.mode = 0;
    dev_cfg.queue_size = 4;

    esp_err_t err = spi_bus_add_device(host_id, &dev_cfg, &device);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }
    installed = true;
    return ESP_OK;
}

esp_err_t SPIInterface::deinit()
{
    if (!installed)
        return ESP_OK;
    if (device == nullptr)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = spi_bus_remove_device(device);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_remove_device failed: %s", esp_err_to_name(err));
        return err;
    }
    device = nullptr;
    installed = false;
    return ESP_OK;
}

esp_err_t SPIInterface::writeRegister(uint8_t reg_addr, uint8_t data)
{
    return writeRegisters(reg_addr, &data, 1);
}

esp_err_t SPIInterface::writeRegisters(uint8_t reg_addr, const uint8_t *data, size_t len)
{
    if (!installed || device == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (len == 0)
        return ESP_OK;

    std::vector<uint8_t> tx(len + 1);
    tx[0] = reg_addr | ADDR_WRITE_FLAG;
    memcpy(&tx[1], data, len);

    spi_transaction_t t{};
    t.length = static_cast<int>(len + 1) * 8;
    t.tx_buffer = tx.data();

    esp_err_t err = spi_device_transmit(device, &t);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "spi_device_transmit failed (write 0x%02x, %zu bytes): %s",
                 static_cast<int>(reg_addr), len, esp_err_to_name(err));
    return err;
}

esp_err_t SPIInterface::readRegister(uint8_t reg_addr, uint8_t &data)
{
    return readRegisters(reg_addr, &data, 1);
}

esp_err_t SPIInterface::readRegisters(uint8_t reg_addr, uint8_t *data, size_t len)
{
    if (!installed || device == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (len == 0)
        return ESP_OK;

    // ESP-IDF rejects rxlength > length in full duplex, so send the address
    // followed by len zero bytes and read the same number of bytes back
    std::vector<uint8_t> tx(len + 1, 0);
    tx[0] = reg_addr & ~ADDR_WRITE_FLAG;

    std::vector<uint8_t> rx(len + 1);
    spi_transaction_t t{};
    t.length = static_cast<int>(len + 1) * 8;
    t.tx_buffer = tx.data();
    t.rx_buffer = rx.data();

    esp_err_t err = spi_device_transmit(device, &t);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "spi_device_transmit failed (read 0x%02x, %zu bytes): %s",
                 static_cast<int>(reg_addr), len, esp_err_to_name(err));
    else
        memcpy(data, &rx[1], len);
    return err;
}
