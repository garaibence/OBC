#pragma once

#include "SPIInterface.hpp"
#include <cstdint>
#include <esp_err.h>

class SD
{
public:
    SD(SPIInterface *spi);
    ~SD();

    // Initialize SD card
    esp_err_t init();
    esp_err_t deinit();

    // Card operations
    esp_err_t readBlock(uint32_t block_addr, uint8_t *buffer, size_t len);
    esp_err_t writeBlock(uint32_t block_addr, const uint8_t *buffer, size_t len);

    // Status
    bool isInitialized() const { return initialized; }
    uint32_t getCardSize() const { return card_size; }

private:
    SPIInterface *spi;
    bool initialized;
    uint32_t card_size;

    // Helper methods
    esp_err_t sendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
    esp_err_t readResponse(uint8_t *response, size_t len);
    esp_err_t waitForReady(uint32_t timeout_ms);
};