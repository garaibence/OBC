#include "SPIInterface.hpp"
#include "SD.hpp"
#include <cstdint>
#include <esp_err.h>
#include <esp_log.h>
#include <cstring>

static const char *TAG = "SD";

// SD Card Commands
#define CMD0  0   // GO_IDLE_STATE
#define CMD1  1   // SEND_OP_COND
#define CMD8  8   // SEND_IF_COND
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD24 24  // WRITE_SINGLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SD_SEND_OP_COND

#define SD_BLOCK_SIZE 512
#define SD_INIT_TIMEOUT_MS 1000

SD::SD(SPIInterface *spi)
    : spi(spi), initialized(false), card_size(0)
{
}

SD::~SD()
{
    deinit();
}

esp_err_t SD::init()
{
    if (initialized)
        return ESP_OK;

    if (!spi) {
        ESP_LOGE(TAG, "SPI interface not provided");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = spi->init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t dummy[10] = {0xFF};
    spi->transmitOnly(dummy, 10);

    err = sendCommand(CMD0, 0, 0x95);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CMD0 failed");
        return err;
    }

    uint8_t response;
    err = readResponse(&response, 1);
    if (err != ESP_OK || response != 0x01) {
        ESP_LOGE(TAG, "Card not in idle state");
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint32_t timeout = SD_INIT_TIMEOUT_MS;
    while (timeout > 0) {
        sendCommand(CMD55, 0, 0);
        readResponse(&response, 1);

        // Send ACMD41
        err = sendCommand(ACMD41, 0x40000000, 0);
        if (err == ESP_OK) {
            err = readResponse(&response, 1);
            if (response == 0x00) {
                initialized = true;
                ESP_LOGI(TAG, "SD card initialized successfully");
                return ESP_OK;
            }
        }
        timeout--;
    }

    ESP_LOGE(TAG, "SD card initialization timeout");
    return ESP_ERR_TIMEOUT;
}

esp_err_t SD::deinit()
{
    if (!initialized)
        return ESP_OK;

    initialized = false;
    ESP_LOGI(TAG, "SD card deinitialized");
    return ESP_OK;
}

esp_err_t SD::sendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t buffer[6];
    buffer[0] = 0x40 | cmd;
    buffer[1] = (arg >> 24) & 0xFF;
    buffer[2] = (arg >> 16) & 0xFF;
    buffer[3] = (arg >> 8) & 0xFF;
    buffer[4] = arg & 0xFF;
    buffer[5] = crc | 0x01;

    return spi->transmitOnly(buffer, 6);
}

esp_err_t SD::readResponse(uint8_t *response, size_t len)
{
    uint8_t dummy = 0xFF;
    for (int i = 0; i < 10; i++) {
        esp_err_t err = spi->receiveOnly(&dummy, 1);
        if (err != ESP_OK)
            return err;

        if ((dummy & 0x80) == 0) {
            response[0] = dummy;
            for (size_t j = 1; j < len; j++) {
                spi->receiveOnly(&response[j], 1);
            }
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "No response from SD card");
    return ESP_ERR_TIMEOUT;
}

esp_err_t SD::waitForReady(uint32_t timeout_ms)
{
    uint8_t response;
    for (uint32_t i = 0; i < timeout_ms; i++) {
        esp_err_t err = spi->receiveOnly(&response, 1);
        if (err == ESP_OK && response == 0xFF) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t SD::readBlock(uint32_t block_addr, uint8_t *buffer, size_t len)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sendCommand(CMD17, block_addr, 0xFF);
    if (err != ESP_OK)
        return err;

    uint8_t response;
    err = readResponse(&response, 1);
    if (err != ESP_OK || response != 0x00) {
        ESP_LOGE(TAG, "Read failed: %02X", response);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (waitForReady(1000) != ESP_OK) {
        ESP_LOGE(TAG, "Data token timeout");
        return ESP_ERR_TIMEOUT;
    }

    return spi->receiveOnly(buffer, len);
}

esp_err_t SD::writeBlock(uint32_t block_addr, const uint8_t *buffer, size_t len)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SD card not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sendCommand(CMD24, block_addr, 0xFF);
    if (err != ESP_OK)
        return err;

    uint8_t response;
    err = readResponse(&response, 1);
    if (err != ESP_OK || response != 0x00) {
        ESP_LOGE(TAG, "Write failed: %02X", response);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t token = 0xFE;
    spi->transmitOnly(&token, 1);
    spi->transmitOnly(buffer, len);

    uint8_t crc[2] = {0xFF, 0xFF};
    spi->transmitOnly(crc, 2);

    return waitForReady(1000);
}
