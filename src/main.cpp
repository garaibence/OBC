#include <esp_log.h>
#include "SD.hpp"
#include "SPIInterface.hpp"
#include <cstring>

extern "C"
{
    void app_main()
    {
        static const char *TAG = "MAIN";
        ESP_LOGD(TAG, "Hello World!");

        SPIInterface spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, 
                        GPIO_NUM_18, GPIO_NUM_5, 10000000);

        SD sd_card(&spi);
        esp_err_t err = sd_card.init();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "SD card initialized successfully");
        } else {
            ESP_LOGE(TAG, "SD card initialization failed: %s", esp_err_to_name(err));
            return;
        }

        uint8_t write_buffer[512];
        const char *test_data = "Hello SD Card! This is a test write.";
        memset(write_buffer, 0, sizeof(write_buffer));
        memcpy(write_buffer, test_data, strlen(test_data));

        err = sd_card.writeBlock(0, write_buffer, 512);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Write successful");
        } else {
            ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(err));
        }

        uint8_t read_buffer[512];
        err = sd_card.readBlock(0, read_buffer, 512);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Read successful: %s", (char *)read_buffer);
        } else {
            ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(err));
        }
    }
}