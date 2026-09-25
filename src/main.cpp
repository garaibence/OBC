#include <esp_err.h>
#include <esp_log.h>
#include "SD.hpp"

#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_5

static const char *TAG = "MAIN";

extern "C"
{
    void app_main()
    {
        ESP_LOGI(TAG, "Hello World!");

        SD sd(SPI2_HOST, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS);

        esp_err_t err = sd.mount();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "SD card mounted");
        sd.printCardInfo();

        const char *file_name = "foo.bar";
        const char *contents = "baz";

        err = sd.writeTextFile(file_name, contents);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Wrote '%s' to %s", contents, file_name);
        }
        else
        {
            ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(err));
        }

        char read_buffer[64] = {0};
        size_t bytes_read = 0;
        err = sd.readFile(file_name, read_buffer, sizeof(read_buffer) - 1, &bytes_read);
        if (err == ESP_OK)
        {
            read_buffer[bytes_read] = '\0';
            ESP_LOGI(TAG, "Read back from %s: '%s'", file_name, read_buffer);
        }
        else
        {
            ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(err));
        }

        sd.unmount();
    }
}