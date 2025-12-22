#include <esp_log.h>
#include <esp_err.h>
#include <cstring>
#include <cstdio>

#include "SPIInterface.hpp"
#include "SD.hpp"

extern "C" void app_main(void)
{
    static const char *TAG = "MAIN";
    ESP_LOGI(TAG, "Starting...");

    SPIInterface spi(SPI2_HOST, GPIO_NUM_23, GPIO_NUM_19, GPIO_NUM_18); // MOSI, MISO, CLK
    if (spi.init() != ESP_OK)
        return;

    SD sd(spi, "/sdcard", GPIO_NUM_5); // mount, CS

    if (sd.mount(true) == ESP_OK)
    {
        sd.print_info();

        sd.write_file("/hello.txt", "Hello from C++ Object!");
        std::string content = sd.read_file("/hello.txt");

        printf("Read from SD: %s\n", content.c_str());

        sd.unmount();
    }
}
