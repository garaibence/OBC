#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SPIInterface.hpp"
#include "SX1278.hpp"

#define PIN_NUM_MOSI      GPIO_NUM_23
#define PIN_NUM_MISO      GPIO_NUM_19
#define PIN_NUM_CLK       GPIO_NUM_18
#define PIN_NUM_LORA_CS   GPIO_NUM_27
#define PIN_NUM_LORA_RST  GPIO_NUM_32
#define PIN_NUM_LORA_DIO0 GPIO_NUM_26
#define PIN_NUM_SD_CS     GPIO_NUM_5

// Select exactly one demo: uncomment one of these two defines
#define LORA_DEMO_TX
//#define LORA_DEMO_RX

static const char *TAG = "MAIN";

#ifdef LORA_DEMO_TX
// TX demo: send "hello <counter>" once per second, waiting for TX_DONE on DIO0
static void lora_demo_tx(SX1278 &lora)
{
    uint32_t counter = 0;
    while (true)
    {
        char payload[32];
        int len = std::snprintf(payload, sizeof(payload), "hello %d", static_cast<int>(counter));

        esp_err_t err = lora.beginPacket();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "beginPacket failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        err = lora.write(reinterpret_cast<const uint8_t *>(payload), static_cast<size_t>(len));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "write failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // async TX: completion arrives as a DIO0 event
        err = lora.endPacket(true);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "endPacket failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint8_t event = 0;
        err = lora.waitEvent(2000, event);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "TX of %d bytes done", len);
        else
            ESP_LOGE(TAG, "No TX_DONE event: %s", esp_err_to_name(err));

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#ifdef LORA_DEMO_RX
// RX demo: wait for incoming packets and log payload, RSSI and SNR
static void lora_demo_rx(SX1278 &lora)
{
    while (true)
    {
        esp_err_t err = lora.parsePacket();
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT && err != ESP_ERR_INVALID_CRC)
            ESP_LOGE(TAG, "parsePacket failed: %s", esp_err_to_name(err));

        if (err == ESP_OK)
        {
            size_t len = 0;
            err = lora.available(len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "available failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            char payload[256];
            for (size_t i = 0; i < len && i < sizeof(payload) - 1; i++)
            {
                uint8_t data = 0;
                err = lora.read(data);
                if (err != ESP_OK)
                {
                    payload[i] = '\0';
                    break;
                }
                payload[i] = static_cast<char>(data);
            }
            payload[len < sizeof(payload) ? len : sizeof(payload) - 1] = '\0';

            int rssi = 0;
            float snr = 0.0f;
            lora.packetRssi(rssi);
            lora.packetSnr(snr);
            ESP_LOGI(TAG, "RX %d bytes: %s, RSSI %d dB, SNR %.2f dB",
                     static_cast<int>(len), payload, rssi, snr);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

extern "C"
{
    void app_main()
    {
        // Both CS lines must be configured as outputs and driven high before
        // either SPI device is attached to the bus
        gpio_config_t cs_cfg{};
        cs_cfg.pin_bit_mask = (1ULL << PIN_NUM_LORA_CS) | (1ULL << PIN_NUM_SD_CS);
        cs_cfg.mode = GPIO_MODE_OUTPUT;
        cs_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cs_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cs_cfg.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&cs_cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "gpio_config failed for CS pins: %s", esp_err_to_name(err));
            return;
        }
        gpio_set_level(PIN_NUM_LORA_CS, 1);
        gpio_set_level(PIN_NUM_SD_CS, 1);

        // The SPI bus is shared by the LoRa module and the SD card: initialize it once.
        spi_bus_config_t bus_cfg{};
        bus_cfg.mosi_io_num = PIN_NUM_MOSI;
        bus_cfg.miso_io_num = PIN_NUM_MISO;
        bus_cfg.sclk_io_num = PIN_NUM_CLK;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        // Enough for the longest LoRa packet (255 bytes) plus the address byte
        bus_cfg.max_transfer_sz = 256;

        err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            return;
        }

        // NOTE: lib/SD/SD.cpp::mount() calls spi_bus_initialize() again on this bus.
        // If the SD card is used here too, that second init would fail - see the
        // conflict note; do not call SD::mount() until it is resolved.

        SPIInterface lora_spi(SPI2_HOST, PIN_NUM_LORA_CS);
        SX1278 lora(lora_spi, PIN_NUM_LORA_RST, PIN_NUM_LORA_DIO0);

        err = lora.init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "lora init failed: %s", esp_err_to_name(err));
            return;
        }

        // Radio-friendly default frequency for the SX1278
        err = lora.begin(433000000);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "lora begin failed: %s", esp_err_to_name(err));
            return;
        }

#ifdef LORA_DEMO_TX
        lora_demo_tx(lora);
#elif defined(LORA_DEMO_RX)
        lora_demo_rx(lora);
#else
#error "Define either LORA_DEMO_TX or LORA_DEMO_RX"
#endif
    }
}
