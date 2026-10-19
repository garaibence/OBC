#include <cstring>

#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>

#include "WiFiLink.hpp"

static const char *TAG = "MAIN";

// Build one board as 0 (rocket/ping initiator) and the other as 1
// (launchpad/pong responder). Set each peer MAC to the other board's STA MAC.
#define WIFI_LINK_TEST_ROLE 0
#define WIFI_LINK_TEST_CHANNEL 1
// rocket mac
//static const uint8_t WIFI_LINK_PEER_MAC[6] = {0x78, 0x1c, 0x3c, 0x2c, 0x7a, 0x38};
// launchpad mac
static const uint8_t WIFI_LINK_PEER_MAC[6] = {0x6c, 0xc8, 0x40, 0x06, 0xab, 0xc4};

extern "C"
{
    void app_main()
    {
        esp_err_t err = nvs_flash_init();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(err));
            if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
                err == ESP_ERR_NVS_NEW_VERSION_FOUND)
            {
                ESP_LOGE(TAG, "NVS needs repair; erase the NVS partition before retrying");
            }
            return;
        }

        WiFiLink wifi_link(WIFI_LINK_PEER_MAC, WIFI_LINK_TEST_CHANNEL);
        err = wifi_link.start();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "WiFiLink start failed: %s", esp_err_to_name(err));
            return;
        }

        uint8_t own_mac[ESP_NOW_ETH_ALEN];
        err = esp_wifi_get_mac(WIFI_IF_STA, own_mac);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read station MAC: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "Station MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 own_mac[0], own_mac[1], own_mac[2],
                 own_mac[3], own_mac[4], own_mac[5]);

#if WIFI_LINK_TEST_ROLE == 0
        ESP_LOGI(TAG, "WiFiLink test: rocket role, sending PING");
        while (true)
        {
            static const char ping[] = "PING";
            err = wifi_link.send(ping, sizeof(ping) - 1, 1000);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "PING send failed: %s", esp_err_to_name(err));
            }
            else
            {
                uint8_t response[WiFiLink::MAX_MESSAGE_SIZE];
                size_t response_size = 0;
                err = wifi_link.receive(response, sizeof(response), response_size, 2000);
                if (err == ESP_OK &&
                    response_size == 4 &&
                    std::memcmp(response, "PONG", 4) == 0)
                {
                    ESP_LOGI(TAG, "Received PONG from launchpad");
                }
                else if (err == ESP_OK)
                {
                    ESP_LOGW(TAG, "Unexpected response (%u bytes)",
                             static_cast<unsigned>(response_size));
                }
                else
                {
                    ESP_LOGW(TAG, "PONG receive failed: %s", esp_err_to_name(err));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
#elif WIFI_LINK_TEST_ROLE == 1
        ESP_LOGI(TAG, "WiFiLink test: launchpad role, replying PONG");
        while (true)
        {
            uint8_t request[WiFiLink::MAX_MESSAGE_SIZE];
            size_t request_size = 0;
            err = wifi_link.receive(request, sizeof(request), request_size, 5000);
            if (err == ESP_ERR_TIMEOUT)
            {
                ESP_LOGI(TAG, "Waiting for PING");
                continue;
            }
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Receive failed: %s", esp_err_to_name(err));
                continue;
            }
            if (request_size != 4 || std::memcmp(request, "PING", 4) != 0)
            {
                ESP_LOGW(TAG, "Ignoring unexpected message (%u bytes)",
                         static_cast<unsigned>(request_size));
                continue;
            }

            static const char pong[] = "PONG";
            err = wifi_link.send(pong, sizeof(pong) - 1, 1000);
            if (err != ESP_OK)
                ESP_LOGW(TAG, "PONG send failed: %s", esp_err_to_name(err));
            else
                ESP_LOGI(TAG, "Received PING; sent PONG");
        }
#else
#error "WIFI_LINK_TEST_ROLE must be 0 (rocket) or 1 (launchpad)"
#endif
    }
}
