#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <esp_err.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

class WiFiLink
{
public:
    static constexpr size_t MAX_MESSAGE_SIZE = ESP_NOW_MAX_DATA_LEN;

    WiFiLink(const uint8_t *peer_mac, uint8_t channel = 1);
    ~WiFiLink();

    WiFiLink(const WiFiLink &) = delete;
    WiFiLink &operator=(const WiFiLink &) = delete;

    esp_err_t start();
    esp_err_t stop();
    bool isStarted() const { return started; }
    uint32_t getDroppedMessageCount() const { return dropped_messages.load(); }

    esp_err_t send(const void *data, size_t length, int timeout_ms = 1000);
    esp_err_t receive(void *buffer, size_t buffer_size, size_t &received_size,
                      int timeout_ms = -1);

private:
    struct Message
    {
        size_t length;
        uint8_t data[MAX_MESSAGE_SIZE];
    };

    static void receiveCallback(const esp_now_recv_info_t *info,
                                const uint8_t *data, int data_length);
    static void sendCallback(const esp_now_send_info_t *info,
                             esp_now_send_status_t status);

    bool isValidPeerMac() const;
    esp_err_t cleanup();

    std::array<uint8_t, ESP_NOW_ETH_ALEN> peer_mac;
    uint8_t channel;
    QueueHandle_t receive_queue;
    SemaphoreHandle_t send_mutex;
    SemaphoreHandle_t send_complete;
    std::atomic<bool> awaiting_send_result;
    std::atomic<esp_now_send_status_t> send_status;
    std::atomic<uint32_t> dropped_messages;
    bool wifi_initialized;
    bool wifi_started;
    bool esp_now_initialized;
    bool peer_added;
    bool started;

    static WiFiLink *active_link;
};
