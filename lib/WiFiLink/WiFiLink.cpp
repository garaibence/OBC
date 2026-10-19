#include "WiFiLink.hpp"

#include <cstring>

#include <esp_log.h>
#include <esp_wifi.h>

static const char *TAG = "WIFI_LINK";

WiFiLink *WiFiLink::active_link = nullptr;

WiFiLink::WiFiLink(const uint8_t *peer_mac, uint8_t channel)
    : peer_mac{}, channel(channel), receive_queue(nullptr), send_mutex(nullptr),
      send_complete(nullptr), awaiting_send_result(false), send_status(ESP_NOW_SEND_FAIL),
      dropped_messages(0), wifi_initialized(false), wifi_started(false), esp_now_initialized(false),
      peer_added(false), started(false)
{
    if (peer_mac != nullptr)
        std::memcpy(this->peer_mac.data(), peer_mac, this->peer_mac.size());
}

WiFiLink::~WiFiLink()
{
    stop();
}

bool WiFiLink::isValidPeerMac() const
{
    bool all_zero = true;
    bool all_broadcast = true;
    for (uint8_t byte : peer_mac)
    {
        all_zero = all_zero && byte == 0;
        all_broadcast = all_broadcast && byte == 0xFF;
    }
    return !all_zero && !all_broadcast && (peer_mac[0] & 0x01) == 0;
}

esp_err_t WiFiLink::start()
{
    if (started)
        return ESP_OK;
    if (!isValidPeerMac() || channel < 1 || channel > 13)
        return ESP_ERR_INVALID_ARG;
    if (active_link != nullptr && active_link != this)
        return ESP_ERR_INVALID_STATE;

    receive_queue = xQueueCreate(4, sizeof(Message));
    send_mutex = xSemaphoreCreateMutex();
    send_complete = xSemaphoreCreateBinary();
    if (receive_queue == nullptr || send_mutex == nullptr || send_complete == nullptr)
    {
        cleanup();
        return ESP_ERR_NO_MEM;
    }

    active_link = this;

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&wifi_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi initialization failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }
    wifi_initialized = true;

    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err == ESP_OK)
        err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK)
        err = esp_wifi_start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi startup failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }
    wifi_started = true;

    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err == ESP_OK)
        err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Wi-Fi configuration failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }

    err = esp_now_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW initialization failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }
    esp_now_initialized = true;

    err = esp_now_register_recv_cb(receiveCallback);
    if (err == ESP_OK)
        err = esp_now_register_send_cb(sendCallback);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW callback registration failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }

    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, peer_mac.data(), peer_mac.size());
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    err = esp_now_add_peer(&peer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Adding ESP-NOW peer failed: %s", esp_err_to_name(err));
        cleanup();
        return err;
    }
    peer_added = true;
    started = true;
    ESP_LOGI(TAG, "ESP-NOW link started on channel %u", channel);
    return ESP_OK;
}

esp_err_t WiFiLink::stop()
{
    return cleanup();
}

esp_err_t WiFiLink::cleanup()
{
    esp_err_t first_error = ESP_OK;

    if (active_link == this)
    {
        active_link = nullptr;
        if (esp_now_initialized)
        {
            esp_err_t err = esp_now_unregister_recv_cb();
            if (err != ESP_OK && first_error == ESP_OK)
                first_error = err;
            err = esp_now_unregister_send_cb();
            if (err != ESP_OK && first_error == ESP_OK)
                first_error = err;
        }
    }

    if (peer_added)
    {
        esp_err_t err = esp_now_del_peer(peer_mac.data());
        if (err != ESP_OK && first_error == ESP_OK)
            first_error = err;
        peer_added = false;
    }
    if (esp_now_initialized)
    {
        esp_err_t err = esp_now_deinit();
        if (err != ESP_OK && first_error == ESP_OK)
            first_error = err;
        esp_now_initialized = false;
    }
    if (wifi_started)
    {
        esp_err_t err = esp_wifi_stop();
        if (err != ESP_OK && first_error == ESP_OK)
            first_error = err;
        wifi_started = false;
    }
    if (wifi_initialized)
    {
        esp_err_t err = esp_wifi_deinit();
        if (err != ESP_OK && first_error == ESP_OK)
            first_error = err;
        wifi_initialized = false;
    }

    if (receive_queue != nullptr)
    {
        vQueueDelete(receive_queue);
        receive_queue = nullptr;
    }
    if (send_mutex != nullptr)
    {
        vSemaphoreDelete(send_mutex);
        send_mutex = nullptr;
    }
    if (send_complete != nullptr)
    {
        vSemaphoreDelete(send_complete);
        send_complete = nullptr;
    }

    awaiting_send_result.store(false);
    started = false;
    return first_error;
}

esp_err_t WiFiLink::send(const void *data, size_t length, int timeout_ms)
{
    if (!started)
        return ESP_ERR_INVALID_STATE;
    if ((data == nullptr && length != 0) || length == 0 ||
        length > MAX_MESSAGE_SIZE || timeout_ms <= 0)
        return ESP_ERR_INVALID_ARG;

    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    if (timeout == 0)
        timeout = 1;
    if (xSemaphoreTake(send_mutex, timeout) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    if (awaiting_send_result.load())
    {
        xSemaphoreGive(send_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(send_complete, 0) == pdTRUE)
    {
    }
    awaiting_send_result.store(true);
    esp_err_t err = esp_now_send(peer_mac.data(),
                                 static_cast<const uint8_t *>(data), length);
    if (err != ESP_OK)
    {
        awaiting_send_result.store(false);
        xSemaphoreGive(send_mutex);
        return err;
    }

    if (xSemaphoreTake(send_complete, timeout) != pdTRUE)
    {
        xSemaphoreGive(send_mutex);
        return ESP_ERR_TIMEOUT;
    }

    err = send_status.load() == ESP_NOW_SEND_SUCCESS ? ESP_OK : ESP_FAIL;
    xSemaphoreGive(send_mutex);
    return err;
}

esp_err_t WiFiLink::receive(void *buffer, size_t buffer_size, size_t &received_size,
                            int timeout_ms)
{
    received_size = 0;
    if (!started)
        return ESP_ERR_INVALID_STATE;
    if (timeout_ms < -1)
        return ESP_ERR_INVALID_ARG;
    if (buffer == nullptr && buffer_size != 0)
        return ESP_ERR_INVALID_ARG;

    TickType_t timeout = timeout_ms == -1
                             ? portMAX_DELAY
                             : pdMS_TO_TICKS(timeout_ms);
    Message message{};
    if (xQueuePeek(receive_queue, &message, timeout) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    received_size = message.length;
    if (buffer_size < message.length)
        return ESP_ERR_INVALID_SIZE;

    if (xQueueReceive(receive_queue, &message, 0) != pdTRUE)
        return ESP_ERR_INVALID_STATE;
    std::memcpy(buffer, message.data, message.length);
    return ESP_OK;
}

void WiFiLink::receiveCallback(const esp_now_recv_info_t *info,
                               const uint8_t *data, int data_length)
{
    WiFiLink *link = active_link;
    if (link == nullptr || info == nullptr || info->src_addr == nullptr ||
        data == nullptr || data_length <= 0 ||
        static_cast<size_t>(data_length) > MAX_MESSAGE_SIZE ||
        std::memcmp(info->src_addr, link->peer_mac.data(), link->peer_mac.size()) != 0)
        return;

    Message message{};
    message.length = static_cast<size_t>(data_length);
    std::memcpy(message.data, data, message.length);
    if (xQueueSend(link->receive_queue, &message, 0) != pdTRUE)
        link->dropped_messages.fetch_add(1);
}

void WiFiLink::sendCallback(const esp_now_send_info_t *, esp_now_send_status_t status)
{
    WiFiLink *link = active_link;
    if (link == nullptr || !link->awaiting_send_result.exchange(false))
        return;

    link->send_status.store(status);
    xSemaphoreGive(link->send_complete);
}
