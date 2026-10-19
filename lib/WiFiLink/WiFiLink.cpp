#include "WiFiLink.hpp"

#include <cstring>

#include <esp_log.h>
#include <esp_wifi.h>

static const char *TAG = "WIFI_LINK";

WiFiLink *WiFiLink::active_link = nullptr;

/**
 * @brief Constructs a link to a peer identified by its ESP-NOW MAC address.
 *
 * Copies the 6-byte peer MAC into the member array; a null pointer leaves the
 * MAC all-zero (invalid, so start() will reject it until one is provided).
 *
 * @param peer_mac Peer's ESP-NOW MAC address (ESP_NOW_ETH_ALEN bytes), may be null.
 * @param channel  ESP-NOW channel ID (1-13), validated later in start().
 */
WiFiLink::WiFiLink(const uint8_t *peer_mac, uint8_t channel)
    : peer_mac{}, channel(channel), receive_queue(nullptr), send_mutex(nullptr),
      send_complete(nullptr), awaiting_send_result(false), send_status(ESP_NOW_SEND_FAIL),
      dropped_messages(0), wifi_initialized(false), wifi_started(false), esp_now_initialized(false),
      peer_added(false), started(false)
{
    if (peer_mac != nullptr)
        std::memcpy(this->peer_mac.data(), peer_mac, this->peer_mac.size());
}

/**
 * @brief Destructor; tears down the link so Wi-Fi/ESP-NOW resources are released
 * even when the object goes out of scope without an explicit stop().
 */
WiFiLink::~WiFiLink()
{
    stop();
}

/**
 * @brief Checks whether the stored peer MAC is a valid individual ESP-NOW address.
 *
 * Rejects all-zero and all-0xFF (broadcast) addresses, and any address whose
 * first byte has the least-significant bit set (group, not individual address).
 *
 * @return true if the MAC can be registered as an ESP-NOW peer.
 */
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

/**
 * @brief Brings the link up: creates FreeRTOS queue/semaphores, initializes and
 * starts Wi-Fi in STA mode on the configured channel, then initializes ESP-NOW,
 * registers send/receive callbacks and adds the peer.
 *
 * Idempotent: a second call on an already-started link returns ESP_OK. Only one
 * link may be active at a time (tracked by active_link). On any failure the
 * partially-built state is torn down via cleanup().
 *
 * @return ESP_OK on success; ESP_ERR_INVALID_ARG if the MAC or channel is
 * invalid; ESP_ERR_INVALID_STATE if another link is already active;
 * ESP_ERR_NO_MEM if the FreeRTOS objects could not be allocated; otherwise the
 * error code from the failing esp_wifi_/esp_now_ call.
 */
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

/**
 * @brief Stops the link and releases all its resources (see cleanup()).
 *
 * Safe to call on a link that was never started or already stopped. NOT safe
 * while send()/receive() is executing on another task: teardown deletes the
 * queue and semaphores those calls may still be using. Intended usage model:
 * one active link between two devices, one sender/receiver thread per side;
 * lifecycle calls are not concurrent with data-path calls.
 *
 * @return First error encountered during teardown, ESP_OK if clean.
 */
esp_err_t WiFiLink::stop()
{
    return cleanup();
}

/**
 * @brief Tears down the link in reverse construction order: unregister ESP-NOW
 * callbacks, remove the peer, deinit ESP-NOW, stop/deinit Wi-Fi, then free the
 * FreeRTOS queue and semaphores.
 *
 * Each step runs only if its flag says it was completed, so cleanup() is safe at
 * any point of start(); every flag it handles is reset. The first failing step is
 * reported but teardown continues regardless.
 *
 * @return First error encountered, ESP_OK if everything tore down cleanly.
 */
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

/**
 * @brief Sends a payload to the peer and waits for ESP-NOW's send callback to
 * report the result.
 *
 * A mutex serializes concurrent send() calls; at most one send may be awaiting
 * its result at a time (awaiting_send_result). The call blocks until the
 * transport-level send callback fires, which only indicates the packet was
 * accepted by the Wi-Fi stack, not that the peer received it.
 *
 * Recovery after a callback timeout: awaiting_send_result is deliberately left
 * set so a late callback cannot be misattributed to a later send. Until the
 * callback arrives (or the link is stopped and restarted), further send()
 * calls return ESP_ERR_INVALID_STATE; the receive path is unaffected.
 *
 * @param data       Payload bytes; must not be null (zero length is rejected).
 * @param length     1..MAX_MESSAGE_SIZE bytes.
 * @param timeout_ms Per-phase timeout in ms: applied separately to acquiring
 *                   send_mutex and to waiting for the send callback, so total
 *                   blocking can approach 2*timeout_ms; must be positive.
 * @return ESP_OK if the send callback reported ESP_NOW_SEND_SUCCESS;
 * ESP_ERR_INVALID_STATE if the link is not started or a send is already pending;
 * ESP_ERR_INVALID_ARG on bad arguments; ESP_ERR_TIMEOUT on mutex or callback
 * timeout; otherwise the esp_now_send error.
 */
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

/**
 * @brief Copies the oldest pending inbound message into the caller's buffer.
 *
 * Messages arrive via receiveCallback() into a 4-deep queue. The wait is done
 * with xQueuePeek first so the message is only consumed once it fits, leaving it
 * queued on buffer-too-small rather than silently discarding it. On
 * ESP_ERR_INVALID_SIZE, received_size reports the length that would be needed;
 * on other failures it stays 0. One receiver per link is assumed: two
 * concurrent consumers can race between peek and receive, so one may get
 * ESP_ERR_INVALID_STATE for a message consumed by the other.
 *
 * @param buffer       Destination; must not be null when buffer_size is nonzero.
 * @param buffer_size Capacity of buffer in bytes.
 * @param received_size Set to the copied byte count on success; to the required
 *                     message length on ESP_ERR_INVALID_SIZE; else 0.
 * @param timeout_ms   Wait time in ms; -1 waits indefinitely (portMAX_DELAY).
 * @return ESP_OK after a successful copy; ESP_ERR_INVALID_STATE if not started
 * or the queued message vanished between peek and receive; ESP_ERR_INVALID_ARG
 * on bad arguments; ESP_ERR_TIMEOUT on wait timeout; ESP_ERR_INVALID_SIZE when
 * buffer_size is smaller than the message.
 */
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

/**
 * @brief ESP-NOW receive callback (registered by start()).
 *
 * Ignores messages that do not come from this link's peer or that are
 * out of range; otherwise copies the payload into a Message and pushes it onto
 * the receive queue non-blockingly. A full queue drops the message and bumps
 * dropped_messages, which getDroppedMessageCount() exposes. Runs on whatever
 * task ESP-NOW delivers on, so it only touches queue/atomic state.
 */
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

/**
 * @brief ESP-NOW send callback (registered by start()).
 *
 * Reports the transport-level result of the last esp_now_send(). Only the first
 * callback after a send is honored: awaiting_send_result is cleared here, so
 * spurious or duplicate callbacks are ignored. The status is stored and the
 * send_complete semaphore unblocks the waiting send() caller.
 */
void WiFiLink::sendCallback(const esp_now_send_info_t *, esp_now_send_status_t status)
{
    WiFiLink *link = active_link;
    if (link == nullptr || !link->awaiting_send_result.exchange(false))
        return;

    link->send_status.store(status);
    xSemaphoreGive(link->send_complete);
}
