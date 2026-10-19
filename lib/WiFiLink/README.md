# WiFiLink

`WiFiLink` is a direct, MAC-addressed ESP-NOW link for two ESP32 devices. It
does not create an access point or require a router or IP addresses. Both
devices must use the same Wi-Fi channel and construct a link with the other
device's station MAC address.

```cpp
const uint8_t peer_mac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x00};
WiFiLink link(peer_mac, 1);

if (link.start() != ESP_OK)
{
    // Handle startup failure.
}

const char message[] = "READY";
esp_err_t err = link.send(message, sizeof(message) - 1);

uint8_t response[WiFiLink::MAX_MESSAGE_SIZE];
size_t response_size = 0;
err = link.receive(response, sizeof(response), response_size, 1000);

link.stop();
```

`send()` waits for ESP-NOW's peer delivery status; it is not an application
acknowledgement. Use explicit messages and timeouts in the application for
application-level handshakes. The driver only accepts packets from its
configured peer and queues up to four received messages. Check
`getDroppedMessageCount()` to detect packets discarded because that queue was
full. MAC filtering alone is not cryptographic authentication; configure
ESP-NOW encryption and provision keys before relying on the link against
spoofing or interception.

The link is stopped explicitly with `stop()` when communication is complete;
the destructor also stops it. Only one `WiFiLink` instance may be active at a
time.
