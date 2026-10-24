#pragma once

#include "SPIInterface.hpp"
#include "SX1278Regs.hpp"
#include <esp_err.h>
#include <cstdint>
#include <cstddef>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class SX1278
{
public:
    SX1278(SPIInterface &spi, gpio_num_t reset_gpio, gpio_num_t dio0_gpio)
        : spi(spi), reset_gpio(reset_gpio), dio0_gpio(dio0_gpio),
          frequency(0), rx_nb_bytes(0), packet_index(0),
          expected_irq(0), events(nullptr), installed(false) {}

    ~SX1278() { deinit(); }

    // Installs the SPI device and the DIO0 GPIO interrupt; does not touch the module
    esp_err_t init();
    esp_err_t deinit();

    // Hardware reset, version check, frequency and defaults; leaves the module in STDBY
    esp_err_t begin(uint32_t frequency_hz);

    enum class SpreadingFactor
    {
        SF_7 = 7,
        SF_8 = 8,
        SF_9 = 9,
        SF_10 = 10,
        SF_11 = 11,
        SF_12 = 12,
    };

    enum class Bandwidth
    {
        BW_7_8K = 0x0,
        BW_10_4K = 0x1,
        BW_15_6K = 0x2,
        BW_20_8K = 0x3,
        BW_31_25K = 0x4,
        BW_41_7K = 0x5,
        BW_62_5K = 0x6,
        BW_125K = 0x7,
        BW_250K = 0x8,
        BW_500K = 0x9,
    };

    enum class CodingRate
    {
        CR_4_5 = 0x1,
        CR_4_6 = 0x2,
        CR_4_7 = 0x3,
        CR_4_8 = 0x4,
    };

    esp_err_t beginPacket();
    esp_err_t write(const uint8_t *data, size_t len);
    // async: TX is started and completion is reported through DIO0 (use waitEvent)
    esp_err_t endPacket(bool async = false);

    esp_err_t setFrequency(uint32_t frequency_hz);
    // PA_BOOST only; 2..20 dBm, values above 17 use the extra PA_DAC boost
    esp_err_t setTxPower(int level);
    esp_err_t setSpreadingFactor(SpreadingFactor sf);
    esp_err_t setBandwidth(Bandwidth bw);
    esp_err_t setCodingRate(CodingRate cr);
    esp_err_t setSyncWord(uint8_t sync_word);
    esp_err_t enableCrc();
    esp_err_t disableCrc();

    // Parses the last RX_DONE event; payload is then read with available()/read()
    esp_err_t parsePacket();
    esp_err_t available(size_t &len);
    esp_err_t read(uint8_t &data);

    // Signal-to-noise and RSSI of the packet received last
    esp_err_t packetRssi(int &rssi);
    esp_err_t packetSnr(float &snr);

    // Waits for a DIO0 event (TX_DONE / RX_DONE); ESP_ERR_TIMEOUT on timeout
    esp_err_t waitEvent(uint32_t timeout_ms, uint8_t &event);

private:
    SPIInterface &spi;
    gpio_num_t reset_gpio;
    gpio_num_t dio0_gpio;

    uint32_t frequency;
    uint8_t rx_nb_bytes;
    uint16_t packet_index;
    uint8_t expected_irq;
    QueueHandle_t events;
    bool installed;

    static void dio0_isr(void *arg);
};
