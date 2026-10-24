#include "SX1278.hpp"
#include <esp_attr.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char *TAG = "SX1278";

// Frequency register fraction: FRF = frequency * 2^19 / FOSC (SX1278 FOSC = 32 MHz)
static constexpr uint32_t FXOSC_HZ = 32000000;

// REG_PA_DAC values selecting the normal and boosted power ranges
static constexpr uint8_t PA_DAC_NORMAL = 0x84;
static constexpr uint8_t PA_DAC_HIGH = 0x87;

// Operating characteristics for the two PA ranges (Semtech RF reference tables)
static constexpr uint8_t OCP_100MA = 0x2b;
static constexpr uint8_t OCP_140MA = 0x31;

// REG_MODEM_CONFIG_1 bit 0 selects implicit header mode, kept cleared
static constexpr uint8_t MC1_EXPLICIT_HEADER = 0xfe;
// REG_MODEM_CONFIG_2 bit 2 enables the payload CRC
static constexpr uint8_t MC2_CRC_ENABLE = 0x04;

// Signal bandwidths in Hz, indexed by the Bandwidth enum values
static constexpr uint32_t BW_HZ[] = {7800, 10400, 15600, 20800, 31250,
                                     41700, 62500, 125000, 250000, 500000};

// LDO must be enabled when the symbol duration exceeds this (ms)
static constexpr uint32_t LDO_SYMBOL_MS = 16;

// REG_DETECTION_OPTIMIZE/THRESHOLD values: SF=2 needs the most sensitive settings
static constexpr uint8_t DET_OPT_SF_MIN = 0xc5;
static constexpr uint8_t DET_THRESH_SF_MIN = 0x0c;
static constexpr uint8_t DET_OPT_DEFAULT = 0xc3;
static constexpr uint8_t DET_THRESH_DEFAULT = 0x0a;

// Module states and waits
static constexpr uint32_t HW_RESET_DELAY_MS = 10;
static constexpr uint32_t TX_WAIT_TIMEOUT_MS = 2000;

// Puts the module into STANDBY, the idle state between packets
static esp_err_t set_idle(SPIInterface &spi)
{
    return spi.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

// Enable/disable the LDO depending on the current bandwidth and spreading factor.
// Both are read back from the modem registers so no state has to be cached.
static esp_err_t set_ldo_flag(SPIInterface &spi)
{
    uint8_t mc1 = 0;
    uint8_t mc2 = 0;
    esp_err_t err = spi.readRegister(REG_MODEM_CONFIG_1, mc1);
    if (err != ESP_OK)
        return err;
    err = spi.readRegister(REG_MODEM_CONFIG_2, mc2);
    if (err != ESP_OK)
        return err;

    uint8_t bw_index = (mc1 & 0xf0) >> 4;
    uint8_t sf_shift = (mc2 & 0xf0) >> 4;
    if (bw_index >= sizeof(BW_HZ) / sizeof(BW_HZ[0]))
        bw_index = 0;

    uint32_t symbol_duration = 1000 / (BW_HZ[bw_index] >> sf_shift);
    uint8_t ldo_on = symbol_duration > LDO_SYMBOL_MS ? MC3_LDO_ENABLE : 0;

    uint8_t mc3 = 0;
    err = spi.readRegister(REG_MODEM_CONFIG_3, mc3);
    if (err != ESP_OK)
        return err;
    mc3 = (mc3 & ~MC3_LDO_ENABLE) | ldo_on;
    return spi.writeRegister(REG_MODEM_CONFIG_3, mc3);
}

esp_err_t SX1278::init()
{
    if (installed)
        return ESP_OK;

    esp_err_t err = spi.init();
    if (err != ESP_OK)
        return err;

    if (events == nullptr)
    {
        events = xQueueCreate(4, sizeof(uint8_t));
        if (events == nullptr)
        {
            ESP_LOGE(TAG, "xQueueCreate failed");
            return ESP_ERR_NO_MEM;
        }
    }

    // RST drives the module reset: an output held high when idle; DIO0 is a plain input
    gpio_config_t rst_cfg{};
    rst_cfg.pin_bit_mask = 1ULL << static_cast<int>(reset_gpio);
    rst_cfg.mode = GPIO_MODE_OUTPUT;
    rst_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    rst_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_cfg.intr_type = GPIO_INTR_DISABLE;
    err = gpio_config(&rst_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed for RST pin: %s", esp_err_to_name(err));
        return err;
    }
    // The module is not being reset here: keep RST released
    gpio_set_level(reset_gpio, 1);

    gpio_config_t dio0_cfg{};
    dio0_cfg.pin_bit_mask = 1ULL << static_cast<int>(dio0_gpio);
    dio0_cfg.mode = GPIO_MODE_INPUT;
    dio0_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    dio0_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dio0_cfg.intr_type = GPIO_INTR_DISABLE;
    err = gpio_config(&dio0_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed for DIO0 pin: %s", esp_err_to_name(err));
        return err;
    }

    // DIO0 signals module events through the shared GPIO ISR service. A service
    // already installed by another component is accepted (ESP_ERR_INVALID_STATE).
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }

    // DIO0 goes high when the mapped event is detected
    err = gpio_set_intr_type(dio0_gpio, GPIO_INTR_POSEDGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_set_intr_type failed: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(dio0_gpio, dio0_isr, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }

    // The default DIO0 mapping signals RX_DONE
    expected_irq = IRQ_RX_DONE;
    installed = true;
    ESP_LOGI(TAG, "DIO0 interrupt handler installed on GPIO %d", static_cast<int>(dio0_gpio));
    return ESP_OK;
}

esp_err_t SX1278::deinit()
{
    if (!installed)
        return ESP_OK;

    esp_err_t err = gpio_isr_handler_remove(dio0_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_isr_handler_remove failed: %s", esp_err_to_name(err));
        return err;
    }

    err = spi.deinit();
    if (err != ESP_OK)
        return err;

    // The event queue is kept: it holds at most 4 bytes and is reused by the next init()
    installed = false;
    return ESP_OK;
}

esp_err_t SX1278::begin(uint32_t frequency_hz)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    // Hardware reset: hold RST low at least 1 ms, release and wait >=10 ms
    gpio_set_level(reset_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(HW_RESET_DELAY_MS));

    uint8_t version = 0;
    esp_err_t err = spi.readRegister(REG_VERSION, version);
    if (err != ESP_OK)
        return err;
    if (version != VERSION_SX1278)
    {
        ESP_LOGE(TAG, "Module version 0x%02x is not a SX1278 (expected 0x%02x)",
                 static_cast<int>(version), static_cast<int>(VERSION_SX1278));
        return ESP_ERR_INVALID_VERSION;
    }

    // LongRangeMode can only be entered from the SLEEP state
    err = spi.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    if (err != ESP_OK)
        return err;

    err = setFrequency(frequency_hz);
    if (err != ESP_OK)
        return err;

    // Both FIFO streams start at address 0
    err = spi.writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
    if (err != ESP_OK)
        return err;

    // Maximum LNA extension
    uint8_t lna = 0;
    err = spi.readRegister(REG_LNA, lna);
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_LNA, lna | 0x03);
    if (err != ESP_OK)
        return err;

    // Automatic rate and power control
    err = spi.writeRegister(REG_MODEM_CONFIG_3, MC3_AGC_AUTO);
    if (err != ESP_OK)
        return err;

    err = setTxPower(17);
    if (err != ESP_OK)
        return err;

    return set_idle(spi);
}

esp_err_t SX1278::beginPacket()
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    // A new payload always starts from the idle state
    esp_err_t err = set_idle(spi);
    if (err != ESP_OK)
        return err;

    packet_index = 0;

    uint8_t mc1 = 0;
    err = spi.readRegister(REG_MODEM_CONFIG_1, mc1);
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_MODEM_CONFIG_1, mc1 & MC1_EXPLICIT_HEADER);
    if (err != ESP_OK)
        return err;

    err = spi.writeRegister(REG_FIFO_ADDR_PTR, 0);
    if (err != ESP_OK)
        return err;
    return spi.writeRegister(REG_PAYLOAD_LENGTH, 0);
}

esp_err_t SX1278::write(const uint8_t *data, size_t len)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;
    if (len == 0)
        return ESP_OK;

    uint8_t payload_length = 0;
    esp_err_t err = spi.readRegister(REG_PAYLOAD_LENGTH, payload_length);
    if (err != ESP_OK)
        return err;

    size_t free = MAX_PKT_LENGTH - payload_length;
    if (len > free)
    {
        ESP_LOGW(TAG, "Truncating payload to %d bytes", static_cast<int>(free));
        len = free;
    }

    err = spi.writeRegisters(REG_FIFO, data, len);
    if (err != ESP_OK)
        return err;
    return spi.writeRegister(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(payload_length + len));
}

esp_err_t SX1278::endPacket(bool async)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    // Clear stale event flags before starting transmission (both modes)
    esp_err_t err = spi.writeRegister(REG_IRQ_FLAGS, IRQ_ALL_FLAGS);
    if (err != ESP_OK)
        return err;

    if (async)
    {
        // DIO0 signals TX_DONE; waitEvent() picks it up from the queue
        err = spi.writeRegister(REG_DIO_MAPPING_1, DIO0_MAP_TX_DONE);
        if (err != ESP_OK)
            return err;
        expected_irq = IRQ_TX_DONE;
        return spi.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    }

    err = spi.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    if (err != ESP_OK)
        return err;

    // Poll IRQ_FLAGS until TX_DONE is set and clear the flags when it is.
    // The deadline is kept in ticks and compared wrap-safe.
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(TX_WAIT_TIMEOUT_MS);
    while ((int32_t)(deadline - xTaskGetTickCount()) > 0)
    {
        uint8_t irq = 0;
        err = spi.readRegister(REG_IRQ_FLAGS, irq);
        if (err != ESP_OK)
            return err;
        if (irq & IRQ_TX_DONE)
            return spi.writeRegister(REG_IRQ_FLAGS, irq);
        // vTaskDelay(1) is the shortest legal wait
        vTaskDelay(1);
    }

    ESP_LOGE(TAG, "Timeout waiting for TX_DONE");
    uint8_t irq = 0;
    esp_err_t err2 = spi.readRegister(REG_IRQ_FLAGS, irq);
    if (err2 == ESP_OK)
        spi.writeRegister(REG_IRQ_FLAGS, irq);
    return ESP_ERR_TIMEOUT;
}

esp_err_t SX1278::setFrequency(uint32_t frequency_hz)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    // FRF = frequency * 2^19 / FOSC, same integer arithmetic as Arduino-LoRa
    uint64_t frf = (static_cast<uint64_t>(frequency_hz) << 19) / FXOSC_HZ;

    esp_err_t err = spi.writeRegister(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16));
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_FRF_MID, static_cast<uint8_t>(frf >> 8));
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_FRF_LSB, static_cast<uint8_t>(frf));
    if (err != ESP_OK)
        return err;

    frequency = frequency_hz;
    return ESP_OK;
}

esp_err_t SX1278::setTxPower(int level)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    if (level > 20)
        level = 20;
    if (level < 2)
    {
        ESP_LOGW(TAG, "Clamping TX power to %d dBm", 2);
        level = 2;
    }

    // Levels above 17 dBm use the extra PA_DAC boost and a higher internal power
    uint8_t pa_dac = PA_DAC_NORMAL;
    uint8_t ocp = OCP_100MA;
    if (level > 17)
    {
        level -= 3;
        pa_dac = PA_DAC_HIGH;
        ocp = OCP_140MA;
    }

    esp_err_t err = spi.writeRegister(REG_PA_DAC, pa_dac);
    if (err != ESP_OK)
        return err;
    err = spi.writeRegister(REG_OCP, ocp);
    if (err != ESP_OK)
        return err;
    return spi.writeRegister(REG_PA_CONFIG, PA_BOOST_ENABLE | static_cast<uint8_t>(level - 2));
}

esp_err_t SX1278::setSpreadingFactor(SpreadingFactor sf)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t sf_value = static_cast<uint8_t>(sf);

    // SF=2 needs the most sensitive signal detection settings
    esp_err_t err;
    if (sf_value == 6)
    {
        err = spi.writeRegister(REG_DETECTION_OPTIMIZE, DET_OPT_SF_MIN);
        if (err != ESP_OK)
            return err;
        err = spi.writeRegister(REG_DETECTION_THRESHOLD, DET_THRESH_SF_MIN);
    }
    else
    {
        err = spi.writeRegister(REG_DETECTION_OPTIMIZE, DET_OPT_DEFAULT);
        if (err != ESP_OK)
            return err;
        err = spi.writeRegister(REG_DETECTION_THRESHOLD, DET_THRESH_DEFAULT);
    }
    if (err != ESP_OK)
        return err;

    uint8_t mc2 = 0;
    esp_err_t err_read = spi.readRegister(REG_MODEM_CONFIG_2, mc2);
    if (err_read != ESP_OK)
        return err_read;
    mc2 = (mc2 & 0x0f) | static_cast<uint8_t>(sf_value << 4);
    err = spi.writeRegister(REG_MODEM_CONFIG_2, mc2);
    if (err != ESP_OK)
        return err;

    return set_ldo_flag(spi);
}

esp_err_t SX1278::setBandwidth(Bandwidth bw)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t mc1 = 0;
    esp_err_t err = spi.readRegister(REG_MODEM_CONFIG_1, mc1);
    if (err != ESP_OK)
        return err;
    mc1 = (mc1 & 0x0f) | static_cast<uint8_t>(static_cast<uint8_t>(bw) << 4);
    err = spi.writeRegister(REG_MODEM_CONFIG_1, mc1);
    if (err != ESP_OK)
        return err;

    return set_ldo_flag(spi);
}

esp_err_t SX1278::setCodingRate(CodingRate cr)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t mc1 = 0;
    esp_err_t err = spi.readRegister(REG_MODEM_CONFIG_1, mc1);
    if (err != ESP_OK)
        return err;
    mc1 = (mc1 & 0xf1) | static_cast<uint8_t>(static_cast<uint8_t>(cr) << 1);
    return spi.writeRegister(REG_MODEM_CONFIG_1, mc1);
}

esp_err_t SX1278::setSyncWord(uint8_t sync_word)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;
    return spi.writeRegister(REG_SYNC_WORD, sync_word);
}

esp_err_t SX1278::enableCrc()
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t mc2 = 0;
    esp_err_t err = spi.readRegister(REG_MODEM_CONFIG_2, mc2);
    if (err != ESP_OK)
        return err;
    mc2 |= MC2_CRC_ENABLE;
    return spi.writeRegister(REG_MODEM_CONFIG_2, mc2);
}

esp_err_t SX1278::disableCrc()
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t mc2 = 0;
    esp_err_t err = spi.readRegister(REG_MODEM_CONFIG_2, mc2);
    if (err != ESP_OK)
        return err;
    mc2 &= ~MC2_CRC_ENABLE;
    return spi.writeRegister(REG_MODEM_CONFIG_2, mc2);
}

esp_err_t SX1278::parsePacket()
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t irq = 0;
    esp_err_t err = spi.readRegister(REG_IRQ_FLAGS, irq);
    if (err != ESP_OK)
        return err;

    // Writing the same value back clears every flag that was set
    err = spi.writeRegister(REG_IRQ_FLAGS, irq);
    if (err != ESP_OK)
        return err;

    if ((irq & IRQ_RX_DONE) && !(irq & IRQ_PAYLOAD_CRC_ERROR))
    {
        uint8_t len = 0;
        err = spi.readRegister(REG_RX_NB_BYTES, len);
        if (err != ESP_OK)
            return err;
        rx_nb_bytes = len;
        packet_index = 0;

        // Point the FIFO reader at the start of the received packet
        uint8_t addr = 0;
        err = spi.readRegister(REG_FIFO_RX_CURRENT_ADDR, addr);
        if (err != ESP_OK)
            return err;
        err = spi.writeRegister(REG_FIFO_ADDR_PTR, addr);
        if (err != ESP_OK)
            return err;

        set_idle(spi);
        return ESP_OK;
    }

    // No valid packet: make sure the module keeps looking for one
    uint8_t op_mode = 0;
    err = spi.readRegister(REG_OP_MODE, op_mode);
    if (err != ESP_OK)
        return err;
    if (op_mode != (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS))
    {
        // An async TX may have remapped DIO0 to TX_DONE: RX_DONE must signal again
        err = spi.writeRegister(REG_DIO_MAPPING_1, DIO0_MAP_RX_DONE);
        if (err != ESP_OK)
            return err;
        expected_irq = IRQ_RX_DONE;
        err = spi.writeRegister(REG_FIFO_ADDR_PTR, 0);
        if (err != ESP_OK)
            return err;
        err = spi.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
        if (err != ESP_OK)
            return err;
    }

    return irq & IRQ_RX_DONE ? ESP_ERR_INVALID_CRC : ESP_ERR_TIMEOUT;
}

esp_err_t SX1278::available(size_t &len)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    len = rx_nb_bytes > packet_index ? static_cast<size_t>(rx_nb_bytes - packet_index) : 0;
    return ESP_OK;
}

esp_err_t SX1278::read(uint8_t &data)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;
    if (packet_index >= rx_nb_bytes)
        return ESP_ERR_NOT_FOUND;

    esp_err_t err = spi.readRegister(REG_FIFO, data);
    if (err != ESP_OK)
        return err;
    packet_index++;
    return ESP_OK;
}

esp_err_t SX1278::packetRssi(int &rssi)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t raw = 0;
    esp_err_t err = spi.readRegister(REG_PKT_RSSI_VALUE, raw);
    if (err != ESP_OK)
        return err;

    int offset = frequency < RF_MID_BAND_THRESHOLD_HZ ? RSSI_OFFSET_LOW_FREQ : RSSI_OFFSET_HIGH_FREQ;
    rssi = static_cast<int>(raw) - offset;
    return ESP_OK;
}

esp_err_t SX1278::packetSnr(float &snr)
{
    if (!installed)
        return ESP_ERR_INVALID_STATE;

    uint8_t raw = 0;
    esp_err_t err = spi.readRegister(REG_PKT_SNR_VALUE, raw);
    if (err != ESP_OK)
        return err;

    snr = static_cast<int8_t>(raw) * 0.25f;
    return ESP_OK;
}

esp_err_t SX1278::waitEvent(uint32_t timeout_ms, uint8_t &event)
{
    if (!installed || events == nullptr)
        return ESP_ERR_INVALID_STATE;

    // Blocking receive: FreeRTOS does the waiting itself, in ticks
    if (xQueueReceive(events, &event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
    {
        // In async mode nobody else clears the flags after TX_DONE: do it here
        if (event & IRQ_TX_DONE)
        {
            esp_err_t err = spi.writeRegister(REG_IRQ_FLAGS, IRQ_ALL_FLAGS);
            if (err != ESP_OK)
                return err;
        }
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

// Runs inside the GPIO ISR: IRAM_ATTR and queue.h FromISR calls only
IRAM_ATTR void SX1278::dio0_isr(void *arg)
{
    // No real work in the ISR: just hand the event over to waitEvent()
    SX1278 *self = static_cast<SX1278 *>(arg);
    if (self == nullptr || self->events == nullptr)
        return;

    uint8_t irq = self->expected_irq;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(self->events, &irq, &woken);
    (void)woken;
}
