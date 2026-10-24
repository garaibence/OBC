#pragma once

#include <cstdint>

// Register map of the Semtech SX1276/SX1278 family (Ra-02 compatible)
static constexpr uint8_t REG_FIFO = 0x00;
static constexpr uint8_t REG_OP_MODE = 0x01;
static constexpr uint8_t REG_FRF_MSB = 0x06;
static constexpr uint8_t REG_FRF_MID = 0x07;
static constexpr uint8_t REG_FRF_LSB = 0x08;
static constexpr uint8_t REG_PA_CONFIG = 0x09;
static constexpr uint8_t REG_OCP = 0x0b;
static constexpr uint8_t REG_LNA = 0x0c;
static constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0d;
static constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0e;
static constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0f;
static constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
static constexpr uint8_t REG_IRQ_FLAGS = 0x12;
static constexpr uint8_t REG_RX_NB_BYTES = 0x13;
static constexpr uint8_t REG_PKT_SNR_VALUE = 0x19;
static constexpr uint8_t REG_PKT_RSSI_VALUE = 0x1a;
static constexpr uint8_t REG_MODEM_CONFIG_1 = 0x1d;
static constexpr uint8_t REG_MODEM_CONFIG_2 = 0x1e;
static constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
static constexpr uint8_t REG_MODEM_CONFIG_3 = 0x26;
static constexpr uint8_t REG_DETECTION_OPTIMIZE = 0x31;
static constexpr uint8_t REG_DETECTION_THRESHOLD = 0x37;
static constexpr uint8_t REG_SYNC_WORD = 0x39;
static constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;
static constexpr uint8_t REG_VERSION = 0x42;
static constexpr uint8_t REG_PA_DAC = 0x4d;
// REG_PA_CONFIG: bit 7 selects PA_BOOST, bits 3-2 are the normal-range power
static constexpr uint8_t PA_BOOST_ENABLE = 0x80;
static constexpr uint8_t PA_RANGE_MAX = 0x07;

// REG_OP_MODE: bit 7 selects long-range mode, low bits select the device state
static constexpr uint8_t MODE_LONG_RANGE_MODE = 0x80;
static constexpr uint8_t MODE_SLEEP = 0x00;
static constexpr uint8_t MODE_STDBY = 0x01;
static constexpr uint8_t MODE_TX = 0x03;
static constexpr uint8_t MODE_RX_CONTINUOUS = 0x05;
static constexpr uint8_t MODE_RX_SINGLE = 0x06;

// REG_IRQ_FLAGS bit masks (LoRa meanings)
static constexpr uint8_t IRQ_CAD_DETECTED = 0x01;
static constexpr uint8_t IRQ_FHSS_CHANGE = 0x02;
static constexpr uint8_t IRQ_CAD_DONE = 0x04;
static constexpr uint8_t IRQ_TX_DONE = 0x08;
static constexpr uint8_t IRQ_VALID_HEADER = 0x10;
static constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR = 0x20;
static constexpr uint8_t IRQ_RX_DONE = 0x40;
static constexpr uint8_t IRQ_RX_TIMEOUT = 0x80;
// Writing this value clears all event flags at once
static constexpr uint8_t IRQ_ALL_FLAGS = 0xff;

// REG_DIO_MAPPING_1 values for DIO0 (bit 7-6)
static constexpr uint8_t DIO0_MAP_RX_DONE = 0x00;
static constexpr uint8_t DIO0_MAP_TX_DONE = 0x40;

// REG_MODEM_CONFIG_3 bit masks
static constexpr uint8_t MC3_AGC_AUTO = 0x04;
static constexpr uint8_t MC3_LDO_ENABLE = 0x08;

// Signal measurement constants (Semtech SX1276/77/78/79 manual)
static constexpr uint32_t RF_MID_BAND_THRESHOLD_HZ = 525000000;
static constexpr int RSSI_OFFSET_LOW_FREQ = 164;
static constexpr int RSSI_OFFSET_HIGH_FREQ = 157;

// REG_VERSION content of the SX1278
static constexpr uint8_t VERSION_SX1278 = 0x12;

// Limits
static constexpr size_t MAX_PKT_LENGTH = 255;
