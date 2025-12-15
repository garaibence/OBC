#pragma once
#include <cstdint>

static inline int8_t be8s(uint8_t b) noexcept
{
    return static_cast<int8_t>(b);
}

static inline uint8_t be8u(uint8_t b) noexcept
{
    return b;
}

static inline int16_t be16s(const uint8_t *b) noexcept
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(b[0]) << 8) |
        static_cast<uint16_t>(b[1]));
}

static inline uint16_t be16u(const uint8_t *b) noexcept
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(b[0]) << 8) |
        static_cast<uint16_t>(b[1]));
}

static inline int16_t le16s(const uint8_t *b) noexcept
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(b[1]) << 8) |
        static_cast<uint16_t>(b[0]));
}

static inline uint16_t le16u(const uint8_t *b) noexcept
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(b[1]) << 8) |
        static_cast<uint16_t>(b[0]));
}
