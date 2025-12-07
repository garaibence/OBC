#pragma once
#include <cstdint>

static inline int8_t be8(const uint8_t b) noexcept
{
    return static_cast<int8_t>(b);
}

static inline int8_t le8(const uint8_t b) noexcept
{
    return be8(b);
}

static inline int16_t be16(const uint8_t *b) noexcept
{
    return static_cast<int16_t>((static_cast<uint16_t>(b[0]) << 8) | static_cast<uint16_t>(b[1]));
}

static inline int16_t le16(const uint8_t *b) noexcept
{
    return static_cast<int16_t>((static_cast<uint16_t>(b[1]) << 8) | static_cast<uint16_t>(b[0]));
}