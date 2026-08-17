#include "crc32.h"

uint32_t robot_crc32(const uint8_t *data, size_t length)
{
    const uint32_t poly = 0xEDB88320UL;
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            uint32_t mask = -(crc & 1UL);
            crc = (crc >> 1U) ^ (poly & mask);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}
