#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "stm32f4xx_hal.h"

typedef union {
    struct {
        uint16_t temp_data;          // Temperature data (2 bytes)
        uint8_t temp_crc;            // Temperature CRC (1 byte)
        uint8_t temp_crc_expected;   // Temperature CRC expected (1 byte)
        uint16_t humi_data;          // Humidity data (2 bytes)
        uint8_t humi_crc;            // Humidity CRC (1 byte)
        uint8_t humi_crc_expected;   // Humidity CRC expected (1 byte)
    } fields;
    uint32_t values[2];                // 2 words (32 bits each)
} SensorData;

#endif /* SENSOR_DATA_H */
