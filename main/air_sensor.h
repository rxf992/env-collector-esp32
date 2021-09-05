#ifndef __AIR_SENSOR__
#define __AIR_SENSOR__
#include <stdio.h>
typedef struct {
        uint16_t CO2;
        uint16_t CH2O;
        uint16_t TVOC;
        uint16_t PM25;
        uint16_t PM10;
        int8_t Temp_int;
        uint8_t Temp_float;
        uint8_t Hummidity_int;
        uint8_t Hummidity_float;
}AIR_DATA;

void air_sensor_task(void *arg);
#endif