#ifndef __AIR_SENSOR_1101_
#define __AIR_SENSOR_1101_
#include <stdio.h>

#define AIR_SENSOR_TXD (26)
#define AIR_SENSOR_RXD (25)
#define AIR_SENSOR_RTS (UART_PIN_NO_CHANGE)
#define AIR_SENSOR_CTS (UART_PIN_NO_CHANGE)
#define AIR_SENSOR_UART_PORT_NUM      (1)
#define AIR_SENSOR_UART_BAUD_RATE     (9600)


typedef struct {//big endian.H->L
        uint8_t Header[3];
        uint8_t CO2[2];
        uint8_t TVOC[2];
        uint8_t CH2O[2];
        uint8_t PM25[2];
        uint8_t Humi[2];
        uint8_t Temp[2];//signed
        uint8_t PM10[2];
        uint8_t PM01[2];
        uint8_t LUX[2];
        uint8_t dB[2];
        uint8_t PA[4];
        uint8_t CRC_L;
        uint8_t CRC_H;
}AIR_1101_DATA;
typedef union{
    AIR_1101_DATA packet_data;
    uint8_t data[3+0x18+2];
}DATA_FRAME_SSM_A1101;


typedef struct {
        uint16_t CO2;
        uint16_t TVOC;
        uint16_t CH2O;
        uint16_t PM25;
        uint16_t Humi;
        int16_t Temp;//signed
        uint16_t PM10;
        uint16_t PM01;
        uint16_t LUX;
        uint16_t dB;
        uint32_t PA;
}ENV_DATA;// share for all tasks.
void air_sensor_1101_task(void *arg);
#endif