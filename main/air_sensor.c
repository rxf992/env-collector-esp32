#include "air_sensor.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
///////////////////UART related ////////////////////////
#define BUF_SIZE (1024)

////////////////////////////////////// Air sensor ////////////////////////////////
#define AIR_SENSOR_TXD (26)
#define AIR_SENSOR_RXD (25)
#define AIR_SENSOR_RTS (UART_PIN_NO_CHANGE)
#define AIR_SENSOR_CTS (UART_PIN_NO_CHANGE)
#define AIR_SENSOR_UART_PORT_NUM      (1)
#define AIR_SENSOR_UART_BAUD_RATE     (9600)

static const char *TAG_AIR_TASK = "+TASK_AIR_UART+";

static uint8_t air_sensor_data[17]={0};////recv a complete data frame.
AIR_DATA air_data={0};
void parse_air_data(AIR_DATA* data)
{
    air_data.CO2  = (air_sensor_data[2]<<8) | air_sensor_data[3];
    air_data.CH2O =  (air_sensor_data[4]<<8) | air_sensor_data[5];
    air_data.TVOC =  (air_sensor_data[6]<<8) | air_sensor_data[7];
    air_data.PM25 =  (air_sensor_data[8]<<8) | air_sensor_data[9];
    air_data.PM10 =  (air_sensor_data[10]<<8) | air_sensor_data[11];
    
    air_data.Temp_int = air_sensor_data[12] & 0x7F;//bit7 is sign bit.
    if(air_sensor_data[12] & 0x80)
    {
        air_data.Temp_int = -air_data.Temp_int;
    }
    air_data.Temp_float = air_sensor_data[13];

    air_data.Hummidity_int = air_sensor_data[14];
    air_data.Hummidity_float = air_sensor_data[15];
}

void air_sensor_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = AIR_SENSOR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    
    uint32_t check_sum = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(AIR_SENSOR_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(AIR_SENSOR_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(AIR_SENSOR_UART_PORT_NUM, AIR_SENSOR_TXD, AIR_SENSOR_RXD, AIR_SENSOR_RTS, AIR_SENSOR_CTS));

    // Configure a temporary buffer for the incoming data
    int len = -1;
    while (1) {
        ESP_LOGD(TAG_AIR_TASK, "!!! BEGIN to Read from air sensor !!!");
        // Read data from the UART
        // data[0]=0x3C
        while (1){
            len = uart_read_bytes(AIR_SENSOR_UART_PORT_NUM, air_sensor_data, 1, 1000 / portTICK_RATE_MS);
            // ESP_LOGD(TAG, "!!! uart_read_bytes return len=%d, data=0x%x !!!", len, air_sensor_data[0]);
            if (len > 0 && air_sensor_data[0]==0x3C) {
                break;
            }else{
                ESP_LOGD(TAG_AIR_TASK, "!!! uart_read first byte failed. retring.");
            }

        
        }
        //data[1]=0x02
        //data[2:15]
        //data[16] checksum
        // ESP_LOGD(TAG_AIR_TASK, "!!! First Byte Received, begin to read rest of the data !!!");
        len = uart_read_bytes(AIR_SENSOR_UART_PORT_NUM, &air_sensor_data[1], 16, 200 / portTICK_RATE_MS);
        ESP_LOGD(TAG_AIR_TASK, "!!! uart_read_bytes return len=%d, data[16]=0x%x !!!", len, air_sensor_data[16]);
        if (len != (sizeof(air_sensor_data)-1) || air_sensor_data[1]!=0x02) {
            ESP_LOGE(TAG_AIR_TASK, "!!! uart_read_bytes recv rest data error !!!");
            continue;
        }
        
        // verify check sum
        check_sum = 0;//clear check sum first.
        for(int i=0; i < 16; i++ ){
            check_sum += air_sensor_data[i];
            // ESP_LOGD(TAG_AIR_TASK, "===>[%d]=0x%x, sum=%x", i, air_sensor_data[i], check_sum);
        }
        
        uint8_t u8_sum = (uint8_t)(check_sum & 0xFF);
        // ESP_LOGD(TAG_AIR_TASK, "!!! check_sum=%x, u8_sum=%x",check_sum, u8_sum);
        if (u8_sum == air_sensor_data[16])
        {
            ESP_LOGD(TAG_AIR_TASK, "!!! Received ==valid== data frame from air sensor !!!");
        }else{
            ESP_LOGW(TAG_AIR_TASK, "!!! checksum error !!! u8_sum=0x%x, recv = 0x%x", u8_sum, air_sensor_data[16]);
            for(int i=0, check_sum=0; i < 16; i++ ){
                check_sum += air_sensor_data[i];
                ESP_LOGD(TAG_AIR_TASK, "===>[%d]=0x%x, sum=%x", i, air_sensor_data[i], check_sum);
            }
            continue;
        }
        // parse data and print.
        parse_air_data(&air_data);
        ESP_LOGI("", "=====================================================");
        ESP_LOGI("", "\tCO2:\t%d\n\t\tCH2O:\t%d\n\t\tTVOC:\t%d\n\t\tPM25:\t%d\n\t\tPM10:\t%d",air_data.CO2,air_data.CH2O,air_data.TVOC,air_data.PM25,air_data.PM10);
        ESP_LOGI("", "\tTEMP:%d.%d C", air_data.Temp_int,air_data.Temp_float);
        ESP_LOGI("", "\tHUMIDITY:%d.%d %%", air_data.Hummidity_int,air_data.Hummidity_float);

    }
}
