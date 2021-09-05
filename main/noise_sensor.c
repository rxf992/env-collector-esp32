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

#include "noise_sensor.h"


#define NOISE_SENSOR_TXD (17)
#define NOISE_SENSOR_RXD (16)
#define NOISE_SENSOR_RTS (UART_PIN_NO_CHANGE)
#define NOISE_SENSOR_CTS (UART_PIN_NO_CHANGE)
#define NOISE_SENSOR_UART_PORT_NUM      (2)
#define NOISE_SENSOR_UART_BAUD_RATE     (115200)

#define BUF_SIZE (1024)
static const char *TAG_NOISE_TASK = "+TASK_NOISE_UART+";
float noise_sensor_db = 0;

void noise_sensor_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = NOISE_SENSOR_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    uint8_t sensor_data[6] = {0};   ////store a complete data frame.
    // uint32_t check_sum = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(NOISE_SENSOR_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(NOISE_SENSOR_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(NOISE_SENSOR_UART_PORT_NUM, NOISE_SENSOR_TXD, NOISE_SENSOR_RXD, NOISE_SENSOR_RTS, NOISE_SENSOR_CTS));

    // Configure a temporary buffer for the incoming data
    int len = -1;
    while (1) {
        ESP_LOGD(TAG_NOISE_TASK, "!!! BEGIN to Read from noise sensor !!!");
        sensor_data[0] = 0;
        // Read data from the UART
        // data[0]=0xBB
        while (1){
            len = uart_read_bytes(NOISE_SENSOR_UART_PORT_NUM, sensor_data, 1, 1000 / portTICK_RATE_MS);
            if (len > 0 && sensor_data[0]==0xBB) {
                break;
            }else{
                ESP_LOGE(TAG_NOISE_TASK, "!!! uart_read first byte failed. len=%d, sensor_data[0]=0x%x", len, sensor_data[0]);
            }

        
        }
        //data[1]=0xAA
        //data[2]:CMD/0x01
        //data[3..4]:Noise Value
        //data[5] checksum/0xCC
        ESP_LOGD(TAG_NOISE_TASK, "!!! First Byte Received, begin to read rest of the data !!!");
        len = uart_read_bytes(NOISE_SENSOR_UART_PORT_NUM, &sensor_data[1], 5, 200 / portTICK_RATE_MS);
        ESP_LOGD(TAG_NOISE_TASK, "!!! uart_read_bytes return len=%d, checksum=0x%x !!!", len, sensor_data[5]);
        if (len != (sizeof(sensor_data)-1) || sensor_data[1]!=0xAA || sensor_data[5]!=0xCC) {
            ESP_LOGE(TAG_NOISE_TASK, "!!! uart_read_bytes recv rest data error !!!");
            for (int i=0; i<6; i++){
                ESP_LOGE(TAG_NOISE_TASK,"[%d]=0x%x",i, sensor_data[i]);
            }
            
            continue;
        }
        

        noise_sensor_db =  ((sensor_data[4]*256) + sensor_data[3]) / 10.0;
        ESP_LOGE(TAG_NOISE_TASK, "=====>Noise:%f dB", noise_sensor_db);


    }
}