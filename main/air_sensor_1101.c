#include "air_sensor_1101.h"
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
#define _READ_DATA_FAIL_CNT_THRESHOLD_ (20)
////////////////////////////////////// Air sensor ////////////////////////////////

static const char *TAG_AIR_TASK = "+TASK_AIR_1101_UART+";

static DATA_FRAME_SSM_A1101 air_sensor_data={0};
extern ENV_DATA g_env_data;
int failed_to_read_data_cnt = 0;
volatile bool g_uart_read_fail_flag = false;

void parse_env_data()
{
    AIR_1101_DATA *pData = &air_sensor_data.packet_data;
    g_env_data.CO2  = (pData->CO2[0]<<8) | pData->CO2[1];
    g_env_data.CH2O = (pData->CH2O[0]<<8) | pData->CH2O[1];
    g_env_data.TVOC = (pData->TVOC[0]<<8) | pData->TVOC[1];
    g_env_data.PM25 =  (pData->PM25[0]<<8) | pData->PM25[1];
    g_env_data.PM10 =  (pData->PM10[0]<<8) | pData->PM10[1];
    g_env_data.PM01 =  (pData->PM01[0]<<8) | pData->PM01[1];
    g_env_data.dB =  (pData->dB[0]<<8) | pData->dB[1];
    g_env_data.LUX =  (pData->LUX[0]<<8) | pData->LUX[1];
    g_env_data.PA =  (pData->PA[0]<<24) | (pData->PA[1]<<16) | (pData->PA[2]<<8) | (pData->PA[3]);
    g_env_data.Humi = (pData->Humi[0]<<8) | pData->Humi[1];
    if((pData->Temp[0] & 0x80) != 0x0){//negitive value
        g_env_data.Temp =  ((~(pData->Temp[0])<<8) | (~(pData->Temp[1]))) + 1;
    }else{// positive value
        g_env_data.Temp = ((pData->Temp[0])<<8) | (pData->Temp[1]);
    }
}
static void init_sensor_uart()
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



#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(AIR_SENSOR_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(AIR_SENSOR_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(AIR_SENSOR_UART_PORT_NUM, AIR_SENSOR_TXD, AIR_SENSOR_RXD, AIR_SENSOR_RTS, AIR_SENSOR_CTS));
}

void air_sensor_1101_task(void *arg)
{    
    // uint32_t check_sum = 0;
    failed_to_read_data_cnt = 0;
    g_uart_read_fail_flag = false;
    init_sensor_uart();
    // Configure a temporary buffer for the incoming data
    int len = -1;
    while (1) {
        ESP_LOGD(TAG_AIR_TASK, "!!! BEGIN to Read from air sensor 1101 !!!");
        // Read data from the UART
        // data header = 0x01 0x03 0x18
        
        uint8_t data_header[3];
        while(1){
            memset(data_header, 0, sizeof(data_header));// reset data_header array. 
            len = uart_read_bytes(AIR_SENSOR_UART_PORT_NUM, data_header, 3, 1000 / portTICK_RATE_MS);
            if(len == 3 && data_header[0]==0x01 && data_header[1]==0x03 && data_header[2]==0x18){
                break;
       
            }else{
                // no matter what reason cause, 
                #if 1
                if (len == 0 ){// sensor is not connected or just bad luck, just do another read.
                    // do nothing.
                    continue;
                }else{//sensor is connected.
                    failed_to_read_data_cnt++;
                }
                if (failed_to_read_data_cnt > _READ_DATA_FAIL_CNT_THRESHOLD_)
                {
                    g_uart_read_fail_flag = true;
                    while(1){
                        vTaskDelay(1000/portTICK_RATE_MS);
                        ESP_LOGW(TAG_AIR_TASK, "!!! can not read sensor data1, wait for reset and retry.");
                    }
                }
                #endif 
                ESP_LOGW(TAG_AIR_TASK, "!!!CNT:%d Read Header len=%d, header[0]=0x%x, header[1]=0x%x header[2]=0x%x !!!", failed_to_read_data_cnt, len, data_header[0],data_header[1],data_header[2]);
            }
        }
        /*reset flag*/
        failed_to_read_data_cnt = 0;
        g_uart_read_fail_flag = false;
        ESP_LOGW(TAG_AIR_TASK, "!!! CNT:%d OKOKOK Read Header len=%d, header[0]=0x%x, header[1]=0x%x header[2]=0x%x !!!", failed_to_read_data_cnt, len, data_header[0],data_header[1],data_header[2]);
        

        // ESP_LOGD(TAG_AIR_TASK, "!!! Header Successfully Received, begin to read rest of the data !!!");
        len = uart_read_bytes(AIR_SENSOR_UART_PORT_NUM, &air_sensor_data.data[3], 26, 1000 / portTICK_RATE_MS);
        
        if (len != 26) {
            ESP_LOGE(TAG_AIR_TASK, "!!! uart_read_bytes recv rest data error !!! len=%d", len);
            #if 1//if gets header, there is no need to do this job here.
                failed_to_read_data_cnt += 5;
            #endif
            continue;
        }

        ESP_LOGD(TAG_AIR_TASK, "!!! read  %d bytes data with CRC_L=0x%x CRC_H=0x%x !!!", len, air_sensor_data.packet_data.CRC_L, air_sensor_data.packet_data.CRC_H);
        #if 0
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
        #endif
        // parse data and print.
        parse_env_data();
        memset(&air_sensor_data, 0, sizeof(air_sensor_data));
        ESP_LOGI("", "=====================================================");
        ESP_LOGI("", "\tCO2:\t%d\n\t\tCH2O:\t%d\n\t\tTVOC:\t%d\n\t\tPM25:\t%d\n\t\tPM10:\t%d",g_env_data.CO2,g_env_data.CH2O,g_env_data.TVOC,g_env_data.PM25,g_env_data.PM10);
        ESP_LOGI("", "\tTEMP:%.2f C", g_env_data.Temp/100.0);
        ESP_LOGI("", "\tHUMIDITY:%.2f %%", g_env_data.Humi/100.0);
        ESP_LOGI("", "\tNoise:%d dB ", g_env_data.dB);
        ESP_LOGI("", "\tAirPress:%d Pa", g_env_data.PA);
        ESP_LOGI("", "\tLight:%d Lux", g_env_data.LUX);

    }
}
