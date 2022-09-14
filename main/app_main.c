/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "my_network.h"
#include "air_sensor_1101.h"
#include "sd_card.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define DO_RESET_THRESHOLD          (60*60*24)
#define ECHO_TASK_STACK_SIZE        (2048)
uint32_t g_running_cnt = 0;
bool g_running_need_reset_flag = false;
bool g_repeat_time_reach_threshold = false;
ENV_DATA g_env_data;

static const char *TAG = "+TASK_APP_MAIN+";
extern volatile bool g_uart_read_fail_flag;

// extern float light_sensor_lux;
// extern float noise_sensor_db;
// SemaphoreHandle_t I2CMutex = NULL;

// #define GPIO_OUTPUT_IO_0    2
// #define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
// #define GPIO_OUTPUT_I2C_SCL (18)
// #define GPIO_OUTPUT_PIN_SEL_SCL (1ULL<<GPIO_OUTPUT_I2C_SCL)

// #define BUF_SIZE (1024)


int generate_one_data_line_csv(char* buffer, size_t buffer_size, const ENV_DATA* air)
{

    if (air->Humi ==0 && air->PA == 0){
        return -1;//invalid data.
    }
    #if 1// use PA value to detect if the system needs to be reset.
    static uint32_t last_PA_value = 0;
    static int PA_value_repeat_cnt = 0;
    
    if(last_PA_value == air->PA)
    {
        PA_value_repeat_cnt++;
    }else{
        PA_value_repeat_cnt = 0;
        g_repeat_time_reach_threshold = false;
    }
    if(PA_value_repeat_cnt > 30)// repeat for over threshold.
    {
        g_repeat_time_reach_threshold = true;// in sdcard thread, will detect this flag and trigger log record and esp_restart.
    }
    /* update with new PA value.*/
    last_PA_value = air->PA;
    #endif

    /*
      parse to buffer.
      parse air to buffer.
    */
    int res = sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d\n", \
                        air->CH2O, air->CO2, air->TVOC, air->PM25, \
                        air->PM10, air->LUX, air->dB,   (air->Temp)/100.0, (air->Humi)/100.0, air->PA);
    ESP_LOGI(TAG,"csv buffer=%s", buffer);
    return res;
}

// static void gpio_setup(uint64_t pin_mask)
// {
//     gpio_config_t io_conf;
//     //disable interrupt
//     io_conf.intr_type = GPIO_INTR_DISABLE;
//     //set as output mode
//     io_conf.mode = GPIO_MODE_OUTPUT;
//     //bit mask of the pins that you want to set,e.g.GPIO18/19
//     io_conf.pin_bit_mask = pin_mask;
//     //disable pull-down mode
//     io_conf.pull_down_en = 0;
//     //disable pull-up mode
//     io_conf.pull_up_en = 0;
//     //configure GPIO with the given settings
//     gpio_config(&io_conf);
// }
void app_main(void)
{
    ESP_LOGI(TAG, "Hello world!\n");

    //Initialize NVS needed by WiFi
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    // ESP_ERROR_CHECK(i2c_master_init());
    // I2CMutex = xSemaphoreCreateMutex();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    // xTaskCreate(i2c_task_bme280_bmp280_bmp180, "i2c_bmp280_task", 2048, NULL, 10, NULL); 
    // xTaskCreate(light_sensor_task, "i2c_read_BH1750_task", 1024 * 2, (void *)0, 10, NULL);  
    xTaskCreate(air_sensor_1101_task, "uart_air_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    // xTaskCreate(noise_sensor_task, "uart_noise_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(sd_card_task, "sd_card_task", 4096, NULL, 10, NULL);
    
    
    // gpio_setup();
    // int level = 0;
    while (1) {
        // gpio_set_level(GPIO_OUTPUT_IO_0, level);//GPIO2 also SD_D2 conflict.
        // level = !level;
        #if 1
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        g_running_cnt++;
        if (g_running_cnt >= DO_RESET_THRESHOLD)
        {
            g_running_need_reset_flag = true;
        }
        if (g_uart_read_fail_flag || g_running_need_reset_flag)
        {
            ESP_LOGE(TAG, "need to do reset !!!");
            // esp_restart();
        }
        #endif
    }
}
