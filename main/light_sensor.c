/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "light_sensor.h"
#include "my_iic.h"
#include "BME280_BMP280_BMP180.h"

static const char *TAG = "+TASK-LIGHT-SENSOR-BH1750+";
float light_sensor_lux = 0;

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */

// SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
// static esp_err_t i2c_master_check_BMP280_chipid(i2c_port_t i2c_num, uint8_t *chipid)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, 0x76 << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, 0xD0, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(30 / portTICK_RATE_MS);
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, 0x76 << 1 | READ_BIT, ACK_CHECK_EN);

//     i2c_master_read(cmd, chipid, 1, LAST_NACK_VAL);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     return ret;
// }

uint8_t i2c_addr = 0;

void light_sensor_task(void *arg)
{
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    
    ESP_ERROR_CHECK(i2c_master_init());

    uint8_t sensor_data_h, sensor_data_l;
    // int cnt = 0;
    while (1) {
        // ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
        ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
        // xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ SENSOR( BH1750 )\n", task_idx);
            printf("*******************\n");
            // printf("data_h: %02x\n", sensor_data_h);
            // printf("data_l: %02x\n", sensor_data_l);
            light_sensor_lux = (sensor_data_h << 8 | sensor_data_l) / 1.2;
            printf("Light sensor val: %.02f [Lux]\n", light_sensor_lux);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }
        // xSemaphoreGive(print_mux);
        // vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
        //---------------------------------------------------
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    // vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

// void app_main(void)
// {
//     // print_mux = xSemaphoreCreateMutex();

//     ESP_ERROR_CHECK(i2c_master_init());
//     xTaskCreate(light_sensor_task, "i2c_read_BH1750_task", 1024 * 2, (void *)0, 10, NULL);

// }
