/* BME280 BMP280 BMX180 example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "my_iic.h"

#define BMX280_I2C_ADDR 0x76///根据硬件配置修改

#define BMP180_CHIP_ID 0x55

#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60
#define BMX280_RESET_VALUE 0xB6

#define BMP280_DIG_AC1_MSB_REG               0xAA

#define BMP280_DIG_T1_LSB_REG                0x88
#define BMP280_DIG_H1_LSB_REG                0xA1
#define BMP280_DIG_H2_LSB_REG                0xE1

#define BMX280_CHIPID_REG                    0xD0  /*Chip ID Register */
#define BMX280_RESET_REG                     0xE0  /*Softreset Register */
#define BME280_CTRLHUM_REG                   0xF2  /*Ctrl Humidity Register */
#define BMX280_STATUS_REG                    0xF3  /*Status Register */
#define BMX280_CTRLMEAS_REG                  0xF4  /*Ctrl Measure Register */
#define BMX280_CONFIG_REG                    0xF5  /*Configuration Register */
#define BMX280_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */
#define BMX280_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */
#define BMX280_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */
#define BMX280_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */
#define BMX280_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */
#define BMX280_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */

#define BMP180_OSS_1TIMES_RATE  0    /* wait 4.5ms */
#define BMP180_OSS_2TIMES_RATE  1    /* wait 7.5ms */
#define BMP180_OSS_4TIMES_RATE  2    /* wait 13.5ms */
#define BMP180_OSS_8TIMES_RATE  3    /* wait 25.5ms */

#define BMP180_START_CONVERSION     1<<5

#define BMP180_READ_TEMPERATURE     0x0e
#define BMP180_READ_PRESSURE        0x14

#define BMP180_DATA_MSB_REG         0xF6    /* MSB Reg */
#define BMP180_DATA_LSB_REG         0xF7    /* LSB Reg */
#define BMP180_DATA_XLSB_REG        0xF8    /* XLSB Reg */

void i2c_task_bme280_bmp280_bmp180(void *arg);


