/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif
// #include "BME280_BMP280_BMP180.h"
#include "air_sensor_1101.h"
extern bool g_running_need_reset_flag;// used to indicate if rountine reset is required, which could prevent data error.
extern volatile bool g_uart_read_fail_flag;// used to indicate if uart communication is error and need reset to recover.
extern uint32_t g_running_cnt;//in app_main thread, which increments every 1 sec.
extern bool g_repeat_time_reach_threshold;// if generate_one_data_line_csv() receive too much repeat value,will set this flag to true.
static const char *TAG = "+SD-CARD+";
extern ENV_DATA g_env_data;
extern float light_sensor_lux;
extern float noise_sensor_db;
const int g_SD_CARD_WRITE_INTERVAL_MS = 60 * 1000;// every 60 sec 
// extern ATMOSPHERE_DATA atmosphere_data;
extern int generate_one_data_line_csv(char* buffer, size_t buffer_size, ENV_DATA* air);
#define CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED 1
char line_buffer[256] = {0};
char log_buffer[128] = {0};
// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:
#define MOUNT_POINT "/sdcard"
#define USE_SPI_MODE

// ESP32-S2 and ESP32-C3 doesn't have an SD Host peripheral, always use SPI:
#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#ifndef USE_SPI_MODE
#define USE_SPI_MODE
#endif // USE_SPI_MODE
// on ESP32-S2, DMA channel must be the same as host id
#define SPI_DMA_CHAN    host.slot
#endif //CONFIG_IDF_TARGET_ESP32S2

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13

#elif CONFIG_IDF_TARGET_ESP32C3
#define PIN_NUM_MISO 18
#define PIN_NUM_MOSI 9
#define PIN_NUM_CLK  8
#define PIN_NUM_CS   19

#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#endif //USE_SPI_MODE



void sd_card_task(void *arg)
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;

    while(1){
        ESP_LOGI(TAG, "===> Initializing SD card ==>");

        // Use settings defined above to initialize SD card and mount FAT filesystem.
        // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
        // Please check its source code and implement error recovery when developing
        // production applications.
        #ifndef USE_SPI_MODE
        ESP_LOGI(TAG, "Using SDMMC peripheral");
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();

        // This initializes the slot without card detect (CD) and write protect (WP) signals.
        // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

        // To use 1-line SD mode, uncomment the following line:
        slot_config.width = 1;

        // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
        // Internal pull-ups are not sufficient. However, enabling internal pull-ups
        // does make a difference some boards, so we do that here.
        gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
        gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
        gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
        gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
        gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

        ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    #else
        ESP_LOGI(TAG, "Using SPI peripheral");

        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000,
        };
        ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize bus.");
            return;
        }

        // This initializes the slot without card detect (CD) and write protect (WP) signals.
        // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = PIN_NUM_CS;
        slot_config.host_id = host.slot;

        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    #endif //USE_SPI_MODE

        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                ESP_LOGE(TAG, "Failed to mount filesystem. "
                    "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
            } else {
                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
            }
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;//wait some time then continue to re-initialize the sd card.
        }else{
            break;// sd card initialized.
        }
    }


    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen(MOUNT_POINT"/data.csv", "a+");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    memset(line_buffer, 0, sizeof(line_buffer));
    while(1)
    {
        // this parse sensor data into data.csv every 1sec.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        #if 1
        if (g_uart_read_fail_flag || g_repeat_time_reach_threshold || g_running_need_reset_flag)
        {
            //
            FILE* f = fopen(MOUNT_POINT"/log.csv", "a+");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing");
            }else{
                int str_len = 0;
                if(g_running_need_reset_flag)
                {
                    str_len = sprintf(log_buffer, "%u g_running_need_reset_flag=true, need to do reset.\n", g_running_cnt);
                }else if(g_uart_read_fail_flag){
                    str_len = sprintf(log_buffer, "%u g_uart_read_fail_flag=true, need to do reset.\n", g_running_cnt);
                }else{
                    // g_repeat_time_reach_threshold
                    str_len = sprintf(log_buffer, "%u g_repeat_time_reach_threshold=true, need to do reset.\n",g_running_cnt);
                }
                
                fwrite(log_buffer, 1, str_len, f);
                fclose(f);
            }
            ESP_LOGW(TAG, "!!! uart read sensor data failure  threshold reached and need to do reset.");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }
        #endif
        int len = generate_one_data_line_csv(line_buffer, sizeof(line_buffer), &g_env_data);

        // write to 
        if(len > 0)
        {
            // ESP_LOGI(TAG, "Opening file");
            FILE* f = fopen(MOUNT_POINT"/data.csv", "a+");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing");
                continue;
            }
            int res = fwrite(line_buffer, 1, len, f);
            fclose(f);

            if(res != len){
                ESP_LOGE(TAG, "~~~  write to csv file error: res:%d != len:%d", res, len);
            }else{
                ESP_LOGI(TAG, "%d bytes data save to SD card .csv file.!!!", res);
            }
            memset(line_buffer, 0, sizeof(line_buffer));
        }else{
            ESP_LOGE(TAG, "generate_one_data_line_csv = %d <=0!!! ", len);
        }
    }

#if 0
    // Check if destination file exists before renaming
    struct stat st;
    if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/foo.txt");
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen(MOUNT_POINT"/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

#ifdef USE_SPI_MODE
    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
#endif
#endif
}
