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

#include "my_iic.h"
#include "light_sensor.h"
#include "air_sensor.h"
#include "noise_sensor.h"
#include "BME280_BMP280_BMP180.h"
#include "sd_card.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#define PORT 3333

// char payload[2048] = "Incoming Message from ESP32\n";

extern AIR_DATA air_data;
extern float light_sensor_lux;
extern float noise_sensor_db;
const int g_UDP_REPORT_INTERVAL_MS = 60 * 1000;// every 60 sec 
static const char *TAG = "+TASK_APP_MAIN+";
esp_ip4_addr_t broadcast_addr = {0}; //广播地址
char broadcast_ip_addr[20] = {0};
TaskHandle_t TaskHandle=NULL; 
char output_buffer[256] = {0};

//LED Status light
#define GPIO_OUTPUT_IO_0    2
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

//////////////////UDP client broadcast part ////////////////
static inline char* get_string_address(struct sockaddr_storage *source_addr)
{
    static char address_str[40]; // 40=(8*4+7+term) is the max size of ascii IPv6 addr "XXXX:XX...XX:XXXX"
    char *res = NULL;
    // Convert ip address to string
    if (source_addr->ss_family == PF_INET) {
        res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr, address_str, sizeof(address_str));
    }
#ifdef CONFIG_LWIP_IPV6
    else if (source_addr->ss_family == PF_INET6) {
        res = inet6_ntoa_r(((struct sockaddr_in6 *)source_addr)->sin6_addr, address_str, sizeof(address_str));
    }
#endif
    if (!res) {
        address_str[0] = '\0'; // Returns empty string if conversion didn't succeed
    }
    return address_str;
}
int generate_one_data_line_csv(char* buffer, size_t buffer_size, AIR_DATA air, float noise, float light, ATMOSPHERE_DATA atmosphere_data)
{
    // parse to buffer.
    // parse air to buffer.
    int res = sprintf(buffer, "%d,%d,%d,%d,%d,%.2f,%d.%d,%.2f,%.2f,%.2f\n", 
                        air.CH2O, air.CO2, air.TVOC, air.PM25, air.PM10, atmosphere_data.temp, air.Hummidity_int, air.Hummidity_float,
                        noise, light, atmosphere_data.pressure);
    
    return res;
}
static void udp_client_task(void *pvParameters)
{
    
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;
        
    while(strlen(broadcast_ip_addr)<10){
        ESP_LOGE(TAG, "broadcast_ip_addr=%s", broadcast_ip_addr);
        sprintf(broadcast_ip_addr, ""IPSTR"", IP2STR(&broadcast_addr) );
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
    }
    while (1) {

        
        dest_addr.sin_addr.s_addr = inet_addr(broadcast_ip_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);


        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", broadcast_ip_addr, PORT);

        
        while (1) {
            /// 生成一行新的csv数据
            memset(output_buffer, 0, sizeof(output_buffer));
            int len = generate_one_data_line_csv(output_buffer, sizeof(output_buffer), air_data, noise_sensor_db, light_sensor_lux, atmosphere_data);

            int err = sendto(sock, output_buffer, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "!!! Message sent, len=%d bytes!!", len);
            ESP_LOGI(TAG,"%s", output_buffer);
            vTaskDelay(g_UDP_REPORT_INTERVAL_MS / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

//////////////////////////////// WIFI Part //////////////////////////////////////
//WIFI
#define EXAMPLE_ESP_WIFI_SSID      "007-ENVIRO-SENSING"
#define EXAMPLE_ESP_WIFI_PASS      "lichang-007"
#define EXAMPLE_ESP_WIFI_CHANNEL   (11)
#define EXAMPLE_MAX_STA_CONN       (99)

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
            MAC2STR(event->mac), event->aid);

        //////////查询udp 任务是否存在，如果不存在则创建新任务，否则什么都不做
           
        // TaskStatus_t TaskStatus;

        TaskHandle=xTaskGetHandle("task_udp");         //根据任务名获取任务句柄
        if (TaskHandle == NULL){
            xTaskCreate(udp_client_task, "task_udp", 4096, NULL, 5, NULL);
            ESP_LOGI(TAG, "!!! Create UDP Task!!! ");
        }
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
            MAC2STR(event->mac), event->aid);
    }
}
#if 1
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
        EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
    
    
    //检查IP地址分配并计算广播地址
    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info));

    printf("IP:\t"IPSTR"\n",IP2STR(&ip_info.ip));
    printf("MASK:\t"IPSTR"\n",IP2STR(&ip_info.netmask));
    printf("GW:\t"IPSTR"\n",IP2STR(&ip_info.gw));
    broadcast_addr.addr = (~ip_info.netmask.addr) | ip_info.ip.addr;
    printf("Broadcast IP Addr: "IPSTR"\n", IP2STR(&broadcast_addr));
    
}
#endif
///////////////////UART related ////////////////////////
#define ECHO_TASK_STACK_SIZE        (2048)
// #define BUF_SIZE (1024)



static void gpio_setup()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
void app_main(void)
{
    printf("Hello world!\n");
    

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    ESP_ERROR_CHECK(i2c_master_init());

    // xTaskCreate(sd_card_task, "sd_card_task", 4096, NULL, 10, NULL);
    // xTaskCreate(light_sensor_task, "i2c_read_BH1750_task", 1024 * 2, (void *)0, 10, NULL);
    // xTaskCreate(air_sensor_task, "uart_air_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    // xTaskCreate(noise_sensor_task, "uart_noise_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
    xTaskCreate(i2c_task_bme280_bmp280_bmp180, "i2c_bmp280_task", 2048, NULL, 10, NULL);    
    
    // gpio_setup();
    int level = 0;
    while (1) {
        // gpio_set_level(GPIO_OUTPUT_IO_0, level);//GPIO2 also SD_D2 conflict.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        level = !level;
    }
}
