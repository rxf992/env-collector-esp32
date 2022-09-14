#include "my_network.h"
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "air_sensor_1101.h"
//WIFI
#define EXAMPLE_ESP_WIFI_SSID      "007-ENVIRO-SENSING"
#define EXAMPLE_ESP_WIFI_PASS      "lichang-007"
#define EXAMPLE_ESP_WIFI_CHANNEL   (11)
#define EXAMPLE_MAX_STA_CONN       (99)
static esp_ip4_addr_t broadcast_addr = {0}; //广播地址
static char broadcast_ip_addr[20] = {0};
static TaskHandle_t TaskHandle=NULL; 
static char output_buffer[256] = {0};
static const char *TAG = "+TASK_MY_NETWORK+";
const int g_UDP_REPORT_INTERVAL_MS = 1 * 1000;// every xx sec 
extern ENV_DATA g_env_data;
extern int generate_one_data_line_csv(char* buffer, size_t buffer_size, const ENV_DATA* air);
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
    ESP_LOGI(TAG, "broadcast_ip_addr=%s", broadcast_ip_addr);
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
            int len = generate_one_data_line_csv(output_buffer, sizeof(output_buffer), &g_env_data);

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
            ESP_LOGE(TAG, "!!! Create UDP Task!!! ");
        }
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
            MAC2STR(event->mac), event->aid);
    }
}


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
    while(1)
    {
        if(ESP_OK == esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info)){
            printf("IP:\t"IPSTR"\n",IP2STR(&ip_info.ip));
            printf("MASK:\t"IPSTR"\n",IP2STR(&ip_info.netmask));
            printf("GW:\t"IPSTR"\n",IP2STR(&ip_info.gw));
            broadcast_addr.addr = (~ip_info.netmask.addr) | ip_info.ip.addr;
            printf("Broadcast IP Addr: "IPSTR"\n", IP2STR(&broadcast_addr));
            break;
        }else{
            printf("GET IP INFO Failed.");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    
}
