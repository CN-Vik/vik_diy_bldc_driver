/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "debug_protocol.h"
#include "app_rtos_resource.h"
#include "app_rtos_config.h"


#define CONFIG_EXAMPLE_IPV4         1
#define PORT                        2345
#define KEEPALIVE_IDLE              60    // 空闲60秒开始探测
#define KEEPALIVE_INTERVAL          10    // 每10秒重发一次
#define KEEPALIVE_COUNT             3     // 连续3次失败判断开

#define TCP_RX_BUF_SIZE             128

static const char *TAG = "tcp_server_eg";

TaskHandle_t tcp_server_task_handle = NULL;

extern const int CONNECTED_BIT;



/**
 * @brief tcp 发送数据
 * 
 * @param sock 
 * @param tx_data_p 发送数据的指针
 * @param len 发送数据的长度单位 bytes
 */
static void tcp_send_data_byte(const int sock, char *tx_data_p,int len)
{
    // send() can return less bytes than supplied length.
    // Walk-around for robust implementation.
    int to_write = len;
    while (to_write > 0) 
    {
        int written = send(sock, tx_data_p + (len - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Err_ocured_during_sending:errno %d", errno);
            // Failed to retransmit, giving up
            return;
        }
        to_write -= written;
    }
    ESP_LOGI(TAG, "tcp_tx_%d_byte: %s", len, tx_data_p);

}


static void tcp_recive_data(const int sock)
{
    int len;
    // uint8_t tcp_rx_buf[128];
    char *tcp_rx_buf = (char*)calloc(TCP_RX_BUF_SIZE,sizeof(char));

    do {
        len = recv(sock, tcp_rx_buf, TCP_RX_BUF_SIZE - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Err_occurred_during_receiving:errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Conect_closed");
        } else {
            // ESP_LOGI(TAG, "1tcp_rx_%d_byte: %s ,%c %d",
            //     len, 
            //     tcp_rx_buf,
            //     tcp_rx_buf[len-1],
            //     (int)tcp_rx_buf[len-1]
            // );
            tcp_rx_buf[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "tcp_rx_%d_byte: %s", len, tcp_rx_buf);
            debug_cmd_proces(tcp_rx_buf,&tcp_cmd_data_v);
        
            #if 0
                // send() can return less bytes than supplied length.
                // Walk-around for robust implementation.
                int to_write = len;
                while (to_write > 0) {
                    int written = send(sock, tcp_rx_buf + (len - to_write), to_write, 0);
                    if (written < 0) {
                        ESP_LOGE(TAG, "Err_ocured_during_sending:errno %d", errno);
                        // Failed to retransmit, giving up
                        return;
                    }
                    to_write -= written;
                }
            #else
                // tcp_send_data_byte(sock,tcp_rx_buf,len);
            #endif
        }
    } while (len > 0);

    if (tcp_rx_buf!=NULL)
    {
        free(tcp_rx_buf);
        tcp_rx_buf = NULL;
    }
    
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable_to_create_socket:errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket_created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket_unable_to_bind:errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket_bound_port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error_occurred_during_listen:errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) 
    {

        // 等待WiFi连接通知
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        EventBits_t uxBits = xEventGroupWaitBits(
            g_wifi_event_group,
            CONNECTED_BIT,
            false,
            false,
            portMAX_DELAY
        );

        if (uxBits & CONNECTED_BIT)
        {/* Wi-Fi 已经连接 */
            
            ESP_LOGI(TAG, "Socket_listening");
        }

        

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable_to_accept_connection:errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket_accepted_ip_addres: %s", addr_str);

        tcp_recive_data(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void tcp_server_main(void)
{
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    // /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    //  * Read "Establishing Wi-Fi or Ethernet Connection" section in
    //  * examples/protocols/README.md for more information about this function.
    //  */
    // ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreatePinnedToCore(
        tcp_server_task, //任务函数
        "tcp_server", //任务名称
        TCP_SERVER_TASK_STACK, //栈大小（4096字 = 16KB）
        (void*)AF_INET, //传递AF_INET参数
        TCP_SERVER_TASK_PRIO,// 优先级（0-24，数字越大优先级越高）
        &tcp_server_task_handle,
        TCP_SERVER_TASK_CORE
    );
#endif
}
