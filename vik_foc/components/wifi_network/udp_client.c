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
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "udp_logger.h"
#include "app_rtos_config.h"
#include "app_rtos_resource.h"



#define DEST_IP_ADDR    ("192.168.1.138") /*目标IP地址*/
#define UDP_DEST_PORT       (3456) /*目标端口*/

#define UDP_LOCAL_PORT   (8586) /*本地端口*/

#define UDP_RX_BUF_LEN      (128)


static const char *TAG = "udp_cleint_eg";
static const char *udp_tx_data = "Message_from_ESP32_Vik\r\n";

TaskHandle_t udp_client_task_handle = NULL;
extern const int CONNECTED_BIT;


void udp_client_recive_data(int net_udp_sock)
{
    // char udp_rx_buf[128];
    char *udp_rx_buf = (char*)calloc(UDP_RX_BUF_LEN,sizeof(char));
    if (udp_rx_buf==NULL)
    {
        return;
    }
    
    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    int udp_rx_len = recvfrom(
        net_udp_sock,
        udp_rx_buf,
        UDP_RX_BUF_LEN - 1,
        0,
        (struct sockaddr *)&source_addr,
        &socklen
    );

    // Error occurred during receiving
    if (udp_rx_len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        
    }else {// Data received
        
        udp_rx_buf[udp_rx_len] = 0; // Null-terminate whatever we received and treat like a string
        ESP_LOGI(TAG, "udp_cleint_Recv:%d bytes", udp_rx_len);
        ESP_LOGI(TAG, "udp_rx_data:%s", udp_rx_buf);
    }


    if (udp_rx_buf!=NULL)
    {
        free(udp_rx_buf);
        udp_rx_buf = NULL;
    }
    

}


static void udp_client_task(void *pvParameters)
{

    while (1) 
    {
        struct sockaddr_in dest_addr={
            .sin_addr.s_addr = inet_addr(DEST_IP_ADDR),
            .sin_family = AF_INET,
            .sin_port = htons(UDP_DEST_PORT)
        };

        int udp_client_sock = socket(
            AF_INET, // AF_INET表示使用IPv4协议
            SOCK_DGRAM, // SOCK_DGRAM表示使用数据报模式,也就是UDP协议
            IPPROTO_IP // IPPROTO_IP表示使用IP协议
        );
        if (udp_client_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        /*绑定ESP32的固定端口*/
        struct sockaddr_in esp32_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(UDP_LOCAL_PORT),/*esp32的固定端口*/
            .sin_addr.s_addr = htonl(INADDR_ANY)
        };
        bind(udp_client_sock, (struct sockaddr*)&esp32_addr, sizeof(esp32_addr) );

        // Set timeout 10s
        struct timeval timeout;
        timeout.tv_sec = 3;
        timeout.tv_usec = 0;
        setsockopt(udp_client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket_created,_sending_to %s:%d", DEST_IP_ADDR, UDP_DEST_PORT);

        // float sped = 0.0f;

        while (1) 
        {
            // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);// 等待WiFi连接通知
            EventBits_t uxBits = xEventGroupWaitBits(
                g_wifi_event_group,
                CONNECTED_BIT,
                false,
                false,
                portMAX_DELAY
            );

            if (uxBits & CONNECTED_BIT)
            {/* Wi-Fi 已经连接 */
                
                // ESP_LOGI(TAG, "udp_client_runing!\r\n");

            }else{

                ESP_LOGI(TAG, "udp_client_wifi_not conect!\r\n");
            }
            


            // 普通信息打印 (Level: INFO)
            // UDP_LOGI("UDP", "Mot%.1f\r\n", 5.6f);

            // sped +=0.01f;
            // if (sped>20.0f)
            // {
            //     sped = 0.0f;
            // }
            
            // UDP_LOGI("UDP", "Hi_Vik: %.3f\r\n",sped);
            
            
            #if 0
                int err = sendto( 
                    udp_client_sock,
                    udp_tx_data,
                    strlen(udp_tx_data),
                    0,
                    (struct sockaddr *)&dest_addr,
                    sizeof(dest_addr)
                );
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                ESP_LOGI(TAG, "udp_mesge_sent23");

                udp_client_recive_data(udp_client_sock);
            #else
                char *tx_buf = NULL;
                // 阻塞等待队列中的日志数据，无限等待 (portMAX_DELAY)
                if (xQueueReceive(udp_log_queue, &tx_buf, portMAX_DELAY) == pdTRUE) {
                    
                    // 发送数据
                    sendto(
                        udp_client_sock,
                        tx_buf,
                        strlen(tx_buf), 
                        0, 
                        (struct sockaddr *)&dest_addr, 
                        sizeof(dest_addr)
                    );

                    if (tx_buf!=NULL)
                    {//队列发过来得是块堆上得内存，使用完了需要释放掉
                     
                        free(tx_buf);
                        tx_buf = NULL;
                    }
                    
                }
            #endif
            vTaskDelay( pdMS_TO_TICKS(100) );

        }

        if (udp_client_sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(udp_client_sock, 0);
            close(udp_client_sock);
        }
    }
    vTaskDelete(NULL);
}

void udp_clinet_main(void)
{
    xTaskCreatePinnedToCore(
        udp_client_task, // 任务函数
        "udp_client", // 任务名称（最多16字符）
        UDP_CLIENT_TASK_STACK, // 栈大小（4096字 = 16KB）
        NULL,
        UDP_CLIENT_TASK_PRIO, // 优先级（0-24，数字越大优先级越高）
        &udp_client_task_handle,
        UDP_CLIENT_TASK_CORE
    );
}
