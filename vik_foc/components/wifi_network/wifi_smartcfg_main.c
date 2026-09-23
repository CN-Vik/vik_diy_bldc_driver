/* Wi-Fi SmartConfig + TCP Server */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"  // OTA操作相关函数
#include "app_rtos_config.h"
#include "app_rtos_resource.h"
#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"
#include "ota_rainmaker.h"


/* ============================================================
 * 配置
 * ============================================================ */

#define WIFI_MAX_RETRY          10

static const char *TAG = "wifi_smart_cfg_eg";

extern TaskHandle_t tcp_server_task_handle;
extern TaskHandle_t udp_client_task_handle;
extern TaskHandle_t ota_https_task_handle;




const int CONNECTED_BIT      = BIT0;
const int ESPTOUCH_DONE_BIT  = BIT1;
const int RUN_OTA_BIT  = BIT2;


/* ============================================================
 * 状态变量
 * ============================================================ */

static uint32_t s_retry_count = 0;

/* 防止 SmartConfig 任务重复创建 */
static bool s_smartconfig_running = false;

/* 防止 TCP Server 重复启动 */
static bool s_tcp_server_started = false;

//静态防重入标志位
static bool rmaker_started = false;


/* ============================================================
 * 外部 TCP Server
 * ============================================================ */
extern void tcp_server_main(void);
extern void udp_clinet_main(void);
extern void ota_https_main(void);
static void wifi_smart_cfg_task(void *parm);




/* ============================================================
 * 启动 SmartConfig
 * ============================================================ */
static void start_smartconfig(void)
{
    if (s_smartconfig_running)
    {
        ESP_LOGW(TAG, "SmartCfg_already_running");
        return;
    }

    s_smartconfig_running = true;

    ESP_LOGW(TAG, "======================================");
    ESP_LOGW(TAG, "Starting SmartConfig...");
    ESP_LOGW(TAG, "Please use ESPTouch APP");
    ESP_LOGW(TAG, "======================================");

    BaseType_t ret = xTaskCreatePinnedToCore(
        wifi_smart_cfg_task,
        "smartconfig_task",
        WIFI_SMART_CFG_TASK_STACK,
        NULL,
        WIFI_SMART_CFG_TASK_PRIO,
        NULL,
        WIFI_SMART_CFG_TASK_CORE
    );

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed_to_create_SmartConfig_task");
        s_smartconfig_running = false;
    }
}



/* ============================================================
 * Wi-Fi Event Handler
 * ============================================================ */
static void event_handler(void *arg,
                          esp_event_base_t event_base,
                          int32_t event_id,
                          void *event_data)
{
    /*HTTP OTA */
    if (event_base == ESP_HTTPS_OTA_EVENT) 
    {
        switch (event_id) 
        {
            case ESP_HTTPS_OTA_START:
                ESP_LOGI(TAG, "OTA started");
                break;
            case ESP_HTTPS_OTA_CONNECTED:
                ESP_LOGI(TAG, "Connected to server");
                break;
            case ESP_HTTPS_OTA_GET_IMG_DESC:
                ESP_LOGI(TAG, "Reading Image Description");
                break;
            case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                ESP_LOGI(TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t *)event_data);
                break;
            case ESP_HTTPS_OTA_DECRYPT_CB:
                ESP_LOGI(TAG, "Callback to decrypt function");
                break;
            case ESP_HTTPS_OTA_WRITE_FLASH:
                ESP_LOGD(TAG, "Writing to flash: %d written", *(int *)event_data);
                break;
            case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                ESP_LOGI(TAG, "Boot partition updated. Next Partition: %d", *(esp_partition_subtype_t *)event_data);
                break;
            case ESP_HTTPS_OTA_FINISH:
                ESP_LOGI(TAG, "OTA finish");
                break;
            case ESP_HTTPS_OTA_ABORT:
                ESP_LOGI(TAG, "OTA abort");
                break;
        }
    }

    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START)
    {
        /* --------------------------------------------------------
        * STA 启动
        * -------------------------------------------------------- */
    
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");

        esp_err_t ret = esp_wifi_connect();

        if (ret == ESP_OK)
        {
            /*
             * 注意：
             *
             * ESP_OK 只表示：
             * esp_wifi_connect() 调用成功
             *
             * 并不表示已经连接 AP。
             *
             * 真正连接成功要等：
             * IP_EVENT_STA_GOT_IP
             */

            ESP_LOGI(TAG, "WiFi_conection_attempt_started");
        }
        else
        {
            ESP_LOGE(TAG,
                     "esp_wifi_conect_failed: %s",
                     esp_err_to_name(ret));

            /*
             * 如果连发起连接都失败，
             * 这里直接启动 SmartConfig。
             */
            start_smartconfig();
        }

    }else if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        /* --------------------------------------------------------
         * STA 断开
         * -------------------------------------------------------- */
        xEventGroupClearBits(
            g_wifi_event_group,
            CONNECTED_BIT
        );

        ESP_LOGW(TAG,
                 "WiFi_disconect, retry = %lu/%d",
                 (unsigned long)s_retry_count,
                 WIFI_MAX_RETRY);

        /*
         * 如果 SmartConfig 正在运行，
         * 不要再进行普通重连。
         */
        if (s_smartconfig_running)
        {
            return;
        }

        /*
         * 重试
         */
        if (s_retry_count < WIFI_MAX_RETRY)
        {
            s_retry_count++;

            esp_err_t ret = esp_wifi_connect();

            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Reconecting_WiFi...");
            }
            else
            {
                ESP_LOGE(TAG,
                         "Reconect_failed: %s",
                         esp_err_to_name(ret));
            }
        }
        else
        {
            /*
             * 连续失败 10 次
             * 进入 SmartConfig
             */

            ESP_LOGW(TAG,
                     "WiFi_conect failed %d times",
                     WIFI_MAX_RETRY);

            s_retry_count = 0;

            start_smartconfig();
        }
   
    }else if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP)
    {
        /* --------------------------------------------------------
        * 真正连接成功，拿到 IP
        * -------------------------------------------------------- */

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        /*
         * 清零重试次数
         */
        s_retry_count = 0;

        /*
         * 设置连接成功标志
         */
        xEventGroupSetBits(
            g_wifi_event_group,
            CONNECTED_BIT
        );


        /* ----------------------------------------------------
         * 获取当前 Wi-Fi 配置
         * ---------------------------------------------------- */
        wifi_config_t wifi_config;

        memset(&wifi_config, 0, sizeof(wifi_config));

        esp_err_t ret =
            esp_wifi_get_config(
                WIFI_IF_STA,
                &wifi_config
            );

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "======================================");
            ESP_LOGI(TAG, "WiFi Connected!");
            ESP_LOGI(TAG,
                     "SSID     : %s",
                     (char *)wifi_config.sta.ssid);

            ESP_LOGI(TAG,
                     "PASSWORD : %s",
                     (char *)wifi_config.sta.password);

            ESP_LOGI(TAG,
                     "IP       : " IPSTR,
                     IP2STR(&event->ip_info.ip));

            ESP_LOGI(TAG,
                     "Gateway  : " IPSTR,
                     IP2STR(&event->ip_info.gw));

            ESP_LOGI(TAG,
                     "Netmask  : " IPSTR,
                     IP2STR(&event->ip_info.netmask));

            ESP_LOGI(TAG, "======================================");
        }
        else
        {
            ESP_LOGE(TAG,
                     "esp_wifi_get_config_failed: %s",
                     esp_err_to_name(ret));
        }


        /* ----------------------------------------------------
         * 启动 TCP Server
         * ---------------------------------------------------- */

        if (!s_tcp_server_started)
        {
            s_tcp_server_started = true;

            ESP_LOGI(TAG, "Starting_TCP_Server...");
            ESP_LOGI(TAG, "Starting_UDP_Client...");
            ESP_LOGI(TAG, "Starting_OTA_HTTPS...");
            tcp_server_main();
            udp_clinet_main();
            // ota_https_main();

            if (!rmaker_started) {
                esp_rmaker_config_t rainmaker_cfg = {
                    .enable_time_sync = true,
                };
                esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "VIK Balance Car", "BalanceCar");
                (void)node; // 防止报 unused variable 警告

                ota_rainmaker_init();//4. 启用云端 OTA
                esp_rmaker_start();//启动 RainMaker 核心 Agent
                
                rmaker_started = true;
            }

            /*wifi连接成功发送个任务通知或者事件标志*/
            // 通知所有等待的线程
            // if (udp_client_task_handle) {
            //     xTaskNotifyGive(udp_client_task_handle);
            // }
            // if (tcp_server_task_handle) {
            //     xTaskNotifyGive(tcp_server_task_handle);
            // }
            // if (ota_https_task_handle) {
            //     xTaskNotifyGive(ota_https_task_handle);
            // }
        }
    
    }else if (event_base == SC_EVENT &&
        event_id == SC_EVENT_SCAN_DONE)
    {
        /* ========================================================
        * SmartConfig Events
        * ======================================================== */
        ESP_LOGI(TAG, "SmartConfig: Scan_done");
    
    }else if (event_base == SC_EVENT &&
             event_id == SC_EVENT_FOUND_CHANNEL)
    {
        ESP_LOGI(TAG, "SmartConfig: Found channel");
    
    }else if (event_base == SC_EVENT &&
        event_id == SC_EVENT_GOT_SSID_PSWD)
    {
        /* --------------------------------------------------------
         * 收到手机发送的 SSID + PASSWORD
         * -------------------------------------------------------- */
        ESP_LOGI(TAG, "======================================");
        ESP_LOGI(TAG, "SmartConfig: Got SSID and Password");
        ESP_LOGI(TAG, "======================================");


        smartconfig_event_got_ssid_pswd_t *evt =
            (smartconfig_event_got_ssid_pswd_t *)event_data;


        wifi_config_t wifi_config;

        memset(&wifi_config, 0, sizeof(wifi_config));


        /*
         * 获取 SmartConfig 发送过来的 SSID
         */
        memcpy(
            wifi_config.sta.ssid,
            evt->ssid,
            sizeof(wifi_config.sta.ssid)
        );


        /*
         * 获取 SmartConfig 发送过来的密码
         */
        memcpy(
            wifi_config.sta.password,
            evt->password,
            sizeof(wifi_config.sta.password)
        );


        /* ----------------------------------------------------
         * 打印 SSID / Password
         * ---------------------------------------------------- */

        ESP_LOGI(TAG,
                 "SSID     : %s",
                 (char *)wifi_config.sta.ssid);

        ESP_LOGI(TAG,
                 "PASSWORD : %s",
                 (char *)wifi_config.sta.password);


#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP

        wifi_config.sta.bssid_set = evt->bssid_set;

        if (wifi_config.sta.bssid_set)
        {
            ESP_LOGI(TAG,
                     "Target AP MAC: " MACSTR,
                     MAC2STR(evt->bssid));

            memcpy(
                wifi_config.sta.bssid,
                evt->bssid,
                sizeof(wifi_config.sta.bssid)
            );
        }

#endif


        /* ----------------------------------------------------
         * 保存 Wi-Fi 配置
         *
         * ESP-IDF Wi-Fi NVS 开启的情况下，
         * esp_wifi_set_config() 会持久化保存配置。
         * ---------------------------------------------------- */

        esp_err_t ret =
            esp_wifi_set_config(
                WIFI_IF_STA,
                &wifi_config
            );

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "esp_wifi_set_config failed: %s",
                     esp_err_to_name(ret));

            return;
        }

        ESP_LOGI(TAG, "WiFi configuration saved");


        /*
         * 断开当前连接
         */
        esp_wifi_disconnect();


        /*
         * 重新连接新的 AP
         */
        ret = esp_wifi_connect();

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "Connecting to new AP...");
        }
        else
        {
            ESP_LOGE(TAG,
                     "esp_wifi_connect failed: %s",
                     esp_err_to_name(ret));
        }
    
    }else if (event_base == SC_EVENT &&
        event_id == SC_EVENT_SEND_ACK_DONE)
    {
        /* --------------------------------------------------------
         * SmartConfig ACK 完成
         * -------------------------------------------------------- */
        ESP_LOGI(TAG, "SmartConfig ACK done");

        xEventGroupSetBits(
            g_wifi_event_group,
            ESPTOUCH_DONE_BIT
        );
    }
}


/* ============================================================
 * 初始化 Wi-Fi
 * ============================================================ */

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(
        esp_netif_init()
    );

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    esp_netif_t *sta_netif =
        esp_netif_create_default_wifi_sta();

    assert(sta_netif);


    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );


    /*
     * 注册 Wi-Fi Event
     */
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &event_handler,
            NULL
        )
    );


    /*
     * 注册 IP Event
     */
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &event_handler,
            NULL
        )
    );


    /*
     * 注册 SmartConfig Event
     */
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            SC_EVENT,
            ESP_EVENT_ANY_ID,
            &event_handler,
            NULL
        )
    );

    /* HTTPS OTA */
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));


    /**  CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
     * We are treating successful WiFi connection as a checkpoint to cancel rollback
     * process and mark newly updated firmware image as active. For production cases,
     * please tune the checkpoint behavior per end application requirement.
     */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) 
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) 
        {
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) 
            {
                ESP_LOGI(TAG, "App is valid, rollback cancelled successfully");
            }else{
                ESP_LOGE(TAG, "Failed to cancel rollback");
            }
        }
    }

    /* Ensure to disable any WiFi power save mode, this allows best throughput
     * and hence timings for overall OTA operation.
    */
    esp_wifi_set_ps(WIFI_PS_NONE);

    // /* WIFI_PS_MIN_MODEM is the default mode for WiFi Power saving. When both
    //  * WiFi and Bluetooth are running, WiFI modem has to go down, hence we
    //  * need WIFI_PS_MIN_MODEM. And as WiFi modem goes down, OTA download time
    //  * increases.
    // */
    // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    /*
     * STA 模式
     */
    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );


    /*
     * 启动 Wi-Fi
     */
    ESP_ERROR_CHECK(
        esp_wifi_start()
    );
}


/* ============================================================
 * SmartConfig Task
 * ============================================================ */

static void wifi_smart_cfg_task(void *parm)
{
    ESP_LOGI(TAG,
             "Starting ESPTouch SmartConfig...");


    /*
     * 设置 SmartConfig 类型
     */
    ESP_ERROR_CHECK(
        esp_smartconfig_set_type(
            SC_TYPE_ESPTOUCH
        )
    );


    smartconfig_start_config_t cfg =
        SMARTCONFIG_START_CONFIG_DEFAULT();


    /*
     * 启动 SmartConfig
     */
    ESP_ERROR_CHECK(
        esp_smartconfig_start(&cfg)
    );


    while (1)
    {
        EventBits_t uxBits;

        uxBits =
            xEventGroupWaitBits(
                g_wifi_event_group,
                CONNECTED_BIT |
                ESPTOUCH_DONE_BIT,
                false,
                false,
                portMAX_DELAY
            );


        /*
         * Wi-Fi 已经连接
         */
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG,
                     "WiFi connected during SmartConfig");
        }


        /*
         * SmartConfig 完成
         */
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG,
                     "SmartConfig finished");

            esp_smartconfig_stop();

            s_smartconfig_running = false;

            vTaskDelete(NULL);
        }

        vTaskDelay( pdMS_TO_TICKS(1000) );
    }
}




/**
 * @brief 
 * 
 */
void wifi_smartcfg_main(void)
{
    // /*
    //  * 初始化 NVS
    //  */
    // esp_err_t ret = nvs_flash_init();

    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    //     ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(
    //         nvs_flash_erase()
    //     );

    //     ESP_ERROR_CHECK(
    //         nvs_flash_init()
    //     );
    // }
    // else
    // {
    //     ESP_ERROR_CHECK(ret);
    // }


    ESP_LOGI(TAG,
             "======================================");

    ESP_LOGI(TAG,
             "ESP32 WiFi SmartConfig + TCP Server");

    ESP_LOGI(TAG,
             "======================================");


    /*
     * 初始化 Wi-Fi
     */
    initialise_wifi();


    /*
     * wifi_smart_main 不退出
     */
    while (1)
    {
        // ESP_LOGI(TAG,"wifi_smart_main running");

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}