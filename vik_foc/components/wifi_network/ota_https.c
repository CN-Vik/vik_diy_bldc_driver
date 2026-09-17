/* Advanced HTTPS OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"

// #if CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
// #include "esp_efuse.h"
// #endif

// #if CONFIG_BT_BLE_ENABLED || CONFIG_BT_NIMBLE_ENABLED
// #include "ble_api.h"
// #endif


#define OTA_URL_SIZE 256

#define OTA_HTTP_URL            "http://192.168.1.138:8070/Desktop/project/esp32c3/project/hello_world_example/build/hello_world_example.bin"
#define OTA_HTTP_RECV_TIMEOUT   5000



static const char *TAG = "OTA_HTTPS";
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

extern const int CONNECTED_BIT;
extern const int ESPTOUCH_DONE_BIT;
extern EventGroupHandle_t g_wifi_event_group;
TaskHandle_t ota_https_task_handle;




static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
    }

#if 0
    #ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK   /*跳过版本检查*/
        if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0) {
            ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
            return ESP_FAIL;
        }
    #endif

    #ifdef CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
        /**
         * Secure version check from firmware image header prevents subsequent download and flash write of
         * entire firmware image. However this is optional because it is also taken care in API
         * esp_https_ota_finish at the end of OTA update procedure.
         */
        const uint32_t hw_sec_version = esp_efuse_read_secure_version();
        if (new_app_info->secure_version < hw_sec_version) {
            ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed, %"PRIu32" < %"PRIu32, new_app_info->secure_version, hw_sec_version);
            return ESP_FAIL;
        }
    #endif

#endif

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    /* Uncomment to add custom headers to HTTP request */
    // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
    return err;
}

void ota_https_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start_OTA_HTTPS");

    /* ================================================================
     * 整个 OTA 流程包在一个死循环里：
     *   成功 → esp_restart() 重启
     *   失败 → 等5秒 → continue 回到开头重试
     * 再也不会 vTaskDelete 自杀
     * ================================================================ */
    while (1)
    {
        /* ------------------------------------------------------------
         * 第 1 步：先等 WiFi 连上（最关键！begin 之前必须有网）
         * ------------------------------------------------------------ */
        EventBits_t uxBits = xEventGroupWaitBits(
            g_wifi_event_group,
            CONNECTED_BIT,
            false,          /* 不清除标志位，其他任务也能读 */
            false,          /* 任意一个bit满足就返回 */
            portMAX_DELAY   /* 死等 */
        );
        if (!(uxBits & CONNECTED_BIT))
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "WiFi connected, starting OTA...");

        /* ------------------------------------------------------------
         * 第 2 步：配置 HTTP 客户端
         * ------------------------------------------------------------ */
        esp_http_client_config_t config = {
            .url = OTA_HTTP_URL,
            .cert_pem = (char *)server_cert_pem_start,  /* HTTP时忽略，保留无妨 */
            .timeout_ms = OTA_HTTP_RECV_TIMEOUT,
            .keep_alive_enable = true,
        };

        esp_https_ota_config_t ota_config = {
            .http_config = &config,
            .http_client_init_cb = _http_client_init_cb,
        };

        /* ------------------------------------------------------------
         * 第 3 步：WiFi 连上了才 begin（原来的bug就是这步跑太前面了）
         * ------------------------------------------------------------ */
        esp_https_ota_handle_t https_ota_handle = NULL;
        esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;  /* 失败不自杀，5秒后重试 */
        }

        /* ------------------------------------------------------------
         * 第 4 步：读取并校验固件头
         * ------------------------------------------------------------ */
        esp_app_desc_t app_desc;
        err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "get img desc failed");
            esp_https_ota_abort(https_ota_handle);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (validate_image_header(&app_desc) != ESP_OK)
        {
            ESP_LOGE(TAG, "image header verification failed");
            esp_https_ota_abort(https_ota_handle);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* ------------------------------------------------------------
         * 第 5 步：下载循环（一块块读固件写到Flash）
         * ------------------------------------------------------------ */
        while (1)
        {
            err = esp_https_ota_perform(https_ota_handle);
            if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
            {
                break;  /* 下载完成或出错，跳出循环 */
            }
            /* 打印已下载字节数，方便看进度 */
            ESP_LOGI(TAG, "Image bytes read: %d",
                     esp_https_ota_get_image_len_read(https_ota_handle));
        }

        /* ------------------------------------------------------------
         * 第 6 步：判断下载是否完整
         * ------------------------------------------------------------ */
        if (esp_https_ota_is_complete_data_received(https_ota_handle))
        {
            /* 数据完整 → 收尾（校验签名、设置启动分区） */
            esp_err_t finish_err = esp_https_ota_finish(https_ota_handle);
            if (finish_err == ESP_OK)
            {
                ESP_LOGI(TAG, "OTA upgrade successful. Rebooting...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();  /* 成功 → 重启跑新固件 */
            }
            else
            {
                if (finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
                {
                    ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                }
                ESP_LOGE(TAG, "OTA finish failed: 0x%x", finish_err);
            }
        }
        else
        {
            /* 数据不完整 → 中止 */
            ESP_LOGE(TAG, "Complete data was not received.");
            esp_https_ota_abort(https_ota_handle);
        }

        /* ------------------------------------------------------------
         * 第 7 步：能走到这里说明失败了，等5秒后回到循环开头重试
         * ------------------------------------------------------------ */
        ESP_LOGW(TAG, "OTA failed, retry in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}



void ota_https_main(void)
{ 
    xTaskCreate( 
        ota_https_task, // 任务函数 
        "ota_https_task", // 任务名称（最多16字符）
        1024 * 8, // 栈大小（4096字 = 16KB）
        NULL, 
        12, // 优先级（0-24，数字越大优先级越高）
        &ota_https_task_handle 
    );
}
