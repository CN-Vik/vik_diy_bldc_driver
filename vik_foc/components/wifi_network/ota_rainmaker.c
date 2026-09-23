#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"

static const char *TAG = "OTA_RAINMAKER";

/**
 * RainMaker OTA 回调函数
 * 注意：类型为 ota_status_t，返回值必须为 esp_err_t
 */
static esp_err_t ota_status_callback(ota_status_t status, void *priv)
{
    switch (status) {
        case OTA_STATUS_IN_PROGRESS:
            ESP_LOGI(TAG, "RainMaker OTA upgrade in progress...");
            break;

        case OTA_STATUS_SUCCESS:
            ESP_LOGI(TAG, "RainMaker OTA upgrade success. Rebooting...");
            
            // 此处可调用电机的紧急停机函数（如 foc_motor_emergency_stop()）
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
            break;

        case OTA_STATUS_FAILED:
            ESP_LOGE(TAG, "RainMaker OTA upgrade failed!");
            break;

        case OTA_STATUS_DELAYED:
            ESP_LOGW(TAG, "RainMaker OTA upgrade delayed.");
            break;

        default:
            ESP_LOGI(TAG, "RainMaker OTA status: %d", status);
            break;
    }
    return ESP_OK;
}

/**
 * 初始化并注册 RainMaker OTA 服务
 */
esp_err_t ota_rainmaker_init(void)
{
    ESP_LOGI(TAG, "Initializing RainMaker OTA...");

    esp_rmaker_ota_config_t ota_config = {
        .server_cert = ESP_RMAKER_OTA_DEFAULT_SERVER_CERT,
        .ota_cb = ota_status_callback,
        .priv = NULL,
    };

    esp_err_t err = esp_rmaker_ota_enable(&ota_config, OTA_USING_TOPICS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RainMaker OTA: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RainMaker OTA enabled successfully");
    return ESP_OK;
}