#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_event.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"

static const char *TAG = "OTA_RAINMAKER";

/**
 * RainMaker OTA 回调函数
 * 注意：签名必须为 esp_rmaker_ota_cb_t
 *   esp_err_t (*)(esp_rmaker_ota_handle_t handle, esp_rmaker_ota_data_t *ota_data)
 */
static esp_err_t ota_status_callback(esp_rmaker_ota_handle_t ota_handle, esp_rmaker_ota_data_t *ota_data)
{
    if (ota_data == NULL) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RainMaker OTA request received:");
    ESP_LOGI(TAG, "  fw_version : %s", ota_data->fw_version ? ota_data->fw_version : "unknown");
    ESP_LOGI(TAG, "  ota_job_id : %s", ota_data->ota_job_id ? ota_data->ota_job_id : "unknown");
    ESP_LOGI(TAG, "  url        : %s", ota_data->url ? ota_data->url : "unknown");

    // 调用 RainMaker 默认回调执行固件下载、校验与刷写
    // 升级成功后内部会自动重启设备
    esp_err_t err = esp_rmaker_ota_default_cb(ota_handle, ota_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RainMaker OTA failed: %s", esp_err_to_name(err));
    }
    return err;
}

/**
 * OTA 完成后的重启事件处理函数
 * 当 CONFIG_ESP_RMAKER_OTA_DISABLE_AUTO_REBOOT=y 时，SDK 不会自动重启，
 * 而是发送 RMAKER_OTA_EVENT_REQ_FOR_REBOOT 事件，由应用层在此执行重启。
 */
static void ota_reboot_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data)
{
    if (event_base == RMAKER_OTA_EVENT && event_id == RMAKER_OTA_EVENT_REQ_FOR_REBOOT) {
        ESP_LOGI(TAG, "OTA reboot requested, rebooting into new firmware...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

/**
 * 初始化并注册 RainMaker OTA 服务
 */
esp_err_t ota_rainmaker_init(void)
{
    ESP_LOGI(TAG, "Initializing RainMaker OTA...");

    // 注册 OTA 完成后的重启事件处理函数
    esp_err_t err = esp_event_handler_register(RMAKER_OTA_EVENT, RMAKER_OTA_EVENT_REQ_FOR_REBOOT,
                                               &ota_reboot_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register OTA reboot handler: %s", esp_err_to_name(err));
        return err;
    }

    esp_rmaker_ota_config_t ota_config = {
        .server_cert = ESP_RMAKER_OTA_DEFAULT_SERVER_CERT,
        .ota_cb = ota_status_callback,
        .priv = NULL,
    };

    err = esp_rmaker_ota_enable(&ota_config, OTA_USING_TOPICS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RainMaker OTA: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RainMaker OTA enabled successfully");
    return ESP_OK;
}