#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_ota.h"

static const char *TAG = "OTA_RAINMAKER";

/* 弱函数：OTA 前的电机停机。由 main 组件提供强定义（ota_safety.c）。
 * 若 main 未提供，则退化为空操作，保证本组件可独立编译。 */
__attribute__((weak)) void app_ota_prepare_shutdown(void)
{
}

/* ============================================================
 * 国内 CDN 固件地址（个体户方案：RainMaker 只当遥控器，
 * 固件大文件走国内 OSS/CDN，下载满速、不按国际流量计费）
 *
 * 固定文件名：每次发新版，直接覆盖上传 vik_foc.bin 即可。
 * 版本判断不靠文件名，靠 RainMaker 下发的 fw_version
 * 与设备当前运行版本做对比（相同则跳过升级）。
 * ============================================================ */
#define OTA_CN_URL  "https://esp32-ota-vik.oss-cn-shenzhen.aliyuncs.com/vik_foc.bin"

/* 国内 CDN 下载失败后的重试次数，全部失败再回退 AWS 原链路 */
#define OTA_CN_MAX_ATTEMPTS  2

/* 国内链路下载超时/缓冲
 * 注意：缓冲区不是越大越快，瓶颈是写 flash。设备跑 FOC 电机时
 * 可用堆紧张，4KB 足够、且能稳定分配成功（32KB 会 ESP_ERR_NO_MEM） */
#define OTA_CN_HTTP_TIMEOUT_MS   15000
#define OTA_CN_RX_BUFFER_SIZE    4096

/* ============================================================
 * 国内 CDN 下载实现：从 OSS/CDN 拉取固件并刷写
 * 返回 ESP_OK 表示升级成功（内部已设置启动分区），
 * 返回 ESP_FAIL 表示失败，由上层决定是否回退 AWS 链路。
 * ============================================================ */
static esp_err_t ota_download_from_cn_cdn(const char *url, char *additional_info, size_t info_len)
{
    esp_err_t err = ESP_OK;

    esp_http_client_config_t http_config = {
        .url = url,
        .timeout_ms = OTA_CN_HTTP_TIMEOUT_MS,
        .buffer_size = OTA_CN_RX_BUFFER_SIZE,
        .keep_alive_enable = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    ESP_LOGI(TAG, "Downloading firmware from domestic CDN: %s", url);

    esp_https_ota_handle_t https_ota_handle = NULL;
    err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK) {
        snprintf(additional_info, info_len, "CN CDN begin failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* 下载循环：一块块读固件写到 flash，顺带打进度看速度 */
    int64_t t_start = esp_timer_get_time();
    int last_reported_kb = 0;
    while (1) {
        err = esp_https_ota_perform(https_ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;  /* 下载完成或出错，跳出 */
        }
        int downloaded = esp_https_ota_get_image_len_read(https_ota_handle);
        int downloaded_kb = downloaded / 1024;
        if (downloaded_kb - last_reported_kb >= 128) {
            int64_t elapsed_ms = (esp_timer_get_time() - t_start) / 1000;
            int speed_kbps = elapsed_ms > 0 ? (int)(downloaded_kb * 1000 / elapsed_ms) : 0;
            ESP_LOGI(TAG, "CN download progress: %d KB, %d KB/s", downloaded_kb, speed_kbps);
            last_reported_kb = downloaded_kb;
        }
    }

    if (!esp_https_ota_is_complete_data_received(https_ota_handle)) {
        esp_https_ota_abort(https_ota_handle);
        snprintf(additional_info, info_len, "CN CDN download incomplete: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* 数据完整 → 收尾（校验镜像、设置启动分区） */
    esp_err_t finish_err = esp_https_ota_finish(https_ota_handle);
    if (finish_err != ESP_OK) {
        snprintf(additional_info, info_len, "CN CDN finish failed: %s", esp_err_to_name(finish_err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CN CDN firmware download & write successful");
    return ESP_OK;
}

/**
 * RainMaker OTA 回调函数
 * 注意：签名必须为 esp_rmaker_ota_cb_t
 *   esp_err_t (*)(esp_rmaker_ota_handle_t handle, esp_rmaker_ota_data_t *ota_data)
 *
 * 策略（个体户性价比方案）：
 *   1. 版本相同 → 直接返回成功，不重复升级
 *   2. 从国内 CDN 下载（快），失败重试 OTA_CN_MAX_ATTEMPTS 次
 *   3. 国内链路全部失败 → 回退 RainMaker 默认 AWS 链路兜底
 *   4. 全程上报状态，APP 上照常能看到进度
 */
static esp_err_t ota_status_callback(esp_rmaker_ota_handle_t ota_handle, esp_rmaker_ota_data_t *ota_data)
{
    if (ota_data == NULL) {
        return ESP_FAIL;
    }

    const char *new_ver = ota_data->fw_version ? ota_data->fw_version : "unknown";
    ESP_LOGI(TAG, "RainMaker OTA request received:");
    ESP_LOGI(TAG, "  fw_version : %s", new_ver);
    ESP_LOGI(TAG, "  ota_job_id : %s", ota_data->ota_job_id ? ota_data->ota_job_id : "unknown");
    ESP_LOGI(TAG, "  aws_url    : %s", ota_data->url ? ota_data->url : "unknown");

    /* 0. 版本比较：与当前运行固件相同则无需升级 */
    const esp_app_desc_t *running = esp_app_get_description();
    if (running != NULL && strcmp(new_ver, running->version) == 0) {
        ESP_LOGI(TAG, "New version %s equals running version, skip OTA", new_ver);
        esp_rmaker_ota_report_status(ota_handle, OTA_STATUS_SUCCESS, "Already running this version");
        return ESP_OK;
    }

    /* 0.5 OTA 前电机安全停机：关 MOS、挂起 FOC 任务，释放 CPU 和堆 */
    app_ota_prepare_shutdown();

    /* 打印堆状态，方便诊断 ESP_ERR_NO_MEM */
    ESP_LOGI(TAG, "Free heap: %lu bytes, min free: %lu bytes",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size());

    /* 1. 国内 CDN URL（固定文件名） */
    const char *cn_url = OTA_CN_URL;

    /* 2. 国内 CDN 下载 + 重试 */
    char info[128] = {0};
    esp_err_t err = ESP_FAIL;

    esp_rmaker_ota_report_status(ota_handle, OTA_STATUS_IN_PROGRESS, "Downloading firmware from CN CDN");
    for (int attempt = 1; attempt <= OTA_CN_MAX_ATTEMPTS; attempt++) {
        ESP_LOGW(TAG, "CN CDN OTA attempt %d/%d", attempt, OTA_CN_MAX_ATTEMPTS);
        err = ota_download_from_cn_cdn(cn_url, info, sizeof(info));
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGE(TAG, "CN CDN attempt %d failed: %s", attempt, info);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (err == ESP_OK) {
        /* 成功 → 上报成功，主动重启进新固件。
         * 注意：自定义回调自己写镜像，不会走 SDK 默认工作流的
         * reboot_pending 保护 + 重启事件，必须在这里自己重启。 */
        esp_rmaker_ota_report_status(ota_handle, OTA_STATUS_SUCCESS, "OTA via CN CDN finished");
        ESP_LOGI(TAG, "CN CDN OTA successful, rebooting into new firmware in 3s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
        return ESP_OK;  /* 不会走到这里 */
    }

    /* 3. 国内链路失败 → 回退 AWS 默认链路兜底 */
    ESP_LOGW(TAG, "CN CDN failed after %d attempts, falling back to RainMaker default (AWS)", OTA_CN_MAX_ATTEMPTS);
    esp_rmaker_ota_report_status(ota_handle, OTA_STATUS_IN_PROGRESS, "Falling back to default HTTPS OTA");
    err = esp_rmaker_ota_default_cb(ota_handle, ota_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RainMaker default OTA also failed: %s", esp_err_to_name(err));
        esp_rmaker_ota_report_status(ota_handle, OTA_STATUS_FAILED, "OTA failed (CN CDN and AWS)");
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