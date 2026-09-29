/**
 * @file ota_safety.c
 * @brief OTA 升级前的电机安全停机
 */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_power.h"
#include "foc_task.h"
#include "motor_cfg_pwm.h"

static const char *TAG = "OTA_SAFE";

void app_ota_prepare_shutdown(void)
{
    ESP_LOGW(TAG, "=== OTA: shutting down motor for safe upgrade ===");

    /* 1. 关闭 MOS 电源 —— 硬件断开，电机立即失电（最安全） */
    motor_power_enable(false);

    /* 2. 挂起 FOC 任务 —— 停止最高优先级抢占，释放 CPU 给 OTA 任务 */
    if (foc_task_handle != NULL) {
        vTaskSuspend(foc_task_handle);
        ESP_LOGI(TAG, "FOC task suspended");
    } else {
        ESP_LOGW(TAG, "FOC task handle is NULL, skip suspend");
    }

    /* 3. PWM 三相输出零电压矢量（50% 占空比），电机无有效驱动电压 */
    motor_set_pwm_duty(0.5f, 0.5f, 0.5f);

    ESP_LOGW(TAG, "=== Motor shutdown done, safe to OTA ===");
}
