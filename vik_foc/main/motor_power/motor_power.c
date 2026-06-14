/**
 * @file motor_power.c
 * @author vik (ufo281@outlook.com)
 * @brief
 * @version 0.1
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "motor_power.h"
#include "app_rtos_resource.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "MOTOR_POWER";

static bool s_motor_mos_en = false;

void motor_power_init(void)
{
    gpio_config_t motor_mos_en_io_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_MOS_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&motor_mos_en_io_cfg));

    /*
     * 上电默认关闭 MOS。
     * 这个非常重要。
     */
    gpio_set_level(MOTOR_MOS_EN_GPIO, 0);
    s_motor_mos_en = false;

    /*
     * 清除 MOS 已使能事件位。
     */
    if (g_app_event_group != NULL)
    {
        xEventGroupClearBits(g_app_event_group, APP_EVT_MOS_ENABLED);
    }

    ESP_LOGI(TAG, "MOS enable GPIO初始化完成，默认关闭");
}

/**
 * @brief
 *
 * @param timeout_ms 等待打开时间
 * @return esp_err_t
 *
 */
esp_err_t motor_power_enable(bool en)
{
    // EventBits_t bits;

    if (g_app_event_group == NULL || g_mos_enable_mutex == NULL)
    {
        ESP_LOGE(TAG, "RTOS资源未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     *
     * 拿 MOS enable 互斥锁。
     *
     * 防止其他任务同时操作 MOS enable。
     */
    if (xSemaphoreTake(g_mos_enable_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    { /*上锁*/
        ESP_LOGE(TAG, "获取 MOS enable 互斥锁失败");
        return ESP_ERR_TIMEOUT;
    }

    switch (en)
    {
        case true:
        {
            xEventGroupSetBits(g_app_event_group, APP_EVT_MOS_ENABLED); /*设置Motor mos enable! */
            gpio_set_level(MOTOR_MOS_EN_GPIO, en);
            s_motor_mos_en = en;
            ESP_LOGI(TAG, "MOS enable 已打开");
            break;
        }
        case false:
        {
            /*
            * 关闭 MOS enable。
            */
            gpio_set_level(MOTOR_MOS_EN_GPIO, en);
            s_motor_mos_en = en;
            /*清除motor MOS使能位*/
            xEventGroupClearBits(g_app_event_group, APP_EVT_MOS_ENABLED);
            ESP_LOGW(TAG, "MOS enable 已关闭");
            break;
        }

        default:
        {
            ESP_LOGE(TAG, "motor_power_enable_param_error!\r\n");
            break;
        }
    }

    // /*
    //  * 第一步：
    //  * 等待电流零漂校准完成。
    //  *
    //  * 如果零漂没完成，这里会阻塞等待。
    //  */
    // bits = xEventGroupWaitBits(
    //     g_app_event_group,
    //     APP_EVT_CURRENT_ZERO_DONE,
    //     pdFALSE,                        /* 不清除事件位 */
    //     pdTRUE,                         /* 等待全部指定bit，这里只有一个bit */
    //     pdMS_TO_TICKS(timeout_ms) /*等待时间*/
    // );

    // if ((bits & APP_EVT_CURRENT_ZERO_DONE) == 0)
    // {
    //     ESP_LOGE(TAG, "等待电流零漂校准完成超时,禁止打开MOS");
    //     return ESP_ERR_TIMEOUT;
    // }

    xSemaphoreGive(g_mos_enable_mutex); /*解锁*/
    return ESP_OK;
}

bool motor_power_is_enabled(void)
{
    return s_motor_mos_en;
}