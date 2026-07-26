/**
 * @file app_main.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "motor_cfg_pwm.h"
#include "app_rtos_config.h"
#include "app_rtos_resource.h"
#include "as5600.h"
#include "bsp_cfg.h"
#include "motor_current.h"
#include "motor_power.h"
#include "vik_foc.h"
#include "vik_foc_pid.h"
#include "motor_angle_acqu.h"
#include "foc_task.h"

#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "vik_foc_example_main";

extern void gptimer_creat_main(void);


void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC float version");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 分区异常，先擦除整个nvs分区
        ESP_ERROR_CHECK(nvs_flash_erase());
        // ⭐擦除完成【必须重新初始化】
        err = nvs_flash_init();
        ESP_LOGI(TAG, "NVS_FLASH_Erase_All_data! \r\n");
    }
    ESP_ERROR_CHECK(err);

    /*1.rtos系统资源初始化*/
    app_rtos_resource_init();

    foc_task_creat();

    gptimer_creat_main();


}
