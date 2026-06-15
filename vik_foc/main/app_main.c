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
#include "m0_fd6287_pwm.h"
#include "app_rtos_config.h"
#include "app_rtos_resource.h"
#include "as5600.h"
#include "bsp_cfg.h"
#include "motor_current.h"
#include "motor_power.h"
#include "vik_foc.h"
#include "vik_foc_pid.h"
#include "motor_angle_acqu.h"


static const char *TAG = "vik_foc_example_main";





void motor_get_current_main(void);


void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC float version");

    /*1.rtos系统资源初始化*/
    app_rtos_resource_init();

    /* 2. 初始化 MOS enable GPIO,默认必须关闭 MOS*/
    motor_power_init();

    /*3.电机编码器初始化获取机械角度*/
    motor_encoder_init();

    /*4.获取电机电流*/
    motor_get_current_main();
    
    // gptimer_creat_main();/*创建定时器*/

    /*
     * 5. 初始化 FOC 参数。
     * 里面设置 pole_pairs = 7。
     */
    vfoc_init();

    /*
     * 6. 初始化 ESP32PWM(MCPWM)。
     * 输出到 M0_IN1 / M0_IN2 / M0_IN3。
     */
    m0_fd6287_mcpwm_init();

    /*
     * 7. 启动 GPTimer 周期控制。
     */
    motor_set_pwm_init();

}
