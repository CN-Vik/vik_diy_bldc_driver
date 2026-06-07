/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 说明：
 * 1. 这个文件基于 ESP-IDF 的 FOC/SVPWM 示例改写。
 * 2. 原示例使用 IQmath 定点数学库，例如 _IQ()、_IQmpy()、_IQdiv2()、_IQtoF()。
 * 3. 这里已经全部替换成 float 浮点计算，方便新手理解 FOC 的数学过程。
 * 4. 这里仍然保留 esp_svpwm 里的“逆变器/MCPWM 输出封装”，因为它负责配置 MCPWM、互补 PWM、死区等硬件输出。
 * 5. 这里不再调用 foc/esp_foc.h 里面的定点 FOC 计算函数，而是自己写 float 版本的 Park 反变换、Clarke 反变换和 SVPWM 零序注入。
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

static const char *TAG = "example_foc_float";



extern void vfoc_init(void);
extern void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);
// extern void gptimer_creat_main(void);
extern void motor_encoder_init(void);



void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC float version");

    /*
     * 创建一个计数信号量。
     * 作用：让 MCPWM 定时器中断和主循环同步。
     * 中断每来一次，给一次信号量；主循环拿到信号量后，计算一次新的 PWM 占空比。
     */
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);
    if (update_semaphore == NULL) {
        ESP_LOGE(TAG, "Create update semaphore failed");
        return;
    }

    /*电机编码器初始化获取机械角度*/
    motor_encoder_init();
    
    // gptimer_creat_main();/*创建定时器*/

    /*
     * 1. 初始化 FOC 参数。
     * 里面设置 pole_pairs = 7。
     */
    vfoc_init();

    /*
     * 2. 初始化 ESP32PWM(MCPWM)。
     * 输出到 M0_IN1 / M0_IN2 / M0_IN3。
     */
    m0_fd6287_mcpwm_init();

    /*
     * 3. 启动 GPTimer 周期控制。
     */
    m0_fd6287_foc_start();

}
