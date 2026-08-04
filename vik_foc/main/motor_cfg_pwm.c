/**
 * @file m0_fd6287_pwm.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "motor_cfg_pwm.h"
#include "vik_foc.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vik_foc_pid.h"
#include <math.h>
#include "esp_timer.h"
#include "app_rtos_resource.h"
#include "motor_power.h"
#include "app_rtos_config.h"
#include "foc_task.h"


static const char *TAG = "MOTOR_CFG_PWM";


/*!< 静态全局变量：电机M0的MCPWM控制器实例，初始化为0（未初始化硬件） */
static m0_mcpwm_t s_m0_pwm = {0};

int64_t pwm_time_stamp = 0; /*PWM ISR 时间戳,单位:us*/




static float m0_limit_float(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }

    if (value < min)
    {
        return min;
    }

    return value;
}

/**
 * @brief 20KHZ PWM分频
 * 
 * @param timer 
 * @param edata 
 * @param user_ctx 
 * @return true 
 * @return false 
 */
static bool IRAM_ATTR mcpwm_timer_isr ( mcpwm_timer_handle_t timer,
                                               const mcpwm_timer_event_data_t *edata,
                                               void *user_ctx)
{
    BaseType_t hp = pdFALSE;

    if (foc_task_handle)
    {
        vTaskNotifyGiveFromISR(
            foc_task_handle,
            &hp
        );
        // pwm_time_stamp = esp_timer_get_time();
    }

    return hp == pdTRUE;
}



static uint32_t m0_duty_to_compare(float pwm_duty)
{
    pwm_duty = m0_limit_float(pwm_duty, M0_DUTY_MIN, M0_DUTY_MAX);

    return (uint32_t)(pwm_duty * (float)M0_PWM_PERIOD_TICKS);
}



esp_err_t motor_set_pwm_duty(float duty_u, float duty_v, float duty_w)
{
    uint32_t cmp_u;
    uint32_t cmp_v;
    uint32_t cmp_w;

    if ((s_m0_pwm.cmp[0] == NULL) ||
        (s_m0_pwm.cmp[1] == NULL) ||
        (s_m0_pwm.cmp[2] == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    #if 0
        static uint32_t cnt = 0;

        if ((cnt++)>=50)
        {
            cnt = 0;
            ESP_LOGI(
                TAG,
                "[motor_set_pwm_duty]: %.3f, %.3f, %.3f\r\n",
                duty_u,
                duty_v,
                duty_w
            );
        }
    #endif

    cmp_u = m0_duty_to_compare(duty_u);
    cmp_v = m0_duty_to_compare(duty_v);
    cmp_w = m0_duty_to_compare(duty_w);

    #if 0
        static uint32_t cnt = 0;

        if ((cnt++)>=50)
        {
            cnt = 0;
            ESP_LOGI(
                TAG,
                "[motor_set_pwm_duty]: %lu, %lu, %lu\r\n",
                cmp_u,
                cmp_v,
                cmp_w
            );
        }
    #endif
    mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[0], cmp_u);
    mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[1], cmp_v);
    mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[2], cmp_w);

    return ESP_OK;
}



esp_err_t esp32_mcpwm_init(void)
{
    int i;
    const int pwm_gpio[3] = {
        M0_IN1_GPIO,
        M0_IN2_GPIO,
        M0_IN3_GPIO,
    };

    /*
     * 1. 创建 MCPWM timer。
     * 三相 PWM 共用同一个 timer。
     */
    mcpwm_timer_config_t mcptimer_basic_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = M0_PWM_RES_HZ,/*10MHZ*/

        /*
        计数分辨率：10MHz，也就是 0.1us 一跳
        周期计数：1000 tick
        PWM周期：1000 × 0.1us = 100us
        PWM频率：10kHz
        compare范围：大约 0 ~ 1000
        占空比：compare / 1000
        */
        .period_ticks = M0_PWM_PERIOD_TICKS,/*1000*/
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,/*单向 向上递增模式*/
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&mcptimer_basic_cfg, &s_m0_pwm.timer));

    /*单项递增计数模式*/
    mcpwm_timer_event_callbacks_t mctimer_cbs = {
        .on_empty = mcpwm_timer_isr,/*单向PWM，每个PWM周期开始触发*/
    };

    ESP_ERROR_CHECK(
        mcpwm_timer_register_event_callbacks(
            s_m0_pwm.timer,
            &mctimer_cbs,
            NULL
        )
    );

    /*
     * 2. 创建 3 个 operator。
     * 每个 operator 对应一相。
     */
    for (i = 0; i < 3; i++)
    {
        mcpwm_operator_config_t oper_config = {
            .group_id = 0,
        };

        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &s_m0_pwm.oper[i]));

        /*
         * 关键：三个 operator 连接到同一个 timer。
         * 这样 U/V/W 三相同步。
         */
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_m0_pwm.oper[i],
                                                     s_m0_pwm.timer));
    }

    /*
     * 3. 每相创建 comparator 和 generator。
     */
    for (i = 0; i < 3; i++)
    {
        mcpwm_comparator_config_t cmp_config = {
            /*
             * update_cmp_on_tez = true：
             * compare 值在 timer 到 0 时更新。
             * 这样 pwm_duty 更新更平滑，不会半个周期突然改变。
             */
            .flags.update_cmp_on_tez = true,
        };

        ESP_ERROR_CHECK(mcpwm_new_comparator(s_m0_pwm.oper[i],
                                             &cmp_config,
                                             &s_m0_pwm.cmp[i]));

        mcpwm_generator_config_t gen_config = {
            .gen_gpio_num = pwm_gpio[i],
        };

        ESP_ERROR_CHECK(mcpwm_new_generator(s_m0_pwm.oper[i],
                                            &gen_config,
                                            &s_m0_pwm.gen[i]
                        )
        );

        /* Timer计数到0，输出High */
        ESP_ERROR_CHECK(
            mcpwm_generator_set_action_on_timer_event(
                s_m0_pwm.gen[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY,
                    MCPWM_GEN_ACTION_HIGH
                )
            )
        );

        /* Timer计数到Compare，输出Low */
        ESP_ERROR_CHECK(
            mcpwm_generator_set_action_on_compare_event(
                s_m0_pwm.gen[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    s_m0_pwm.cmp[i],
                    MCPWM_GEN_ACTION_LOW
                )
            )
        );

        ESP_ERROR_CHECK(
            mcpwm_comparator_set_compare_value(
                s_m0_pwm.cmp[i],
                M0_PWM_PERIOD_TICKS-1
            )
        );


    }

    /*
     * 5. 启动 MCPWM timer。
     */
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_m0_pwm.timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_m0_pwm.timer,
                                           MCPWM_TIMER_START_NO_STOP
                    )
    );

    ESP_LOGI(TAG,
             "M0 MCPWM init done, freq=%dHz, period_ticks=%d\r\n",
             M0_PWM_FREQ_HZ,
             M0_PWM_PERIOD_TICKS
    );

    return ESP_OK;
}



int64_t get_motor_pwm_isr_time_stamp(void)
{
    return pwm_time_stamp;
}
