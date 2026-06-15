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
#include "m0_fd6287_pwm.h"
#include "vik_foc.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "vik_foc_pid.h"
#include <math.h>
#include "esp_timer.h"
#include "app_rtos_resource.h"
#include "motor_power.h"
#include "app_rtos_config.h"


static const char *TAG = "M0_FD6287";

/*
 * 这三个 GPIO 对应你的硬件 M0_IN1 / M0_IN2 / M0_IN3。
 * 按你的 PCB 实际连接修改。
 */
#define M0_IN1_GPIO    32
#define M0_IN2_GPIO    33
#define M0_IN3_GPIO    25

/*
 * 40kHz PWM 载波。
 */
#define M0_PWM_FREQ_HZ             (40 *(1000))

/*
 * MCPWM 分辨率 10MHz。

 * 20kHz == period_ticks = 500。
 * 
 * MCPWM timer 计数频率 = M0_PWM_RES_HZ
目标 PWM 频率 = M0_PWM_FREQ_HZ
一个 PWM 周期需要多少个 tick = 计数频率 / PWM频率

MCPWM timer 每秒数 10000000 次
也就是 1 tick = 0.1us

一个 PWM 周期数 500 个 tick

PWM周期 = 500 × 0.1us = 50us
PWM频率 = 1 / 50us = 20kHz

 */
#define M0_PWM_RES_HZ              10000000

// 一个 PWM 周期有 500 个计数 tick,
/*
计数分辨率：10MHz，也就是 0.1us 一跳
周期计数：500 tick
PWM周期：500 × 0.1us = 50us
PWM频率：20kHz
compare范围：大约 0 ~ 500
占空比：compare / 500
*/
#define M0_PWM_PERIOD_TICKS        (M0_PWM_RES_HZ / M0_PWM_FREQ_HZ) /* 2000 ticks，200us，80kHz */



#define MOTOR_DRV_VBUS             12.0f  /* 12V */


/*
 * FD6287 是 bootstrap 高边驱动。
 * 初期不建议 pwm_duty 到 0% 或 100%。
 */
#define M0_DUTY_MIN                0.02f
#define M0_DUTY_MAX                0.98f


#define GPTIMER_US(us)          (us)
#define GPTIMER_MS(ms)      (ms * GPTIMER_US(1000) )
#define GPTIMER_S(s)        (s * GPTIMER_MS(1000)  )





/**
 * @brief 电机M0专用MCPWM控制器结构体
 * @details ESP32 MCPWM外设包含：定时器、操作器、比较器、发生器
 *          这里封装3路PWM输出（对应FOC三相U/V/W）的所有硬件句柄
 */
typedef struct
{
    /*!< MCPWM定时器句柄：负责生成PWM基准时钟、周期计数（20kHz/500tick） */
    mcpwm_timer_handle_t timer;       
    
    /*!< MCPWM操作器句柄[3路]：每路对应一相输出，控制PWM工作模式 */
    mcpwm_oper_handle_t  oper[3];     
    
    /*!< MCPWM比较器句柄[3路]：设置比较值，决定PWM占空比（决定相电压） */
    mcpwm_cmpr_handle_t  cmp[3];      
    
    /*!< MCPWM发生器句柄[3路]：最终生成PWM波形输出到GPIO引脚 */
    mcpwm_gen_handle_t   gen[3];      
} m0_mcpwm_t;

/*!< 静态全局变量：电机M0的MCPWM控制器实例，初始化为0（未初始化硬件） */
static m0_mcpwm_t s_m0_pwm = {0};


/*!< 静态全局变量：电机M0 FOC控制任务句柄
 *   作用：指向FOC电机控制任务（FreeRTOS任务），用于任务挂起、恢复、删除等管理
 *   初始值NULL表示未创建任务 */
static TaskHandle_t s_m0_foc_task_handle = NULL;




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


static bool IRAM_ATTR mcpwm_timer_on_empty_cb( mcpwm_timer_handle_t timer,
                                               const mcpwm_timer_event_data_t *edata,
                                               void *user_ctx)
{
    static uint8_t div = 0;

    BaseType_t hp = pdFALSE;

    div++;

    if(div >= 4)
    {
        div = 0;

        vTaskNotifyGiveFromISR(
            s_m0_foc_task_handle,
            &hp);
    }

    return hp == pdTRUE;
}

static uint32_t m0_duty_to_compare(float pwm_duty)
{
    pwm_duty = m0_limit_float(pwm_duty, M0_DUTY_MIN, M0_DUTY_MAX);

    return (uint32_t)(pwm_duty * (float)M0_PWM_PERIOD_TICKS);
}

esp_err_t m0_fd6287_set_duty(float duty_u, float duty_v, float duty_w)
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

    cmp_u = m0_duty_to_compare(duty_u);
    cmp_v = m0_duty_to_compare(duty_v);
    cmp_w = m0_duty_to_compare(duty_w);

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[0], cmp_u));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[1], cmp_v));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_m0_pwm.cmp[2], cmp_w));

    return ESP_OK;
}

esp_err_t m0_fd6287_mcpwm_init(void)
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
        周期计数：500 tick
        PWM周期：500 × 0.1us = 50us
        PWM频率：20kHz
        compare范围：大约 0 ~ 500
        占空比：compare / 500
        */
        .period_ticks = M0_PWM_PERIOD_TICKS,/*500*/
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&mcptimer_basic_cfg, &s_m0_pwm.timer));

    mcpwm_timer_event_callbacks_t mctimer_cbs = {
        .on_empty = mcpwm_timer_on_empty_cb,
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

        /*
         * 4. 配置 PWM 输出动作。
         *
         * timer EMPTY，也就是计数到 0：输出 HIGH
         * timer 计数到 compare：输出 LOW
         *
         * 所以：
         * compare 越大，占空比越大。
         */
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                            s_m0_pwm.gen[i],
                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                        MCPWM_TIMER_EVENT_EMPTY,
                                                        MCPWM_GEN_ACTION_HIGH)
                        )
        );

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                                s_m0_pwm.gen[i],
                                MCPWM_GEN_COMPARE_EVENT_ACTION(
                                    MCPWM_TIMER_DIRECTION_UP,
                                    s_m0_pwm.cmp[i],
                                    MCPWM_GEN_ACTION_LOW
                                )
                        )
        );

        /*
         * 初始 50%。
         * 注意：这只是初始化阶段。
         * 真正运行后由 FOC 算法不断更新 duty。
         */
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(
                            s_m0_pwm.cmp[i],
                            M0_PWM_PERIOD_TICKS / 2
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




/**
 * @brief M0 FOC控制任务
 *
 * 当前执行周期：
 *      1ms / 1kHz
 *
 * 当前用途：
 *      调试阶段可以用于低速电流环/力矩环验证。
 *
 * 频率建议：
 *
 *      PWM频率：
 *          40kHz
 *
 *      ADC采样率：
 *          总采样率 80kHz
 *          4路轮询，平均每路约 20kHz
 *
 *      电流环：
 *          当前 1kHz，可以先调通流程
 *          后续建议提高到 5kHz
 *          如果性能允许，再考虑 10kHz
 *
 *      速度环：
 *          不需要几kHz
 *          通常 200Hz ~ 1kHz 即可
 *
 *      位置环：
 *          不需要几kHz
 *          通常 50Hz ~ 500Hz 即可
 *
 *      角度采样：
 *          AS5600 I2C 读取建议 500Hz ~ 1kHz
 *          如果I2C提高到400kHz并且读取稳定，可以维持1kHz
 *
 * 注意：
 *      电流环频率应高于速度环和位置环。
 *      速度环/位置环不要和电流环同频硬跑，否则容易浪费CPU并引入噪声。
 * 
 * 
PWM频率：        20kHz ~ 40kHz

ADC采样：        每路 10kHz ~ 20kHz

电流环：         5kHz ~ 10kHz
                初期可以 1kHz 先调通

速度环：         200Hz ~ 1kHz

位置环：         50Hz ~ 500Hz

角度采样：       500Hz ~ 1kHz


MCPWM Timer
      │
      ▼
触发ADC采样
      │
      ▼
ADC DMA完成中断
      │
      ▼
vTaskNotifyGiveFromISR(FOC Task)
      │
      ▼
读取Ia、Ib
      │
      ▼
读取编码器角度
      │
      ▼
Clarke
      │
      ▼
Park
      │
      ▼
电流PID
      │
      ▼
SVPWM
      │
      ▼
更新Compare
 * 
 *
 * @param arg 任务参数，当前未使用
 */
static void motor_set_pwm_task(void *arg)
{
    pwm_duty_t pwm_duty;

    /*
    * 只调用一次电流环。
    * 不能写 vfoc_curent_loop().Uq / vfoc_curent_loop().Ud，
    * 那样会执行两次电流环。
    */
    park_parm_t vdq_cmd = {0};
    
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vdq_cmd = vfoc_curent_loop();

        /*
         * 如果发现电机远离目标，把 uq 改成 -uq，
         * 或者修正编码器方向/电角度方向。
         */
        vfoc_set_svpwm(
            vdq_cmd.Uq,
            vdq_cmd.Ud,
            MOTOR_DRV_VBUS
        );

        m0_fd6287_set_duty(
            vfoc_get_pwm_duty().duty_Ua,
            vfoc_get_pwm_duty().duty_Ub,
            vfoc_get_pwm_duty().duty_Uc
        );

    }
}


/**
 * @brief 
 * 
 * 整个工业FOC时序图
 *               PWM Timer

TEZ ------------------------------------ TEZ
│                                         │
│                                         │
│<--------- 一个PWM周期 ----------------->│
│
│  MOS导通
│
│      ↓
│
│  电流稳定
│
│      ↓
│
│  ADC Trigger（硬件）
│
│      ↓
│
│  ADC转换
│
│      ↓
│
│ DMA Done ISR
│      │
│      ▼
│ vTaskNotifyGive()
│      │
│      ▼
│ FOC Task
│      │
│      ├──Ia Ib
│      ├──Encoder
│      ├──Clark
│      ├──Park
│      ├──Id PID
│      ├──Iq PID
│      ├──InvPark
│      ├──SVPWM
│      └──Compare
│
└──────────────等待下一TEZ自动更新PWM──────────────


MCPWM (40kHz)
        │
        ▼
ADC Continuous DMA（20kHz）
        │
        ▼
ADC DMA Done Callback
        │
        ▼
vTaskNotifyGiveFromISR(foc_task)
        │
        ▼
FOC Task（唯一高优先级任务）
        │
        ├──读取DMA电流
        ├──读取Encoder
        ├──Clark
        ├──Park
        ├──Id/Iq PID
        ├──InvPark
        ├──SVPWM
        └──写Compare
 * 
 * @return esp_err_t 
 */
esp_err_t motor_set_pwm_init(void)
{
    /*
    * 1. 创建 FOC 控制任务。
    *    创建一个固定运行在 CPU 核心1上的高优先级电机控制任务
    */
    xTaskCreatePinnedToCore(
        motor_set_pwm_task,        /* 任务函数指针：FOC 电机控制主循环函数（无限循环） */
        "m0_set_pwm_task",              /* 任务名称：调试时方便识别，无实际功能 */
        MO_SET_PWM_TASK_STACK,  /* 任务堆栈大小：分配 4096 字节栈空间（ESP32 单位是字，不是字节） */
        NULL,                       /* 任务入参：不需要传递参数，填 NULL */
        MO_SET_PWM_TASK_PRIO,   /* 任务优先级：20级（最高优先级，保证电机控制实时性） */
        &s_m0_foc_task_handle,      /* 任务句柄：输出参数，保存创建的任务句柄，用于后续任务管理 */
        MO_SET_PWM_TASK_CORE    /* 绑定CPU核心：指定任务**只运行在 CPU 1** 上 */
    );

    /*开启电机MOS*/
    if ( !motor_power_is_enabled() )
    {/*没打开电机MOS 死等打开MOS*/

        if (g_app_event_group)
        {
            EventBits_t bits;
            /*
            * 第一步：
            * 等待电流零漂校准完成。
            *
            * 如果零漂没完成，这里会阻塞等待。
            */
            bits = xEventGroupWaitBits(
                g_app_event_group,
                APP_EVT_MOS_ENABLED,
                pdFALSE,                        /* 不清除事件位 */
                pdTRUE,                         /* 等待全部指定bit，这里只有一个bit */
                pdMS_TO_TICKS(5000) /*等待时间5s*/
            );
    
            if ((bits & APP_EVT_MOS_ENABLED) == 0)
            {
                ESP_LOGE(TAG, "等待电流零漂校准完成超时,禁止打开MOS");
                ESP_LOGE(TAG, "foc_start_MOS打开失败,禁止启动电机\r\n");
            }
        }
    }


    return ESP_OK;
}


