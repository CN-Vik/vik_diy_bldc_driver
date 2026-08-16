#ifndef MOTOR_CFG_PWM_H
#define MOTOR_CFG_PWM_H

#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"

/*
 * 这三个 GPIO 对应你的硬件 M0_IN1 / M0_IN2 / M0_IN3。
 * 按你的 PCB 实际连接修改。
 */
#define M0_IN1_GPIO    32
#define M0_IN2_GPIO    33
#define M0_IN3_GPIO    25

/*
    平衡车建议(10~20KHZ)
 * 新手先设置10kHz PWM 载波。
 */
#define M0_PWM_FREQ_HZ              (20 *(1000))
#define M0_PWM_TASK_T               ((1.0f/M0_PWM_FREQ_HZ)*(1000000.0f))
#define M0_PWM_T_S                  ((1.0f/M0_PWM_FREQ_HZ))

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
#define M0_PWM_PERIOD_TICKS    ((M0_PWM_RES_HZ / (M0_PWM_FREQ_HZ))) // 1000
#define M0_PWM_MAX_CMPV         ((M0_PWM_RES_HZ / (M0_PWM_FREQ_HZ))/2) /*中心对齐模式下，最大计数值是周期值的一半*/

// 死区时间：1μs = 10个tick（10MHz分辨率，1tick=0.1μs）
#define M0_DEAD_TIME_TICKS  10

/*
 * FD6287 是 bootstrap 高边驱动。
 * 初期不建议 pwm_duty 到 0% 或 100%。
 */
#define M0_DUTY_MIN                0.001f
#define M0_DUTY_MAX                0.999f


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

int64_t get_motor_pwm_isr_time_stamp(void);
esp_err_t esp32_mcpwm_init(void);
esp_err_t motor_set_pwm_duty(float duty_u, float duty_v, float duty_w);

#endif
