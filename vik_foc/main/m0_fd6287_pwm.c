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
 * 20kHz 下 period_ticks = 500。
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

/*
 * FOC 控制周期。
 * 先用 1kHz，低速开环测试够用。
 */
#define M0_FOC_CTRL_FREQ_HZ        1000

/*dt_s = 0.001 秒 = 1ms
这个 dt_s 会传给你的开环角度积分函数。*/
#define M0_FOC_DT_S                (1.0f / M0_FOC_CTRL_FREQ_HZ)

/*
 * GPTimer 分辨率 1MHz。
 * 1 tick = 1us。
 * 1kHz 控制周期就是 1000us。
 */
#define M0_CTRL_TIMER_RES_HZ       1000000
#define M0_CTRL_ALARM_COUNT        (M0_CTRL_TIMER_RES_HZ / M0_FOC_CTRL_FREQ_HZ) /*1KHZ,1ms*/

/*
 * 开环测试参数。
 * 先低速、低电压。
 *  第一次空载测试	10 ~ 30 rpm	     0.3 ~ 0.8V	最安全，看能不能轻微转动
    初步启动	    30 ~ 100 rpm	0.8 ~ 1.5V	观察方向、抖动、电流
    低速稳定	    100 ~ 300 rpm	1.0 ~ 2.0V	适合先验证算法
    中速测试	    300 ~ 800 rpm	2.0 ~ 3.0V	建议 FOC 控制频率提高到 5kHz
    高速测试	    800 ~ 1300 rpm	3.0 ~ 4.0V	建议 FOC 控制频率提高到 10kHz，必须斜坡启动
 */
#define M0_TEST_RPM                30.0f // (r/min）
#define M0_TEST_UQ                 0.8f /*3.8f = 3.8V*/
#define M0_TEST_UD                 0.0f /* */
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

/*
 * 栅极驱动芯片使能脚。
 * 很多三相驱动板会有一个 EN 引脚，用来总开关 MOSFET 驱动输出。
 * enable = true  时，允许驱动 MOSFET。
 * enable = false 时，关闭驱动输出，电机不再受控输出。
 */
#define VBUS_EN_GPIO         12

/**Uq_max ≈ 12 / 1.732 ≈ 6.9V */
#define UQ_LIMIT            3.2f      // 初期限制 ±1.2V
#define POS_DEADBAND_DEG    0.5f      // 小误差死区
#define SPEED_DEADBAND_RPM  3.0f

#define SPEED_I_OUT_LIMIT  1.5f


// 360° 环形期望值限幅（自动绕回）
#define LIMIT_EXP_MECH_360(exp_mech)                \
do {                                            \
    while ((exp_mech) >= 360.0f) (exp_mech) -= 360.0f; \
    while ((exp_mech) < 0.0f)    (exp_mech) += 360.0f; \
} while(0)


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

/*!< 静态全局变量：电机M0控制定时器句柄（GPTimer软件定时器）
 *   作用：提供精准定时中断，用于FOC控制算法的固定周期调用（如电流环/速度环）
 *   初始值NULL表示未创建定时器 */
static gptimer_handle_t s_m0_ctrl_timer = NULL;

/*!< 静态全局变量：电机M0 FOC控制任务句柄
 *   作用：指向FOC电机控制任务（FreeRTOS任务），用于任务挂起、恢复、删除等管理
 *   初始值NULL表示未创建任务 */
static TaskHandle_t s_m0_foc_task_handle = NULL;




/*
 * 初始化栅极驱动芯片 EN 引脚。
 */
void vbus_en_io_init(void)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << VBUS_EN_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
}

/*
 * 打开或者关闭三相驱动输出。
 */
void vbus_enable(bool enable)
{
    ESP_LOGI(TAG, "%s MOSFET gate", enable ? "Enable" : "Disable");
    gpio_set_level(VBUS_EN_GPIO, enable);
}



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


static bool gptimer_1ms_cb(gptimer_handle_t timer,
                            const gptimer_alarm_event_data_t *edata,
                            void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (s_m0_foc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_m0_foc_task_handle, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}


/**
 * @brief 斜坡限速函数
 *
 * @details
 * 这个函数的作用是：让 now 不要一下子跳到 target，
 * 而是每次最多只变化 max_step。
 *
 * 举例：
 * now = 0
 * target = 60
 * max_step = 1
 *
 * 每次调用结果：
 * 0 -> 1 -> 2 -> 3 -> ... -> 60
 *
 * 这样可以避免：
 * 1. 目标速度突然变化太大
 * 2. Uq突然变化太大
 * 3. 电机启动猛冲、超调、震动
 * 
 * 比如你 1ms 调用一次，想让目标速度每秒最多增加 80rpm，那每次最大步长就是：

RPM_RAMP_PER_S * dt_s = 80.0f * 0.001f = 0.08rpm

也就是目标速度会这样慢慢爬：

0 -> 0.08 -> 0.16 -> 0.24 -> ... -> 60rpm
 *
 * @param now       当前值，比如当前目标速度、当前Uq
 * @param target    最终想达到的目标值
 * @param max_step  本次调用允许变化的最大步长，必须是正数
 *
 * @return float    限速后的新值
 */
static float ramp_float(float now, float target, float max_step)
{
    /*
     * 计算目标值和当前值之间的差值。
     *
     * diff > 0：说明目标值比当前值大，需要往上增加。
     * diff < 0：说明目标值比当前值小，需要往下降低。
     */
    float diff = target - now;

    if (diff > max_step)
    {
        /*
         * 如果差值大于最大允许变化量，
         * 说明这次不能一下子加这么多，只允许最多增加 max_step。
         *
         * 例如：
         * now = 0，target = 60，max_step = 1
         * diff = 60
         * 实际本次只允许 +1
         */
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        /*
         * 如果差值小于 -max_step，
         * 说明目标值比当前值小很多，
         * 这次不能一下子减太多，只允许最多减少 max_step。
         *
         * 例如：
         * now = 60，target = 0，max_step = 1
         * diff = -60
         * 实际本次只允许 -1
         */
        diff = -max_step;
    }

    /*
     * 当前值加上被限制后的变化量。
     *
     * 如果 target 离 now 很远：
     *      每次只靠近 max_step。
     *
     * 如果 target 离 now 很近：
     *      直接到达 target，不会来回震荡。
     */
    return now + diff;
}

static float limit_float(float x, float min, float max)
{
    if (x > max)
    {
        return max;
    }

    if (x < min)
    {
        return min;
    }

    return x;
}

/**
 * @brief FOC位置控制角度误差计算
 *        输入当前角度值，和期望角度值
 * 
 * 电机旋转方向：顺时针+正值，逆时针-负值
 * 
 * @param expct_deg 期望角度值
 * @param current_deg 当前角度值
 * @return float 输出的带旋转方向的误差角度值，eg: -45(逆时针旋转四十五度), 90(顺时针旋转90度)
 */
static float angle_error_deg(float expct_deg, float current_deg)
{
    /*误差值 = 期望值-当前值*/
    float err = expct_deg - current_deg;

    while (err > 180.0f)
    {
        err -= 360.0f;
    }

    while (err < -180.0f)
    {
        err += 360.0f;
    }

    return err;
}

/**
 * @brief 函数执行周期-1ms
 * 
 * @param arg 
 */
static void m0_foc_control_task(void *arg)
{
    pwm_duty_t pwm_duty;

    float uq = 0.0f;
    float exp_angle = 10.0f;/*期望角度值*/
    float now_angle = 0.0f;
    float err_angle = 0.0f;
    float last_err_angle = 0.0f;
    
    float exp_motor_rpm = 89.0f;/*期望电机转速值*/
    float now_motor_rpm = 0.0f;
    float err_motor_rpm = 0.0f;/*rpm:(r/min)*/
    float last_err_motor_rpm = 0.0f;/*上次转速误差*/

    float kp = 0.02f;
    float kp_out = 0.0f;/*比例控制输出*/
        
    float ki = 0.06f;/*0.001f ~ 0.005f;*/
    float ki_err_sum = 0.0f;/*微分控制参数*/
    float ki_out = 0.0f;/*积分控制输出*/
    
    float kd = 0.0f;/*0.001f ~ 0.005f;*/
    float kd_err_parm = 0.0f;/*微分控制参数*/
    float kd_out = 0.0f;/*微分控制输出*/

    
    uint32_t log_cnt = 0;

    uint64_t stamp_time_us = 0;
    uint64_t last_stamp_time_us = 0;


    static uint64_t time_ms = 0;
    static uint64_t time_s = 0;
    static uint64_t time_min = 0;

    /*位置式PID:(dt)
    u[k] = Kp * e[k] + Ki * sum(e[0..k]) * dt + Kd * (e[k] - e[k-1]) / dt

    // 增量式PID（FOC中更常用）
    delta_u[k] = Kp * (e[k] - e[k-1]) + Ki * e[k] * dt + Kd * (e[k] - 2*e[k-1] + e[k-2]) / dt
    u[k] = u[k-1] + delta_u[k]
    */
    float dt_s = 0.0f;/*pid计算时间间隔，单位s*/

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (stamp_time_us)
        {/*第一次不计算时间,第一次的last_stamp_time_us=0，计算出的时间有问题的*/
            stamp_time_us = esp_timer_get_time();/*单位us*/
            dt_s = (float)((stamp_time_us - last_stamp_time_us)/(1000000.0f));
        }else{
            
            stamp_time_us = esp_timer_get_time();/*单位us*/
            dt_s = (float)(1.0f);
        }

        time_ms++;
        // exp_angle++;

        // 60ms = 1秒计数
        if (time_ms >= 60)
        {
            time_ms = 0;  // 清零，不是取余！
            time_s++;

            // 60秒 = 1分钟
            if (time_s >= 60)
            {
                time_s = 0;
                time_min++;
            }
        }
        
        // LIMIT_EXP_MECH_360(exp_angle);

        // now_angle = get_vfoc_theta_m_deg();/* 获取当前机械角度值 */
        // err_angle = angle_error_deg( exp_angle , now_angle );/*本次误差值*/
        now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/
        err_motor_rpm = exp_motor_rpm - now_motor_rpm;/*本次误差值*/
        /*速度死区，这样速度在 97 ~ 103 rpm 之间就不要来回调整 Uq 了，声音会安静很多。*/
        /*
        * 4. 死区处理
        */
        if (fabsf(err_motor_rpm) < SPEED_DEADBAND_RPM)
        {
            err_motor_rpm = 0.0f;

            /*
            * 误差很小时，积分慢慢释放一点，防止来回顶。
            */
            ki_err_sum *= 0.98f;
        }
        else
        {
            /*
            * 5. 积分
            */
            ki_err_sum += (err_motor_rpm * dt_s);
        }

        kp_out = (kp * err_motor_rpm);

        ki_out = (ki * ki_err_sum);
        /*积分限幅*/
        ki_out = limit_float(ki_out, -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);
        /* 反推积分，防止 ki_err_sum 内部继续无限变大 */
        if (ki > 0.000001f)
        {
            ki_err_sum = ki_out / ki;
        }

        /*kd微分参数:本次误差值-上一次误差值*/
        // kd_err_parm = err_angle - last_err_angle;
        kd_err_parm = err_motor_rpm - last_err_motor_rpm;
        kd_out = (kd * kd_err_parm / dt_s);
        

        // uq = (kp * err_angle) + (kd * kd_err_parm);
        uq = kp_out + ki_out + kd_out;
        uq *= -1;
        // ESP_LOGI(TAG,
        //         " uq: %.2f\r\n",
        //         uq
        // );
        /* 限制 Uq 最大输出，防止上电/大误差时力矩过猛 */
        uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
        
        if (++log_cnt >= 100)
        {
            log_cnt = 0;

            ESP_LOGI(TAG,
                    "uq: %.2f,%.2f,%.2f,%.2f,%.2f,%.5f\r\n",
                    uq,
                    kp,
                    exp_motor_rpm,/*期望转速*/
                    now_motor_rpm,/*现在转速*/
                    err_motor_rpm /*转速误差*/,
                    dt_s
            );
        }

        /*
         * 如果发现电机远离目标，把 uq 改成 -uq，
         * 或者修正编码器方向/电角度方向。
         */
        vfoc_set_svpwm(uq,
                       M0_TEST_UD,
                       MOTOR_DRV_VBUS);

        pwm_duty = vfoc_get_pwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);

        last_stamp_time_us = stamp_time_us; 

        last_err_motor_rpm = err_motor_rpm;/*更新上次误差值*/
    }
}
esp_err_t m0_fd6287_foc_start(void)
{
    /*
    * 1. 创建 FOC 控制任务。
    *    创建一个固定运行在 CPU 核心1上的高优先级电机控制任务
    */
    xTaskCreatePinnedToCore(
        m0_foc_control_task,        /* 任务函数指针：FOC 电机控制主循环函数（无限循环） */
        "m0_foc_task",              /* 任务名称：调试时方便识别，无实际功能 */
        4096*2,                      /* 任务堆栈大小：分配 4096 字节栈空间（ESP32 单位是字，不是字节） */
        NULL,                       /* 任务入参：不需要传递参数，填 NULL */
        20,                         /* 任务优先级：20级（最高优先级，保证电机控制实时性） */
        &s_m0_foc_task_handle,      /* 任务句柄：输出参数，保存创建的任务句柄，用于后续任务管理 */
        1                           /* 绑定CPU核心：指定任务**只运行在 CPU 1** 上 */
    );

    /*
     * 2. 创建 GPTimer。
     */
    gptimer_config_t gptimer_basic_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,

        /* 定时器频率
         *
         * 1000000 Hz = 1 MHz
         *
         * 意味着定时器每秒计数 1000000 次。
         * 所以：
         * 1 tick = 1 / 1000000 秒 = 1 us
         *
         * 这也是后面 alarm_count = 1000000 表示 1 秒的原因。
         */
        .resolution_hz = M0_CTRL_TIMER_RES_HZ,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_basic_cfg, &s_m0_ctrl_timer));

    /*
     * 3. 设置周期 alarm。
     *
     * 1MHz 分辨率下：
     * 1000 ticks = 1000us = 1ms
     */
    gptimer_alarm_config_t gptimer_cfg_alarm = {
        .reload_count = 0,
        .alarm_count = M0_CTRL_ALARM_COUNT,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_m0_ctrl_timer, &gptimer_cfg_alarm));

    /*
     * 4. 注册 alarm 回调。
     */
    gptimer_event_callbacks_t gptimer_cb = {
        .on_alarm = gptimer_1ms_cb,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_m0_ctrl_timer,
                                                     &gptimer_cb,
                                                     NULL
                    )
    );

    /*
     * 5. 启动 GPTimer。
     */
    ESP_ERROR_CHECK(gptimer_enable(s_m0_ctrl_timer));
    ESP_ERROR_CHECK(gptimer_start(s_m0_ctrl_timer));

    vbus_en_io_init();
    vbus_enable(true);/*打开VBUS*/

    ESP_LOGI(TAG,
             "M0 FOC started, ctrl_freq=%dHz",
             M0_FOC_CTRL_FREQ_HZ
    );

    return ESP_OK;
}


