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



/**Uq_max ≈ 12 / 1.732 ≈ 6.9V */
#define UQ_LIMIT            3.2f      // 初期限制 ±1.2V
#define POS_DEADBAND_DEG    0.5f      // 小误差死区
#define SPEED_DEADBAND_RPM  3.0f

#define SPEED_I_OUT_LIMIT   1.5f
#define CURENT_I_OUT_LIMIT  0.3f
// #define MOTOR0_UQ_DIR   (-1.0f)
/*
 * Uq输出方向修正：
 * 用来让 Uq_cmd 对应你想要的电机机械方向。
 */
#define MOTOR0_UQ_DIR          (1.0f)

/*
 * 正转对应的Iq方向：
 * 如果实测正转时 Iq 是负数，这里就填 -1。
 * 如果实测正转时 Iq 是正数，这里就填 +1。
 */
#define MOTOR0_FORWARD_IQ_DIR  (-1.0f)



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
 * @brief PID运算
 * 
 * @param kp 比例参数
 * @param ki 积分参数
 * @param ki_out_min 积分限幅最小值
 * @param ki_out_max 积分限幅最大值
 * @param kd 微分参数
 * @param exp_v 期望值
 * @param now_v 当前值
 * @return float PID的输出值out
 * 
 *  位置式PID:(dt)
    u[k] = Kp * e[k] + Ki * sum(e[0..k]) * dt + Kd * (e[k] - e[k-1]) / dt

    // 增量式PID（FOC中更常用）
    delta_u[k] = Kp * (e[k] - e[k-1]) + Ki * e[k] * dt + Kd * (e[k] - 2*e[k-1] + e[k-2]) / dt
    u[k] = u[k-1] + delta_u[k]
 */
float vfoc_pid_calute(  float kp,
                        float ki,
                        float ki_out_min,
                        float ki_out_max,
                        float kd,
                        float exp_v,
                        float now_v)
{

    int64_t stamp_time_us = esp_timer_get_time();/*单位us*/
    static int64_t last_stamp_time_us = 0;
    float dt_s = 0.0f;/*pid计算时间间隔，单位s*/

    float err_now = exp_v - now_v;/*当前误差 = 期望值-当前值*/
    static float last_err = 0.0f;/*上次误差值*/
    
    float kp_out = 0.0f;/*比例输出*/

    float ki_out = 0.0f;/*积分输出*/
    static float ki_err_sum = 0.0f;
    
    float kd_out = 0.0f;/*微分输出*/
    float kd_parm = 0.0f;
    
    float pid_out = 0.0f;/*PID整体结果输出*/


    /*
     * 第一次调用：
     * 只初始化时间和误差，不计算I和D。
     * 防止第一次dt异常导致积分/微分突变。
     */
    if (last_stamp_time_us==0)
    {/*第一次不计算时间,第一次的last_stamp_time_us=0，计算出的时间有问题的*/

        last_stamp_time_us = stamp_time_us; 
        last_err = err_now;/*更新上次误差值*/

        /*比例部分*/
        kp_out = (kp * err_now);
        return kp_out;/*第一次只计算比例*/
    }
    
    /*
     * 计算PID调用间隔。
     * esp_timer_get_time()单位是us，这里转换成s。
    */
    dt_s = (float)((stamp_time_us - last_stamp_time_us)/(1000000.0f));/*us转化成s*/

    /*
     * dt保护。
     * 防止dt太小导致D项爆炸。
     * 如果你的FOC控制周期是1kHz，可以默认按0.001s处理。
     */
    if (dt_s <= 0.000001f || dt_s > 1.0f)
    {
        dt_s = 0.001f;
    }

    /*比例部分*/
    kp_out = (kp * err_now);

    /*
     * 积分部分。
     * 重点：限制积分累加值，而不是只限制ki_out。
     */
    if ((ki > 0.000001f) || (ki < -0.000001f))
    {
        float ki_err_sum_min = ki_out_min / ki;
        float ki_err_sum_max = ki_out_max / ki;

        /*
         * 如果ki是负数，上面除完以后min/max可能反过来，所以这里修正一下。
         */
        if (ki_err_sum_min > ki_err_sum_max)
        {
            float temp = ki_err_sum_min;
            ki_err_sum_min = ki_err_sum_max;
            ki_err_sum_max = temp;
        }

        ki_err_sum += (err_now * dt_s);
        ki_err_sum = limit_float(ki_err_sum, ki_err_sum_min, ki_err_sum_max);

        ki_out = ki * ki_err_sum;
    }
    else
    {
        /*
         * ki为0时，不做积分。
         * 防止ki以后重新打开时，旧积分突然冒出来。
         */
        ki_err_sum = 0.0f;
        ki_out = 0.0f;
    }

    /*kd微分参数:本次误差值-上一次误差值*/
    kd_parm = err_now - last_err;
    kd_out = (kd * (kd_parm / dt_s));

    pid_out = kp_out + ki_out +kd_out;

    last_stamp_time_us = stamp_time_us; 
    last_err = err_now;/*更新上次误差值*/

    return pid_out;
}


/**
 * @brief 位置环
 * 
 */
void vfoc_position_loop(void)
{
    float now_angle = get_vfoc_theta_m_deg();/* 获取当前机械角度值 */

    // 第三，如果用在位置环角度控制，err_now = exp_v - now_v 暂时不适合处理 0°/360° 跨界。速度环没问题，位置环后面要换成：

    // err_now = angle_error_deg(exp_v, now_v);

    // LIMIT_EXP_MECH_360(exp_angle);
    // err_angle = angle_error_deg( exp_angle , now_angle );/*本次误差值*/
    // now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/
    // err_motor_rpm = exp_motor_rpm - now_motor_rpm;/*本次误差值*/

}

/**
 * @brief 速度环
 * 
 */
void vfoc_speed_loop(void)
{
    float now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/

}


/**
 * @brief 电流环
 * 
 * @return float PID算出的Uq值
 */
park_parm_t vfoc_curent_loop(void)
{
    park_parm_t l_temp_park_v = {0};

    /*获取三相实际电流值，已经过低通滤波*/
    float motor_ia = get_vfoc_ia_current();
    float motor_ib = get_vfoc_ib_current();
    float motor_ic = get_vfoc_ic_current();

    /*clark变换*/
    clark_parm_t clark_temp = clark_tansform(
        motor_ia,
        motor_ib,
        motor_ic
    );

    /*获取电角度*/
    float theta_e_temp = get_vfoc_theta_e_rad();

    /*park变换*/
    park_parm_t park_temp = park_tansform(
        clark_temp.I_alpha,
        clark_temp.I_beta,
        theta_e_temp
    );

    float now_iq = park_temp.Uq;/*当前实际的Uq值*/

    /*当前电源每V电压支持0.071A， 0.071A/V，
    12V 是母线总电压（VBUS），在 SVPWM 调制下，d/q 轴电压的理论最大幅值只有约 6.93V 
    6.93*0.071A=0.49A
    或者直接uq=6.93V,测试堵转电流值*/
    float exp_iq = 0.40f;/*期望iq值*/


    float kp = 0.99f;
    float ki = 0.0f;/*0.001f ~ 0.005f;*/
    float kd = 0.0f;/*0.001f ~ 0.005f;*/
    
    /**
     * @brief FOC电流环_Iq_PI控制
     * 
     */
    float pid_out_uq = vfoc_pid_calute(
        kp,
        ki,
        -CURENT_I_OUT_LIMIT,
        +CURENT_I_OUT_LIMIT,
        kd,
        exp_iq,/*expect:0.8A*/
        now_iq
    );


    float exp_id = 0.0f;/*期望id值*/
    float now_id = park_temp.Ud;/*当前实际的Uq值*/
    kp = 0.3f;
    ki = 0.01f;
    kd = 0.0f;

    /**
     * @brief FOC电流环_Id_PI控制
     * 
     */
    float pid_out_ud = vfoc_pid_calute(
        kp,
        ki,
        -CURENT_I_OUT_LIMIT,
        +CURENT_I_OUT_LIMIT,
        kd,
        exp_id,/*expect:0.8A*/
        now_id
    );

    /*输出uq限幅*/
    pid_out_uq = limit_float(pid_out_uq, -UQ_LIMIT, +UQ_LIMIT);
    // float uq_cmd = MOTOR0_FORWARD_IQ_DIR * pid_out_uq;
    float uq_cmd = 3.5;
    
    l_temp_park_v.Uq = uq_cmd;
    // l_temp_park_v.Uq = pid_out_uq;
    l_temp_park_v.Ud = pid_out_ud;

    /*
     * 统计平均绝对值，避免只看某一个瞬时点。
     * 因为 ia/ib/ic/iq 是交流量，单点日志可能刚好采到过零点。
     */
    static uint32_t log_cnt = 0;

    static float ia_abs_sum = 0.0f;
    static float ib_abs_sum = 0.0f;
    static float ic_abs_sum = 0.0f;
    static float id_abs_sum = 0.0f;
    static float iq_abs_sum = 0.0f;

    ia_abs_sum += fabsf(motor_ia);
    ib_abs_sum += fabsf(motor_ib);
    ic_abs_sum += fabsf(motor_ic);
    id_abs_sum += fabsf(now_id);
    iq_abs_sum += fabsf(now_iq);

    log_cnt++;

    if (log_cnt >= 100)
    {
        float ia_abs_avg = ia_abs_sum / (float)log_cnt;
        float ib_abs_avg = ib_abs_sum / (float)log_cnt;
        float ic_abs_avg = ic_abs_sum / (float)log_cnt;
        float id_abs_avg = id_abs_sum / (float)log_cnt;
        float iq_abs_avg = iq_abs_sum / (float)log_cnt;

        float iq_err = exp_iq - now_iq;
        float id_err = exp_id - now_id;

        ESP_LOGI(
            TAG,
            "cur_loop: "
            "uq_cmd=%.3fV, ud_cmd=%.3fV, "
            "uq_pid=%.3fV, ud_pid=%.3fV, "
            "iq_ref=%.3fA, iq_now=%.3fA, iq_err=%.3fA, "
            "id_ref=%.3fA, id_now=%.3fA, id_err=%.3fA, "
            "ia=%.3fA, ib=%.3fA, ic=%.3fA, "
            "iq_abs_avg=%.3fA, id_abs_avg=%.3fA, "
            "ia_abs_avg=%.3fA, ib_abs_avg=%.3fA, ic_abs_avg=%.3fA, "
            "theta=%.3frad",
            uq_cmd,
            pid_out_ud,
            pid_out_uq,
            pid_out_ud,
            exp_iq,
            now_iq,
            iq_err,
            exp_id,
            now_id,
            id_err,
            motor_ia,
            motor_ib,
            motor_ic,
            iq_abs_avg,
            id_abs_avg,
            ia_abs_avg,
            ib_abs_avg,
            ic_abs_avg,
            theta_e_temp
        );

        log_cnt = 0;
        ia_abs_sum = 0.0f;
        ib_abs_sum = 0.0f;
        ic_abs_sum = 0.0f;
        id_abs_sum = 0.0f;
        iq_abs_sum = 0.0f;
    }
    

    return l_temp_park_v;
}

/**
 * @brief 力矩环
 * 
 */
void vfoc_torque_loop(void)
{
        // LIMIT_EXP_MECH_360(exp_angle);
    // now_angle = get_vfoc_theta_m_deg();/* 获取当前机械角度值 */
    // err_angle = angle_error_deg( exp_angle , now_angle );/*本次误差值*/
    // now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/
    // err_motor_rpm = exp_motor_rpm - now_motor_rpm;/*本次误差值*/
    
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
 * 
 *
 * @param arg 任务参数，当前未使用
 */
static void m0_foc_control_task(void *arg)
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
esp_err_t m0_fd6287_foc_start(void)
{
    /*
    * 1. 创建 FOC 控制任务。
    *    创建一个固定运行在 CPU 核心1上的高优先级电机控制任务
    */
    xTaskCreatePinnedToCore(
        m0_foc_control_task,        /* 任务函数指针：FOC 电机控制主循环函数（无限循环） */
        "m0_foc_task",              /* 任务名称：调试时方便识别，无实际功能 */
        MO_FOC_CONTROL_TASK_STACK,  /* 任务堆栈大小：分配 4096 字节栈空间（ESP32 单位是字，不是字节） */
        NULL,                       /* 任务入参：不需要传递参数，填 NULL */
        MO_FOC_CONTROL_TASK_PRIO,   /* 任务优先级：20级（最高优先级，保证电机控制实时性） */
        &s_m0_foc_task_handle,      /* 任务句柄：输出参数，保存创建的任务句柄，用于后续任务管理 */
        MO_FOC_CONTROL_TASK_CORE    /* 绑定CPU核心：指定任务**只运行在 CPU 1** 上 */
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

    /*
     * 5. 启动 GPTimer。
     */
    ESP_ERROR_CHECK(gptimer_enable(s_m0_ctrl_timer));
    ESP_ERROR_CHECK(gptimer_start(s_m0_ctrl_timer));

    ESP_LOGI(TAG,
             "M0 FOC started, ctrl_freq=%dHz",
             M0_FOC_CTRL_FREQ_HZ
    );

    return ESP_OK;
}


