/**
 * @file foc_pid.c
 * @author vik (ufo281@outlook.com)
 * @brief FOC PID 控制器实现
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 */
#include "vik_foc_pid.h"
#include "vik_foc.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "FOC_PID";

vfoc_pid_t vfoc_pid_dt={0};


/**
 * @brief PID运算-电流环Id
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
float vfoc_pid_calt_curent_id(  float kp,
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
 * @brief PID运算-电流环Iq
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
float vfoc_pid_calt_curent_iq(  float kp,
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

