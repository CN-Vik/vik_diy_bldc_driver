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
#include "esp_timer.h"

static const char *TAG = "FOC_PID";


/**
 * @brief PID运算(通用API)
 * 
 *  *  位置式PID:(dt)
    u[k] = Kp * e[k] + Ki * sum(e[0..k]) * dt + Kd * (e[k] - e[k-1]) / dt

 * @param pid pid结构体变量指针
 * @return float 
 */
void vfoc_pid_calt(vfoc_pid_t *pid)
{
    if (pid==NULL)
    {
        ESP_LOGI(
            TAG,
            "vfoc_pid_calt_pid_is_null!\r\n"
        );
        return;
    }
    
    float pid_out = 0.0f;/*PID整体结果输出*/

    /*比例部分: KP*当前误差值*/
    pid->kp_out = (pid->kp * pid->err_v);

    /*积分部分*/
    pid->ki_sum_err += (pid->err_v * pid->pid_dt);
    /*积分限幅*/
    pid->ki_sum_err = limit_float(pid->ki_sum_err, pid->ki_integral_min, pid->ki_integral_max);
    pid->ki_out = (pid->ki * pid->ki_sum_err);

    /*kd微分参数:本次误差值 - 上一次误差值*/
    pid->kd_parm = pid->err_v - pid->last_err_v;
    pid->kd_out = ( pid->kd * (pid->kd_parm / pid->pid_dt) );

    pid->pid_out = pid->kp_out + pid->ki_out + pid->kd_out;

}

