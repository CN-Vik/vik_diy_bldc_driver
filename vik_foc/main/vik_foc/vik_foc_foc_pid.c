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

    if ( fabs(pid->pid_out) < pid->pid_out_max)/*防止积分饱和*/
    {
        /*积分部分：先累加误差 ,当超过当前值超过期望值，积分部分就起负反馈作用*/
        pid->ki_integral += (pid->err_v * pid->pid_dt);
        
    }

    
    /* 计算积分输出 */
    pid->ki_out = (pid->ki * pid->ki_integral);
    /*积分输出限幅*/
    if (pid->ki_out > pid->ki_out_max)
    {
        pid->ki_out = pid->ki_out_max;
    }
    
    if (pid->ki_out < pid->ki_out_min )
    {
        pid->ki_out = pid->ki_out_min;
    }


    /*kd微分参数:本次误差值 - 上一次误差值*/
    pid->kd_parm = pid->err_v - pid->last_err_v;
    pid->kd_out = ( pid->kd * (pid->kd_parm / pid->pid_dt) );

    /*pid计算输出*/
    pid->pid_out = pid->kp_out + pid->ki_out + pid->kd_out;

    /*pid输出限幅*/
    if (pid->pid_out > pid->pid_out_max)
    {
        pid->pid_out = pid->pid_out_max;
    }

    if (pid->pid_out < pid->pid_out_min)
    {
        pid->pid_out = pid->pid_out_min;
    }
    

}

