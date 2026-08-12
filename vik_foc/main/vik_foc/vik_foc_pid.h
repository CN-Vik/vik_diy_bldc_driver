/**
 * @file vik_foc_pid.h
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef VIK_FOC_PID_H
#define VIK_FOC_PID_H

#include "stdint.h"


#define CURRENT_LOOP_FREQ       (20000.0f)  /*20KHZ*/
#define CURRENT_LOOP_DT         (1.0f/CURRENT_LOOP_FREQ) /*50us = 0.00005s*/
#define SPEED_LOOP_DT           ( 1.0f / (CURRENT_LOOP_FREQ/20.0f) ) /* 20/20= 1KHZ*/
#define POSTION_LOOP_DT         ( 1.0f / (CURRENT_LOOP_FREQ/100.0f) ) /* 20/100= 200HZ*/


/**
 * @brief FOC PID 控制器结构体
 */
typedef struct
{
    float kp;                 /* 比例系数 */
    float ki;                 /* 积分系数 */
    float kd;                 /* 微分系数 */

    float pid_dt;             /*pid控制周期，单位s*/

    float exp_v;             /* 目标值 */
    float now_v;                /* 当前值 */
    float err_v;              /* 当前误差 */
    float last_err_v;         /* 上一次误差 */

    float ki_integral;            /* 积分误差累计 */
    float ki_sep_err_thr;          /* 积分分离的阈值 , ki_integral_speartion_err_threshold*/
    float kd_parm;           /* 微分参数 */

    float kp_out;
    float ki_out;
    float ki_out_min;
    float ki_out_max;
    float kd_out;
    float pid_out;             /* PID 输出 */

    float pid_out_min;         /* 输出最小限幅 */
    float pid_out_max;         /* 输出最大限幅 */

} vfoc_pid_t;


void vfoc_pid_calt(vfoc_pid_t *pid);


#endif