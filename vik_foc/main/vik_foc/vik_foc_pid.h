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

/**
 * @brief FOC PID 控制器结构体
 */
typedef struct
{
    float kp;                 /* 比例系数 */
    float ki;                 /* 积分系数 */
    float kd;                 /* 微分系数 */

    float target;             /* 目标值 */
    float feedback;           /* 反馈值 */
    float error;              /* 当前误差 */
    float last_error;         /* 上一次误差 */

    float integral;           /* 积分累计值 */
    float derivative;         /* 微分项 */

    float output;             /* PID 输出 */

    float output_min;         /* 输出最小限幅 */
    float output_max;         /* 输出最大限幅 */

    float integral_min;       /* 积分最小限幅 */
    float integral_max;       /* 积分最大限幅 */
} vfoc_pid_t;


typedef struct
{
    float kp;
    float ki;
    float kd;

    float target_angle_deg;      /* 目标机械角度，单位：度 */
    float current_angle_deg;     /* 当前机械角度，单位：度 */

    float error_deg;
    float last_error_deg;

    float integral;
    float derivative;

    float uq;                    /* 位置环输出的 Uq 电压 */
    float torque_limit_v;        /* 力矩限制，本质是 Uq 限幅 */

    float integral_limit;
} vfoc_position_pid_ctrl_t;


void vfoc_position_ctrl_init(vfoc_position_pid_ctrl_t *ctrl,
                             float kp,
                             float ki,
                             float kd,
                             float torque_limit_v);

void vfoc_position_set_target(vfoc_position_pid_ctrl_t *ctrl,
                              float target_angle_deg);

void vfoc_position_set_torque_limit(vfoc_position_pid_ctrl_t *ctrl,
                                    float torque_limit_v);

float vfoc_position_ctrl_calc(vfoc_position_pid_ctrl_t *ctrl,
                              float current_angle_deg,
                              float dt_s);



#endif