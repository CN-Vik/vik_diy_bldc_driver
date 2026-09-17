#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif





typedef struct
{
    float kp;
    float ki;
    float kd;

} pid_param_t;


/**
 * @brief motor_parma
 * 
 */
typedef struct
{
    float rs;
    float ls;

    float flux;

    int pole_pairs;

    float voltage_limit;

    float current_limit;

} motor_param_t;



/**
 * @brief 全局控制参数
 * 
 */
typedef struct
{
    motor_param_t motor;

    pid_param_t id_pid;

    pid_param_t iq_pid;

    pid_param_t speed_pid;

    pid_param_t position_pid;


    /* SMO */

    float smo_ks;

    float smo_lpf;

    float smo_gain;

    float smo_phase_offset;


    /* PLL */

    float pll_kp;

    float pll_ki;

    float pll_lpf;


    /* Balance */

    float balance_angle_kp;

    float balance_angle_ki;

    float balance_angle_kd;

    float balance_speed_kp;

    float balance_speed_ki;

    float balance_speed_kd;


} control_params_t;



extern control_params_t g_ctrl;



#ifdef __cplusplus
}
#endif

#endif