/**
 * @file vik_foc.h
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-31
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef VIK_FOC_H
#define VIK_FOC_H

#include <stdint.h>


/*更号3*/
#define SQRT3               1.7320508075688772f
#define FOC_PI              3.14159265358979323846f
#define FOC_2PI             6.28318530717958647692f

// 角度 → 弧度
#define FOC_DEG_TO_RAD(angle) ((angle) * FOC_PI / 180.0f)




/**
 * @brief 克拉克变换参数
 * 
 */
typedef struct
{
    /*clark变换的关键参数*/
    float I_alpha;
    float I_beta;

}clark_parm_t;


/**
 * @brief 帕克变换参数
 * 
 */
typedef struct
{
    /*park变换的关键值*/
    float Ud;
    float Uq;

}park_parm_t;


typedef struct 
{
    float duty_Ua;/*实际配置到电机的占空比Ua的*/
    float duty_Ub;
    float duty_Uc;
}spwm_duty_t;


/**
 * @brief 电机驱动参数
 * 
 */
typedef struct
{
    /*最终输入到电机的电压值*/
    float Ua;
    float Ub;
    float Uc;
    spwm_duty_t spwm_duty_val;

}motor_driver_parm_t;


/**
 * @brief 电机参数（极对数，电角度，机械角度）
 * 
 */
typedef struct
{
    /*弧度制*/
    float theta_e;/*电角度:转子磁场 / d轴 相对于定子 α 轴的电角度*/
    unsigned int pole_pairs;/*电机磁极对数*/
    float theta_m;/*电机的机械角度：电机转子实际转过的角度*/

}motor_parm_t;


/**
 * @brief foc数据结构体，
 *      使用一个变量来统一管理
 * 
 */
typedef struct 
{
    /*克拉克参数值*/
    clark_parm_t clark_val;
    
    /*帕克参数值*/
    park_parm_t park_val;
    
    /*电机驱动参数值，Ua,Ub,Uc电压值,
        以及三相Ua,Ub,Uc占空比的值
    */
    motor_driver_parm_t motor_drv_val;

    /*电机实际硬件参数：（极对数，电角度，机械角度）*/
    motor_parm_t motor_par;
    
}foc_data_t;


void vfoc_init(void);
void vfoc_update_open_loop_angle(float target_rpm, float dt_s);
motor_driver_parm_t clark_inv_transform(const clark_parm_t *c_v);
clark_parm_t park_inv_transform(const foc_data_t *foc_v);
spwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus);
void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);
spwm_duty_t vfoc_get_spwm_duty(void);


#endif