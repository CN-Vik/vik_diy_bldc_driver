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
#define FOC_DEG_TO_RAD(deg)     ( ( ((float)deg) * FOC_PI ) / 180.0f )

// √3/2 的值（约 0.8660），FOC 算法中常用的固定系数
#define FOC_SQRT3_DIV_2        0.8660254037844386f
// 2/√3 的值（约 1.1547），FOC 算法中常用的固定系数
#define FOC_2_DIV_SQRT3        1.1547005383792515f

/*
 * 量产级 PWM 占空比保护。
 *
 * FD6287 这类 bootstrap 高边驱动，不建议长时间 0% 或 100%。
 * 你 m0_fd6287_pwm.c 里原本也是 0.02 ~ 0.98。
 */
#define VFOC_PWM_DUTY_MIN      0.02f
#define VFOC_PWM_DUTY_MAX      0.98f

// #define USE_FOC_SPWM
#define USE_FOC_SVPWM

/*
 * 浮点计算保护阈值。
 */
#define VFOC_FLOAT_EPSILON     1.0e-6f


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
}pwm_duty_t;


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
    pwm_duty_t pwm_duty_val;
    float mech_rpm;/*机械角度转速*/
    float mech_w;/*机械角速度*/

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





/**
 * @brief SVPWM 运行状态
 */
typedef enum
{
    VFOC_STATUS_OK = 0,

    /* 输入参数有问题 */
    VFOC_STATUS_NULL_PTR,

    /* vbus 不合法 */
    VFOC_STATUS_BAD_VBUS,

    /* alpha/beta 里出现 NaN 或 Inf */
    VFOC_STATUS_BAD_INPUT,

    /*
     * 电压矢量超过线性调制区，已经被缩放。
     * 这不是严重错误，表示算法做了限幅保护。
     */
    VFOC_STATUS_SATURATED,

} vfoc_status_t;


void vfoc_init(void);
void vfoc_update_open_loop_angle(float target_rpm, float dt_s);
motor_driver_parm_t clark_inv_transform(const clark_parm_t *c_v);
clark_parm_t park_inv_transform(const foc_data_t *foc_v);
pwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus);
void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);
pwm_duty_t vfoc_get_pwm_duty(void);

/*
 * 量产级 SVPWM 核心接口：
 * alpha/beta + vbus -> duty
 */
vfoc_status_t vfoc_svpwm_calc_duty_uab(const clark_parm_t *c_v,
                                      float vbus,
                                      pwm_duty_t *duty_out);
/*
 * 开环 SVPWM 运行接口。
 */
void vfoc_open_loop_svpwm_run(float target_rpm,
                              float uq,
                              float vbus,
                              float dt_s);

void vfoc_set_svpwm(float uq,
                    float ud,
                    float vbus);

float get_vfoc_theta_m_deg(void);

void set_vfoc_mech_w(float mech_w);
float get_vfoc_mech_w(void);
void set_vfoc_mech_rpm(float mech_rm);
float get_vfoc_mech_rpm(void);


#endif