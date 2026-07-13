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
#include <stdbool.h>


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


// 360° 环形期望值限幅（自动绕回）
#define LIMIT_EXP_MECH_360(exp_mech)                \
do {                                            \
    while ((exp_mech) >= 360.0f) (exp_mech) -= 360.0f; \
    while ((exp_mech) < 0.0f)    (exp_mech) += 360.0f; \
} while(0)



/**Uq_max ≈ 12 / 1.732 ≈ 6.9V */
#define UQ_LIMIT            5.5f      // uq = 6.5, iq_max=0.559A ,iq/uq=0.086A/uq
#define IQ_LIMIT            (UQ_LIMIT*0.086f)/*uq设置的情况下，iq最大值=0.559*/
#define POS_DEADBAND_DEG    0.5f      // 小误差死区
#define SPEED_DEADBAND_RPM  3.0f

#define CURENT_I_OUT_LIMIT      (UQ_LIMIT*0.5f)
#define SPEED_PID_OUT_LIMIT     (IQ_LIMIT)/*速度PID输出限幅*/
#define SPEED_I_OUT_LIMIT       (IQ_LIMIT*0.5)/*速度PID积分限幅*/
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

#define MOTOR_DRV_VBUS             12.0f  /* 12V */


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
    /*逆park变换的关键值，输出的电压值*/
    float Ud;
    float Uq;

    /*正park变换值，实际的电流值dq电流值*/
    float id;
    float iq;
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

    volatile float ia;/*电机a相电流*/
    volatile float ib;
    volatile float ic;

    volatile float iq;
    volatile float id;

    pwm_duty_t pwm_duty_val;
    volatile float mech_rpm;/*机械角度转速*/
    volatile float w_mech;/*机械角速度*/
    volatile float w_e;/*电角速度*/

}motor_driver_parm_t;


/**
 * @brief 电机参数（极对数，电角度，机械角度）
 * 
 */
typedef struct
{
    /*弧度制*/
   volatile float theta_e;/*电角度:转子磁场 / d轴 相对于定子 α 轴的电角度*/
   volatile unsigned int pole_pairs;/*电机磁极对数*/
   volatile float theta_m;/*电机的机械角度：电机转子实际转过的角度*/

   volatile float theta_e_offset_mech;/*零电角度时候的机械角度偏移值，每次重启都会一直变化*/
   volatile float zero_theta_e_calib_flag;/*零电角度校准标志*/

   uint32_t KV; /*电机KV值，KV=100,供电5V，电机理论转速=5*KV=5*100=500rpm*/
   float Phase_Rs; /*电机相电阻*/
   float Phase_Ls; /*电机相电感*/
    /*表贴式 PMSM（最常见BLDC），相电感等于d轴，q轴电感*/
   float Ld; /*d轴电感*/
   float Lq; /*q轴电感*/

    volatile float ia;/*电机a相电流*/
    volatile float ib;
    volatile float ic;

   float psi_f;/*转子永磁体磁链*/
   float psi_d;/*d轴磁链*/
   float psi_q;/*q轴磁链*/

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

typedef struct
{
    float alpha;      // 滤波系数
    float output;     // 上一次输出
    uint8_t init;     // 初始化标志
} lp_filter_t;



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

} vfoc_status_e_t;


typedef struct 
{
    int64_t strat_t;/*开始时间戳*/
    int64_t end_t;/*结束时间戳*/
    int64_t dt;/*时间间隔*/
}time_stamp_t;

typedef struct 
{
    time_stamp_t time[100];
    uint64_t index;/*索引号*/

}vfoc_time_stamp_t;


extern foc_data_t vfoc_m0_dt;




/*-------------------低通滤波---------------------------*/
/**
 * @brief use_example
 * 
lp_filter_t angle_f;

lp_filter_init(&angle_f, 0.05f);

theta = lp_filter_update(&angle_f, raw_angle);
 *
 * */
void lp_filter_init(lp_filter_t *f, float alpha);
float lp_filter_update(lp_filter_t *f, float input);


/*-------------------低通滤波---------------------------*/
bool get_zero_theta_e_calib_flag(void);
void set_zero_theta_e_calib_flag(bool flag);
void set_theta_e_offset_mech(float mech_offset);
float get_theta_e_offset_mech(void);
float get_vfoc_mech_rpm(void);
float limit_float(float x, float min, float max);
void set_vfoc_theta_e_rad(float e_value);
float get_vfoc_theta_e_rad(float m_angle);
float low_pass_filter(float input, float alpha);
void vfoc_set_motor_drv_iq(float uq);
float vfoc_get_motor_drv_iq(void);
void set_actual_iq(foc_data_t *vfoc_dt , float iq);
void set_actual_id(foc_data_t *vfoc_dt , float id);
float get_d_cross_couple(foc_data_t *vfoc_dt);
float get_q_cross_couple(foc_data_t *vfoc_dt);

void vfoc_init(foc_data_t *vfoc_dt);
void vfoc_update_open_loop_angle(float target_rpm, float dt_s);
clark_parm_t clark_tansform(float ia, float ib, float ic);
motor_driver_parm_t clark_inv_transform(const clark_parm_t *c_v);
park_parm_t park_tansform(float I_alpha, float I_beta, float theta_e_rad);
clark_parm_t park_inv_transform(const foc_data_t *foc_v);
pwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus);
void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);
pwm_duty_t vfoc_get_pwm_duty(void);
void vfoc_set_spwm( float uq,
                    float ud,
                    float vbus);

/*
 * 量产级 SVPWM 核心接口：
 * alpha/beta + vbus -> duty
 */
vfoc_status_e_t vfoc_svpwm_calc_duty_uab(const clark_parm_t *c_v,
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
float get_vfoc_theta_e_w(foc_data_t *vfoc_data);
void calc_vfoc_theta_e_w(foc_data_t *vfoc_data);
void set_vfoc_mech_w(float w_mech);
float get_vfoc_mech_w(void);
void set_vfoc_mech_rpm(float mech_rm);
float get_vfoc_mech_rpm(void);

void set_vfoc_ia_current(float curent);
void set_vfoc_ib_current(float curent);
void set_vfoc_ic_current(float curent);
float get_vfoc_ia_current(void);
float get_vfoc_ib_current(void);
float get_vfoc_ic_current(void);


#endif