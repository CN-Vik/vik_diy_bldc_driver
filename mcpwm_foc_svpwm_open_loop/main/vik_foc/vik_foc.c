/**
 * @file vik_foc.c
 * @author vik (ufo281@outlook.com)
 * @brief vik diy foc
 * @version 0.1
 * @date 2026-05-31
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "vik_foc.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "vik_foc:";


/**
 * @brief 所使用电机为2208，电机参数如下
 * 
1.3 2208电机模块
1.3.1 参数介绍
电机主要参数：

KV值： 110KV

额定电压： 12V

额定电流： 2A

额定功率： 14W

额定转速： 1800RPM

极对数：     14/12,  7极对数

电阻：         21.2Ω 
 */

/*viK_foc_data变量*/
foc_data_t vfoc_dt={0};



/**
 * @brief FOC电机相关参数初始化
 * 
 */
void vfoc_init(void)
{
    /*所使用的是2208电机，极对数为7*/
    vfoc_dt.motor_par.theta_m = 0.0f;
    vfoc_dt.motor_par.theta_e = 0.0f;
    vfoc_dt.motor_par.pole_pairs = 7;
    
    vfoc_dt.park_val.Ud = 0.0f;
    vfoc_dt.park_val.Uq = 0.0f;
}



/**
 * @brief （比如2208电机：0 ~ 2500 RPM）
 * 
 *  开环模式：更新电机电角度 
 * theta_e target_rpm：目标转速（转/分钟） 
 * dt_s：距离上一次调用的时间间隔（秒）
 * 
 * @param target_rpm target_rpm (r/min） 
 * @param dt_s 距离上一次调用的时间间隔（秒）
 */
void vfoc_update_open_loop_angle(float target_rpm, float dt_s)
{

    float W_m = 0.0f;  // 机械角速度 rad/s

    // 1. RPM 转 机械角速度 ωₘ (弧度/秒)
    // RPM = 每分钟转 RPM 圈
    // 1 圈 = 2π 弧度
    // 每分钟总弧度=RPM×2π
    // 每秒钟总弧度=(RPM×2π)/60
    // 公式：ωₘ = RPM × 2π / 60 ,一秒转过多少弧度
    W_m = target_rpm * FOC_2PI / 60.0f;

    // 3. 累积机械角度（对机械速度积分算出机械角度）：角度 = 角度 + 角速度 × 时间
    // 这就是“开环旋转”的本质
    vfoc_dt.motor_par.theta_m += W_m * dt_s;

    /*计算出电角度 = 机械角度*电机磁极对数 */
    vfoc_dt.motor_par.theta_e = vfoc_dt.motor_par.theta_m *
        vfoc_dt.motor_par.pole_pairs;
    
    /*电角度归一化 0 ~ 2π*/
    while (vfoc_dt.motor_par.theta_e >= FOC_2PI)
    {
        vfoc_dt.motor_par.theta_e -= FOC_2PI;
    }

    // 角度为负数，就加上一圈
    while (vfoc_dt.motor_par.theta_e < 0.0f)
    {
        vfoc_dt.motor_par.theta_e += FOC_2PI;
    }
}



/**
 * @brief 克拉克逆变换(clark_inverse_transform)
 *      输入 I_alpha,I_beta,输出三相电压值Ua,Ub,Uc
 * 
 *  公式：
 * u(Ua) = I_alpha
 * v(Ub) = (sqrt(3) * I_beta - I_alpha) / 2
 * w(Uc) = -u - v
 * 
 * (Ua+Ub+Uc=0) => Uc = -Ua-Ub
 * 
 * @param c_v (clark_value)克拉克变换的参数 I_alpha，I_beta
 * @return motor_driver_parm_t 三相电压值Ua,Ub,Uc
 */
motor_driver_parm_t clark_inv_transform(const clark_parm_t *c_v)
{
    /*本地临时克拉克变量值*/
    clark_parm_t l_temp_c_v = {0};
    /*本地临时电机驱动变量值*/
    motor_driver_parm_t l_temp_motor_drv_val={0};
    if (c_v==NULL)
    {/*安全检查*/
        return l_temp_motor_drv_val;
    }

    /*兼容老版本C语言编译器，C89等*/
    l_temp_c_v.I_alpha = c_v->I_alpha;
    l_temp_c_v.I_beta = c_v->I_beta;

    l_temp_motor_drv_val.Ua = l_temp_c_v.I_alpha;
    l_temp_motor_drv_val.Ub = ( (SQRT3 *l_temp_c_v.I_beta) - l_temp_c_v.I_alpha )*0.5f;
    l_temp_motor_drv_val.Uc = ( (-l_temp_motor_drv_val.Ua)-l_temp_motor_drv_val.Ub );

    return l_temp_motor_drv_val;

}


/**
 * @brief 帕克逆变换(parker_inverse_transform)
 *   输入Id,Iq, 输出I_alpha,I_beta
 * 
 * @param foc_v (foc_value) 帕克逆变换参数值 Ud,Uq和电角度等数据
 * @return clark_parm_t: 输出I_alpha,I_beta
 * 
 *  * 公式：
 * alpha = d * cos(theta) - q * sin(theta)
 * beta  = q * cos(theta) + d * sin(theta)
 */
clark_parm_t park_inv_transform(const foc_data_t *foc_v)
{
    park_parm_t l_temp_park_value = {0};
    /*克拉克临时变量值*/
    clark_parm_t l_temp_c_v = {0};
    if (foc_v==NULL)
    {/*安全检查*/
        return l_temp_c_v;
    }

    float cos_theta = cosf(foc_v->motor_par.theta_e);
    float sin_theta = sinf(foc_v->motor_par.theta_e);
    
    l_temp_park_value.Ud = foc_v->park_val.Ud;
    l_temp_park_value.Uq = foc_v->park_val.Uq;
    

    l_temp_c_v.I_alpha = (cos_theta*foc_v->park_val.Ud) -
                         (sin_theta*foc_v->park_val.Uq);

    l_temp_c_v.I_beta = (sin_theta*foc_v->park_val.Ud) +
                        (cos_theta*foc_v->park_val.Uq);

    return l_temp_c_v;
}



/**
 * @brief 把 PWM 占空比强制限制在 0.0 ~ 1.0 之间，防止输出非法占空比
 * 
 * 理论上希望得到：
 *      duty_Ua / duty_Ub / duty_Uc ∈ 0.0 ~ 1.0
 * 
 * @param value 
 * @param min 0 (表示占空比100%)
 * @param max 1 (表示占空比100%)
 * @return float 
 */
static float vfoc_limit(float value, float min, float max)
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

/**
 * @brief 根据Ua,Ub,Uc 输出三相的PWM值(最大为100%)
 * 
 * @param motor_v 
 * @param vbus 
 * @return spwm_duty_t 
 */
spwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus)
{
    spwm_duty_t duty = {0};

    if ((motor_v == NULL) || (vbus <= 0.0f))
    {
        return duty;
    }

    duty.duty_Ua = 0.5f + motor_v->Ua / vbus;
    duty.duty_Ub = 0.5f + motor_v->Ub / vbus;
    duty.duty_Uc = 0.5f + motor_v->Uc / vbus;

    duty.duty_Ua = vfoc_limit(duty.duty_Ua, 0.0f, 1.0f);/*限制输出的PWM超过100%*/
    duty.duty_Ub = vfoc_limit(duty.duty_Ub, 0.0f, 1.0f);
    duty.duty_Uc = vfoc_limit(duty.duty_Uc, 0.0f, 1.0f);

    return duty;
}


/**
 * @brief 
 * 
 * @param target_rpm 
 * @param uq 
 * @param vbus 
 * @param dt_s 如果 dt_s 不稳定，电角度增长就不稳定，电机就容易：
        抖动
        啸叫
        转速不稳
        启动失败
 */
void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s)
{
    clark_parm_t l_temp_clark_v = {0};
    

    if (dt_s <= 0.0f)
    {
        return;
    }

    /*
     * 1. 根据目标转速更新电角度
     */
    vfoc_update_open_loop_angle(target_rpm, dt_s);

    /*
     * 2. 开环电压 FOC：d轴为0，q轴给电压
    先设置 Ud/Uq 
    如果你是 12V 电源，SPWM 下建议初期小一点：Uq = 0.5f ~ 2.0f;
    
    Uq 太大可能表现为：
        电机啸叫
        抖动
        电流大
        驱动芯片发热
        电机发热
     */
    vfoc_dt.park_val.Uq = uq;
    vfoc_dt.park_val.Ud = 0.0f;

    /*
     * 3. 逆 Park：Ud/Uq -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. 逆 Clarke：Ualpha/Ubeta -> Ua/Ub/Uc
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*如果 sum 接近 0，说明逆 Clarke 输出也正常。*/
    ESP_LOGI(TAG, "UVW: %.3f, %.3f, %.3f, sum=%.3f",
        vfoc_dt.motor_drv_val.Ua,
        vfoc_dt.motor_drv_val.Ub,
        vfoc_dt.motor_drv_val.Uc,
        vfoc_dt.motor_drv_val.Ua +
        vfoc_dt.motor_drv_val.Ub +
        vfoc_dt.motor_drv_val.Uc
    );

    /*
     * 5. SPWM：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
     */
    vfoc_dt.motor_drv_val.spwm_duty_val =
        vfoc_spwm_calc_duty(&vfoc_dt.motor_drv_val, vbus);
}

spwm_duty_t vfoc_get_spwm_duty(void)
{
    return vfoc_dt.motor_drv_val.spwm_duty_val;
}