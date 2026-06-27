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
#include "stdbool.h"

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


/*-------------------低通滤波---------------------------*/
void lp_filter_init(lp_filter_t *f, float alpha)
{
    f->alpha = alpha;
    f->output = 0.0f;
    f->init = 1;
}

/**
 * @brief  一阶低通滤波函数
 * 
 * @param input 当前输入值
 * @param alpha 滤波系数，范围 0.0f ~ 1.0f
 *              alpha 越小，滤波越强，响应越慢
 *              alpha 越大，响应越快，滤波越弱
 * 
alpha = 0.05f;   // 滤波很强，输出很稳，但是响应慢
alpha = 0.10f;   // 比较常用，适合电流/速度滤波
alpha = 0.20f;   // 响应更快，滤波弱一点
alpha = 0.50f;   // 响应很快，滤波比较弱
 * 
角度值	0.05 ~ 0.15	角度不要滤太狠，否则位置环会变迟钝
速度 RPM	0.10 ~ 0.30	速度计算本身抖动大，可以适当滤强一点
电流值	0.05 ~ 0.20	只做显示/保护可以小一点；做电流环不能太小
力矩值	0.10 ~ 0.30	如果力矩来自 Iq，基本跟电流滤波一致
PID 的 D 项	0.05 ~ 0.15	D 项最容易放大噪声，建议滤强一点
 * 
 * 
 * 一阶低通滤波公式：
 * output = last_output + alpha * (input - last_output)
 * 
 * @param f 
 * @param input 
 * @return float 滤波后的输出值
 */
float lp_filter_update(lp_filter_t *f, float input)
{
    if (f->init)
    {
        f->init = 0;
        f->output = input;
        return input;
    }

    f->output = f->output + f->alpha * (input - f->output);
    return f->output;
}


/*-------------------低通滤波---------------------------*/





/**
 * @brief 设置A相电流
 * 
 * @param curent 
 */
void set_vfoc_ia_current(float curent)
{
    vfoc_dt.motor_par.ia = curent;
}



/**
 * @brief 获取A相电流
 * 
 * @return float 
 */
float get_vfoc_ia_current(void)
{
    return vfoc_dt.motor_par.ia;
}



/**
 * @brief 设置B相电流
 * 
 * @param curent 
 */
void set_vfoc_ib_current(float curent)
{
    vfoc_dt.motor_par.ib = curent;
}



/**
 * @brief 获取B相电流
 * 
 * @return float 
 */
float get_vfoc_ib_current(void)
{
    return vfoc_dt.motor_par.ib;
}


/**
 * @brief 设置C相电流
 * 
 * @param curent 
 */
void set_vfoc_ic_current(float curent)
{
    vfoc_dt.motor_par.ic = curent;
}



/**
 * @brief 获取C相电流
 * 
 * @return float 
 */
float get_vfoc_ic_current(void)
{
    return vfoc_dt.motor_par.ic;
}



/**
 * @brief 设置FOC机械角速度w(°/s)
 * 
 * @param mech_w 机械角速度w(°/s)
 */
void set_vfoc_mech_w(float mech_w)
{
    vfoc_dt.motor_drv_val.mech_w = mech_w;
}

/**
 * @brief 获取FOC机械角速度w(°/s)
 * 
 * @return float 机械角速度w(°/s)
 */
float get_vfoc_mech_w(void)
{
    return ((float) vfoc_dt.motor_drv_val.mech_w );
}


/**
 * @brief 设置FOC机械转速RPM(r/min)
 * 
 * @param mech_rm RPM(r/min)
 */
void set_vfoc_mech_rpm(float mech_rm)
{
    vfoc_dt.motor_drv_val.mech_rpm = mech_rm;
}


/**
 * @brief 获取FOC机械转速RPM(r/min)
 * 
 * @return float 机械转速RPM(r/min)
 */
float get_vfoc_mech_rpm(void)
{
    return ((float) vfoc_dt.motor_drv_val.mech_rpm );
}


/**
 * @brief 设置机械角度数据(角度制)
 * 
 * @param parm_angle 
 */
void set_vfoc_theta_m_deg(float parm_angle)
{
    vfoc_dt.motor_par.theta_m = parm_angle;
}

/**
 * @brief 获取FOC的机械角度(角度制)
 * 
 * @return float 
 */
float get_vfoc_theta_m_deg(void)
{
    return ((float) vfoc_dt.motor_par.theta_m );
}

/**
 * @brief 零点角度校准标志
 * 
 * @return true 
 * @return false 
 */
bool get_zero_theta_e_calib_flag(void)
{
    return ((bool) vfoc_dt.motor_par.zero_theta_e_calib_flag);
}


/**
 * @brief 设置电角度零点对其标志(后续写入flah中)
 * 
 * @param flag true or false
 */
void set_zero_theta_e_calib_flag(bool flag)
{
    vfoc_dt.motor_par.zero_theta_e_calib_flag = flag;
}


/**
 * @brief 设置零电角度时候的机械角度偏移值
 * 
 * @param mech_offset 
 */
void set_theta_e_offset_mech(float mech_offset)
{
    vfoc_dt.motor_par.theta_e_offset_mech = mech_offset;
}

float get_theta_e_offset_mech(void)
{
    return vfoc_dt.motor_par.theta_e_offset_mech;
}

/**
 * @brief 设置电角度的值
 * 
 * @param e_value 
 */
void set_vfoc_theta_e_rad(float e_value)
{
    vfoc_dt.motor_par.theta_e = e_value;
}

/**
 * @brief vfoc获取电角度(弧度制)
 * 
 * @param m_angle 机械角度(角度值)
 * @return float 电角度弧度制
 */
float get_vfoc_theta_e_rad(float m_angle)
{
    float elec_deg;/*电角度*/

    /*
     * 机械角度 -> 电角度
        电角度 = 机械角度 * 电机磁极对数
     */
    // elec_deg = (m_angle - vfoc_dt.motor_par.theta_e_offset_mech) * vfoc_dt.motor_par.pole_pairs;
    elec_deg = (m_angle - vfoc_dt.motor_par.theta_e_offset_mech) * vfoc_dt.motor_par.pole_pairs;

    /*
     * 限制到 0~360 度
     */
    while (elec_deg >= 360.0f)
    {
        elec_deg -= 360.0f;
    }

    while (elec_deg < 0.0f)
    {
        elec_deg += 360.0f;
    }

    elec_deg = FOC_DEG_TO_RAD(elec_deg);

    /*设置vik_foc的电角度值，弧度制*/
    set_vfoc_theta_e_rad(elec_deg);
    // vfoc_dt.motor_par.theta_e = elec_deg;

    /*
     * 角度制 -> 弧度制
     */
    return elec_deg;

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
 * @brief 克拉克变换/克拉克正变换
 * 输入三相电流值 ia、ib、ic，计算出 I_alpha、I_beta
 *
 * Clarke 变换公式：
 *
 * I_alpha = 2/3 * (ia - 0.5 * ib - 0.5 * ic)
 * I_beta  = 2/3 * (sqrt(3)/2) * (ib - ic)
 *
 * @param ia 电机A相实际电流值，单位A
 * @param ib 电机B相实际电流值，单位A
 * @param ic 电机C相实际电流值，单位A
 *
 * @return clark_parm_t 返回 I_alpha、I_beta
 */
clark_parm_t clark_tansform(float ia, float ib, float ic)
{
    clark_parm_t clark = {0};

    /*
     * 2 / 3
     */
    const float TWO_BY_THREE = 0.6666666667f;

    /* 二分之更号三
     * sqrt(3) / 2
     */
    const float SQRT3_BY_TWO = 0.8660254038f;

    /*
     * I_alpha 轴和 A 相重合。
     */
    clark.I_alpha = TWO_BY_THREE *
                    (ia - 0.5f * ib - 0.5f * ic);

    /*
     * I_beta 轴比 I_alpha 轴超前 90°。
     */
    clark.I_beta = TWO_BY_THREE *
                   SQRT3_BY_TWO *
                   (ib - ic);

    return clark;
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
 * @brief Park变换 / Park正变换
 *
 * 输入两相静止坐标系电流 I_alpha、I_beta，
 * 结合当前电角度 theta_e_rad，
 * 计算旋转坐标系下的 Id、Iq。
 *
 * 公式：
 * Id =  I_alpha * cos(theta_e) + I_beta * sin(theta_e)
 * Iq = -I_alpha * sin(theta_e) + I_beta * cos(theta_e)
 *
 * @param I_alpha alpha轴电流
 * @param I_beta  beta轴电流
 * @param theta_e_rad 当前电角度，单位：弧度
 *
 * @return park_parm_t 返回 Id、Iq
 */
park_parm_t park_tansform(float I_alpha, float I_beta, float theta_e_rad)
{
    park_parm_t park = {0};

    float sin_theta = sinf(theta_e_rad);
    float cos_theta = cosf(theta_e_rad);

    /*
     * d轴电流：
     * 表示和转子磁场方向重合的电流分量。
     */
    park.Ud = I_alpha * cos_theta + I_beta * sin_theta;

    /*
     * q轴电流：
     * 表示和转子磁场垂直的电流分量。
     * BLDC/PMSM 主要靠 Iq 产生转矩。
     */
    park.Uq = -I_alpha * sin_theta + I_beta * cos_theta;

    return park;
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
 * @brief 判断浮点数是否合法
 *
 * 量产代码里，不建议让 NaN/Inf 继续进入 PWM。
 * 一旦 NaN 进入 duty，后面 compare 值可能异常。
 */
static int vfoc_float_is_valid(float value)
{
    // 是有限正常数 → 返回1；是无穷大/NaN → 返回0
    // 判断浮点数是不是正常的有限数字
    return isfinite(value);
}

/**
 * @brief 量产级 SVPWM：alpha/beta -> 三相 duty
 *
 * 输入：
 *      c_v->I_alpha : alpha 轴电压，单位 V
 *      c_v->I_beta  : beta 轴电压，单位 V
 *      vbus          : 母线电压，单位 V
 *
 * 输出：
 *      duty_out->duty_Ua : U 相 duty，范围 VFOC_PWM_DUTY_MIN ~ VFOC_PWM_DUTY_MAX
 *      duty_out->duty_Ub : V 相 duty
 *      duty_out->duty_Uc : W 相 duty
 *
 * 核心思想：
 *      1. 对 alpha/beta 电压矢量做线性调制区限幅
 *      2. alpha/beta -> 三相相电压 ua/ub/uc
 *      3. 找 max/min
 *      4. 注入零序 offset = -0.5 * (max + min)
 *      5. 转换为 duty
 *
 * 为什么这是工程版：
 *      - 不依赖扇区判断，避免扇区边界抖动 bug
 *      - 直接 alpha/beta 输入，减少中间层
 *      - 自动限制在线性调制区
 *      - 保留 duty 上下限，保护 bootstrap 驱动
 *      - 返回状态码，方便后续故障记录
 */
vfoc_status_e_t vfoc_svpwm_calc_duty_uab(const clark_parm_t *c_v,
                                      float vbus,
                                      pwm_duty_t *duty_out)
{
    float alpha;
    float beta;

    float vref;
    float vref_max;
    float scale;

    float ua;
    float ub;
    float uc;

    float max_v;
    float min_v;
    float zero_offset;

    float duty_half_range;

    vfoc_status_e_t status = VFOC_STATUS_OK;

    /*
     * 先给安全默认值。
     * 如果后面参数错误，至少输出 50% 附近，不会随机乱跳。
     */
    pwm_duty_t duty = {
        .duty_Ua = 0.5f,
        .duty_Ub = 0.5f,
        .duty_Uc = 0.5f,
    };

    if (duty_out == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    *duty_out = duty;

    if (c_v == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    if ((vbus <= VFOC_FLOAT_EPSILON) || (!vfoc_float_is_valid(vbus)))
    {
        return VFOC_STATUS_BAD_VBUS;
    }

    alpha = c_v->I_alpha;
    beta  = c_v->I_beta;

    if ((!vfoc_float_is_valid(alpha)) || (!vfoc_float_is_valid(beta)))
    {
        return VFOC_STATUS_BAD_INPUT;
    }

    /*
     * duty 可用半范围。
     *
     * 理想 duty 是 0.0 ~ 1.0，则半范围是 0.5。
     * 但 FD6287 bootstrap 驱动不建议靠近 0% / 100%，
     * 所以这里用 0.02 ~ 0.98，半范围就是 0.48。
     */
    duty_half_range = 0.5f - VFOC_PWM_DUTY_MIN;

    if ((VFOC_PWM_DUTY_MAX - 0.5f) < duty_half_range)
    {
        duty_half_range = VFOC_PWM_DUTY_MAX - 0.5f;
    }

    if (duty_half_range <= 0.0f)
    {
        return VFOC_STATUS_BAD_INPUT;
    }

    /*
     * SVPWM 线性调制区最大 alpha/beta 电压矢量幅值：
     *
     * 理想情况下：
     *      Vref_max = Vbus / sqrt(3)
     *
     * 考虑 duty_min/duty_max 以后：
     *      Vref_max = 2 / sqrt(3) * duty_half_range * Vbus
     *
     * duty_half_range = 0.5 时：
     *      Vref_max = 0.57735 * Vbus
     *
     * duty_half_range = 0.48 时：
     *      Vref_max ≈ 0.554 * Vbus
     */
    vref = sqrtf((alpha * alpha) + (beta * beta));
    vref_max = FOC_2_DIV_SQRT3 * duty_half_range * vbus;

    /*
     * 超过线性调制区就等比例缩小。
     *
     * 注意：
     * 这里不是粗暴 clamp alpha 或 beta，
     * 而是保持矢量方向不变，只缩小幅值。
     * 这样电压角度不会畸变。
     */
    if ((vref > vref_max) && (vref > VFOC_FLOAT_EPSILON))
    {
        scale = vref_max / vref;
        alpha *= scale;
        beta  *= scale;
        status = VFOC_STATUS_SATURATED;
    }

    /*
     * alpha/beta -> 三相。
     *
     * ua + ub + uc = 0
     */
    ua = alpha;
    ub = (-0.5f * alpha) + (FOC_SQRT3_DIV_2 * beta);
    uc = (-0.5f * alpha) - (FOC_SQRT3_DIV_2 * beta);

    /*
     * 找最大值和最小值。
     */
    max_v = fmaxf(fmaxf(ua, ub), uc);
    min_v = fminf(fminf(ua, ub), uc);

    /*
     * 零序注入。
     *
     * 三相同时加同一个 offset，不改变线电压：
     *      Uab = Ua - Ub
     *      Ubc = Ub - Uc
     *      Uca = Uc - Ua
     *
     * 但是可以把三相波形居中塞进 PWM 可输出范围。
     */
    zero_offset = -0.5f * (max_v + min_v);

    ua += zero_offset;
    ub += zero_offset;
    uc += zero_offset;

    // static uint32_t log_cnt = 0;
    // if ( (log_cnt++)>1000 ) 
    // {
    //     /*
    //     * 这里打印的是 SVPWM 注入零序之后的三相电压。
    //     * 注意：这个 sum 不一定等于 0。
    //     */
    //     ESP_LOGI(TAG,"SVPWM UVW: %.3f,%.3f,%.3f,%.3f,%.3f \r\n",
    //             ua,
    //             ub,
    //             uc,
    //             zero_offset,
    //             ua + ub + uc
    //     );

    //     log_cnt = 0;
    // }
    /*
     * 电压 -> duty。
     *
     * duty = 0.5 + phase_voltage / vbus
     */
    duty.duty_Ua = 0.5f + (ua / vbus);
    duty.duty_Ub = 0.5f + (ub / vbus);
    duty.duty_Uc = 0.5f + (uc / vbus);

    /*
     * 最终 duty 保护。
     * 算法前面已经做过线性区缩放，正常不会碰到这里。
     * 这里是最后一道保险。
     */
    duty.duty_Ua = vfoc_limit(duty.duty_Ua, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty.duty_Ub = vfoc_limit(duty.duty_Ub, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty.duty_Uc = vfoc_limit(duty.duty_Uc, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);

    *duty_out = duty;

    return status;
}


/**
 * @brief 开环电压 FOC + SVPWM
 *
 * 这是替代 vfoc_open_loop_spwm_run() 的量产风格版本。
 *
 * 注意：
 * 这仍然是开环控制，不是完整量产 FOC。
 * 真正量产还需要：
 *      电流采样
 *      电流环 PI
 *      速度环 PI
 *      位置/速度估算或编码器
 *      过流/过压/欠压/堵转/温度保护
 *
 * 但是这个 SVPWM 调制器本身是工程化写法。
 */
void vfoc_open_loop_svpwm_run(float target_rpm,
                              float uq,
                              float vbus,
                              float dt_s)
{
    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_e_t svpwm_status;

    if (dt_s <= 0.0f)
    {
        return;
    }

    /*
     * 1. 更新开环电角度。
     */
    vfoc_update_open_loop_angle(target_rpm, dt_s);

    /*
     * 2. 设置 dq 电压。
     *
     * 开环阶段：
     *      Ud = 0
     *      Uq = 给定测试电压
     */
    vfoc_dt.park_val.Ud = 0.0f;
    vfoc_dt.park_val.Uq = uq;

    /*
     * 3. 逆 Park：
     *      Ud/Uq + theta_e -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. 可选：保存 Ua/Ub/Uc，方便你打印调试。
     *
     * 注意：
     * 真正 SVPWM duty 不依赖这里的 Ua/Ub/Uc。
     * SVPWM 是直接用 alpha/beta 算 duty。
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*
     * 5. SVPWM：
     *      Ualpha/Ubeta -> duty_Ua/duty_Ub/duty_Uc
     */
    svpwm_status = vfoc_svpwm_calc_duty_uab(
                        &l_temp_clark_v,
                        vbus,
                        &vfoc_dt.motor_drv_val.pwm_duty_val
                   );

    /*
     * 量产代码里，不建议 1ms 打一次日志。
     * 这里只在异常时打。
     */
    if ((svpwm_status != VFOC_STATUS_OK) &&
        (svpwm_status != VFOC_STATUS_SATURATED))
    {
        ESP_LOGE(TAG, "SVPWM error, status=%d", (int)svpwm_status);
    }

    /*
     * 如果只是 VFOC_STATUS_SATURATED，说明电压指令过大，
     * 算法已经自动缩放，不需要停机。
     *
     * 后续闭环时，可以把这个状态反馈给电压环/电流环，
     * 做 anti-windup 防积分饱和。
     */
}




/**
 * @brief 开环电压 FOC + SVPWM
 *
 * 这是替代 vfoc_open_loop_spwm_run() 的量产风格版本。
 *
 * 注意：
 * 这仍然是开环控制，不是完整量产 FOC。
 * 真正量产还需要：
 *      电流采样
 *      电流环 PI
 *      速度环 PI
 *      位置/速度估算或编码器
 *      过流/过压/欠压/堵转/温度保护
 *
 * 但是这个 SVPWM 调制器本身是工程化写法。
 */
void vfoc_set_svpwm(float uq,
                    float ud,
                    float vbus)
{

    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_e_t svpwm_status;

    /*
     * 2. 设置 dq 电压。
     *
     * 开环阶段：
     *      Ud = 0
     *      Uq = 给定测试电压
     */
    vfoc_dt.park_val.Uq = uq;
    vfoc_dt.park_val.Ud = ud;

    /*获取电角度弧度制*/
    // vfoc_dt.motor_par.theta_e = get_vfoc_theta_e_rad();

    /*
     * 3. 逆 Park：
     *      输入Ud/Uq + theta_e，输出Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. 可选：保存 Ua/Ub/Uc，方便你打印调试。
     *
     * 注意：
     * 真正 SVPWM duty 不依赖这里的 Ua/Ub/Uc。
     * SVPWM 是直接用 alpha/beta 算 duty。
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*
     * 5. SVPWM：
     *      Ualpha/Ubeta -> duty_Ua/duty_Ub/duty_Uc
     */
    svpwm_status = vfoc_svpwm_calc_duty_uab( &l_temp_clark_v,
                                             vbus,
                                             &vfoc_dt.motor_drv_val.pwm_duty_val
    );

    /*
     * 量产代码里，不建议 1ms 打一次日志。
     * 这里只在异常时打。
     */
    if ((svpwm_status != VFOC_STATUS_OK) &&
        (svpwm_status != VFOC_STATUS_SATURATED))
    {
        ESP_LOGE(TAG, "SVPWM error, status=%d", (int)svpwm_status);
    }

    /*
     * 如果只是 VFOC_STATUS_SATURATED，说明电压指令过大，
     * 算法已经自动缩放，不需要停机。
     *
     * 后续闭环时，可以把这个状态反馈给电压环/电流环，
     * 做 anti-windup 防积分饱和。
     */
}


/**
 * @brief 根据Ua,Ub,Uc 输出三相的PWM值(最大为100%)
 * 
 * @param motor_v 
 * @param vbus 
 * @return pwm_duty_t 
 */
pwm_duty_t vfoc_spwm_calc_duty(const motor_driver_parm_t *motor_v, float vbus)
{
    pwm_duty_t duty = {0};

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



void vfoc_set_spwm( float uq,
                    float ud,
                    float vbus)
{
    clark_parm_t l_temp_clark_v = {0};
    
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
    vfoc_dt.park_val.Ud = ud;

    /*
     * 3. Park逆变换：Id/Iq -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. Clarke逆变换：Ualpha/Ubeta -> Ua/Ub/Uc
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*如果 sum 接近 0，说明逆 Clarke 输出也正常。*/
    // ESP_LOGI(TAG, "UVW: %.3f, %.3f, %.3f, sum=%.3f",
    //     vfoc_dt.motor_drv_val.Ua,
    //     vfoc_dt.motor_drv_val.Ub,
    //     vfoc_dt.motor_drv_val.Uc,
    //     vfoc_dt.motor_drv_val.Ua +
    //     vfoc_dt.motor_drv_val.Ub +
    //     vfoc_dt.motor_drv_val.Uc
    // );

    /*
     * 5. SPWM：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
     */
    vfoc_dt.motor_drv_val.pwm_duty_val =
        vfoc_spwm_calc_duty(&vfoc_dt.motor_drv_val, vbus);

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
     * 3. Park逆变换：Id/Iq -> Ualpha/Ubeta
     */
    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    /*
     * 4. Clarke逆变换：Ualpha/Ubeta -> Ua/Ub/Uc
     */
    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /*如果 sum 接近 0，说明逆 Clarke 输出也正常。*/
    // ESP_LOGI(TAG, "UVW: %.3f, %.3f, %.3f, sum=%.3f",
    //     vfoc_dt.motor_drv_val.Ua,
    //     vfoc_dt.motor_drv_val.Ub,
    //     vfoc_dt.motor_drv_val.Uc,
    //     vfoc_dt.motor_drv_val.Ua +
    //     vfoc_dt.motor_drv_val.Ub +
    //     vfoc_dt.motor_drv_val.Uc
    // );

    /*
     * 5. SPWM：Ua/Ub/Uc -> duty_Ua/duty_Ub/duty_Uc
     */
    vfoc_dt.motor_drv_val.pwm_duty_val =
        vfoc_spwm_calc_duty(&vfoc_dt.motor_drv_val, vbus);
}

pwm_duty_t vfoc_get_pwm_duty(void)
{
    return vfoc_dt.motor_drv_val.pwm_duty_val;
}

void vfoc_set_motor_drv_iq(float uq)
{
    vfoc_dt.motor_drv_val.iq = uq;
}


float vfoc_get_motor_drv_iq(void)
{
    return vfoc_dt.motor_drv_val.iq;
}


/**
 * @brief 斜坡限速函数
 *
 * @details
 * 这个函数的作用是：让 now 不要一下子跳到 target，
 * 而是每次最多只变化 max_step。
 *
 * 举例：
 * now = 0
 * target = 60
 * max_step = 1
 *
 * 每次调用结果：
 * 0 -> 1 -> 2 -> 3 -> ... -> 60
 *
 * 这样可以避免：
 * 1. 目标速度突然变化太大
 * 2. Uq突然变化太大
 * 3. 电机启动猛冲、超调、震动
 * 
 * 比如你 1ms 调用一次，想让目标速度每秒最多增加 80rpm，那每次最大步长就是：

RPM_RAMP_PER_S * dt_s = 80.0f * 0.001f = 0.08rpm

也就是目标速度会这样慢慢爬：

0 -> 0.08 -> 0.16 -> 0.24 -> ... -> 60rpm
 *
 * @param now       当前值，比如当前目标速度、当前Uq
 * @param target    最终想达到的目标值
 * @param max_step  本次调用允许变化的最大步长，必须是正数
 *
 * @return float    限速后的新值
 */
float ramp_float(float now, float target, float max_step)
{
    /*
     * 计算目标值和当前值之间的差值。
     *
     * diff > 0：说明目标值比当前值大，需要往上增加。
     * diff < 0：说明目标值比当前值小，需要往下降低。
     */
    float diff = target - now;

    if (diff > max_step)
    {
        /*
         * 如果差值大于最大允许变化量，
         * 说明这次不能一下子加这么多，只允许最多增加 max_step。
         *
         * 例如：
         * now = 0，target = 60，max_step = 1
         * diff = 60
         * 实际本次只允许 +1
         */
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        /*
         * 如果差值小于 -max_step，
         * 说明目标值比当前值小很多，
         * 这次不能一下子减太多，只允许最多减少 max_step。
         *
         * 例如：
         * now = 60，target = 0，max_step = 1
         * diff = -60
         * 实际本次只允许 -1
         */
        diff = -max_step;
    }

    /*
     * 当前值加上被限制后的变化量。
     *
     * 如果 target 离 now 很远：
     *      每次只靠近 max_step。
     *
     * 如果 target 离 now 很近：
     *      直接到达 target，不会来回震荡。
     */
    return now + diff;
}

float limit_float(float x, float min, float max)
{
    if (x > max)
    {
        return max;
    }

    if (x < min)
    {
        return min;
    }

    return x;
}

/**
 * @brief FOC位置控制角度误差计算
 *        输入当前角度值，和期望角度值
 * 
 * 电机旋转方向：顺时针+正值，逆时针-负值
 * 
 * @param expct_deg 期望角度值
 * @param current_deg 当前角度值
 * @return float 输出的带旋转方向的误差角度值，eg: -45(逆时针旋转四十五度), 90(顺时针旋转90度)
 */
static float angle_error_deg(float expct_deg, float current_deg)
{
    /*误差值 = 期望值-当前值*/
    float err = expct_deg - current_deg;

    while (err > 180.0f)
    {
        err -= 360.0f;
    }

    while (err < -180.0f)
    {
        err += 360.0f;
    }

    return err;
}



/**
 * @brief 位置环
 * 
 */
void vfoc_position_loop(void)
{
    // float now_angle = get_vfoc_theta_m_deg();/* 获取当前机械角度值 */

    // 第三，如果用在位置环角度控制，err_now = exp_v - now_v 暂时不适合处理 0°/360° 跨界。速度环没问题，位置环后面要换成：

    // err_now = angle_error_deg(exp_v, now_v);

    // LIMIT_EXP_MECH_360(exp_angle);
    // err_angle = angle_error_deg( exp_angle , now_angle );/*本次误差值*/
    // now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/
    // err_motor_rpm = exp_motor_rpm - now_motor_rpm;/*本次误差值*/

}

/**
 * @brief 速度环
 * 
 */
void vfoc_speed_loop(void)
{
    // float now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/

}


/**
 * @brief 力矩环
 * 
 */
void vfoc_torque_loop(void)
{
        // LIMIT_EXP_MECH_360(exp_angle);
    // now_angle = get_vfoc_theta_m_deg();/* 获取当前机械角度值 */
    // err_angle = angle_error_deg( exp_angle , now_angle );/*本次误差值*/
    // now_motor_rpm = get_vfoc_mech_rpm();/*获取当前转速*/
    // err_motor_rpm = exp_motor_rpm - now_motor_rpm;/*本次误差值*/
    
}



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
