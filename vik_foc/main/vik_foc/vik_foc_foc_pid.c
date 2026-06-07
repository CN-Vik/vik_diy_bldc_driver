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

static const char *TAG = "FOC_PID";

vfoc_pid_t vfoc_pid_dt={0};


/**
 * @brief 浮点限幅函数
 *
 * @param value 输入值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return float 限幅后的值
 */
static float vfoc_pid_clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/**
 * @brief 把角度误差限制到 -180 ~ +180 度
 *
 * 这个函数专门给角度环用。
 *
 * 举例：
 * 目标角度 = 10 度
 * 当前角度 = 350 度
 *
 * 普通误差：
 * error = 10 - 350 = -340 度
 *
 * 但实际电机只需要正向转 20 度即可。
 * 所以角度环不能直接用 target - feedback。
 *
 * 应该把误差限制到 -180 ~ +180 度之间。
 */
static float vfoc_pid_wrap_angle_error_deg(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }

    while (error_deg < -180.0f)
    {
        error_deg += 360.0f;
    }

    return error_deg;
}

/**
 * @brief 初始化 PID 控制器
 *
 * @param pid PID 结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 * @param integral_min 积分最小值
 * @param integral_max 积分最大值
 */
void vfoc_pid_init(vfoc_pid_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_min,
                   float output_max,
                   float integral_min,
                   float integral_max)
{
    if (pid == NULL)
    {
        ESP_LOGE(TAG, "pid is NULL");
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

/**
 * @brief 设置 PID 参数
 *
 * @param pid PID 结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void vfoc_pid_set_param(vfoc_pid_t *pid,
                        float kp,
                        float ki,
                        float kd)
{
    if (pid == NULL)
    {
        ESP_LOGE(TAG, "pid is NULL");
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 清空 PID 历史状态
 *
 * 注意：
 * 这个函数不会清空 kp/ki/kd。
 * 只清空积分、误差、输出。
 *
 * 常见使用场景：
 * 1. 电机停止时
 * 2. 切换模式时
 * 3. 重新使能闭环控制时
 */
void vfoc_pid_reset(vfoc_pid_t *pid)
{
    if (pid == NULL)
    {
        ESP_LOGE(TAG, "pid is NULL");
        return;
    }

    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;

    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 普通 PID 计算
 *
 * @param pid PID 结构体指针
 * @param target 目标值
 * @param feedback 反馈值
 * @param dt_s 控制周期，单位：秒
 * @return float PID 输出
 *
 * PID 公式：
 * error = target - feedback
 *
 * P = kp * error
 * I = ki * integral(error)
 * D = kd * derivative(error)
 *
 * output = P + I + D
 */
float vfoc_pid_calc(vfoc_pid_t *pid,
                    float target,
                    float feedback,
                    float dt_s)
{
    if (pid == NULL)
    {
        ESP_LOGE(TAG, "pid is NULL");
        return 0.0f;
    }

    /*
     * dt_s 是控制周期。
     * 不能是 0，也不能是负数。
     *
     * 比如 1ms 控制周期：
     * dt_s = 0.001f
     */
    if (dt_s <= 0.0f)
    {
        ESP_LOGE(TAG, "dt_s invalid: %.6f", dt_s);
        return pid->output;
    }

    pid->target = target;
    pid->feedback = feedback;

    /*
     * 1. 计算误差
     */
    pid->error = pid->target - pid->feedback;

    /*
     * 2. 积分项累计
     *
     * 为什么要乘 dt_s？
     * 因为积分本质是 error 对时间的累计。
     */
    pid->integral += pid->error * dt_s;

    /*
     * 3. 积分限幅，防止积分越积越大
     *
     * 这叫抗积分饱和。
     */
    pid->integral = vfoc_pid_clamp_float(pid->integral,
                                         pid->integral_min,
                                         pid->integral_max);

    /*
     * 4. 微分项
     *
     * 微分表示误差变化速度。
     */
    pid->derivative = (pid->error - pid->last_error) / dt_s;

    /*
     * 5. PID 输出
     */
    pid->output = (pid->kp * pid->error) +
                  (pid->ki * pid->integral) +
                  (pid->kd * pid->derivative);

    /*
     * 6. 输出限幅
     */
    pid->output = vfoc_pid_clamp_float(pid->output,
                                       pid->output_min,
                                       pid->output_max);

    /*
     * 7. 保存本次误差，给下一次微分使用
     */
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief 角度环 PID 计算
 *
 * @param pid PID 结构体指针
 * @param target_deg 目标角度，单位：度
 * @param feedback_deg 当前反馈角度，单位：度
 * @param dt_s 控制周期，单位：秒
 * @return float PID 输出
 *
 * 这个函数适合位置环。
 *
 * 和普通 PID 的区别：
 * 角度是环形变量。
 * 0 度和 360 度其实是同一个位置。
 *
 * 所以误差不能简单写成：
 * error = target_deg - feedback_deg
 *
 * 而是要把误差转换到 -180 ~ +180 度。
 */
float vfoc_pid_calc_angle_deg(vfoc_pid_t *pid,
                              float target_deg,
                              float feedback_deg,
                              float dt_s)
{
    if (pid == NULL)
    {
        ESP_LOGE(TAG, "pid is NULL");
        return 0.0f;
    }

    if (dt_s <= 0.0f)
    {
        ESP_LOGE(TAG, "dt_s invalid: %.6f", dt_s);
        return pid->output;
    }

    pid->target = target_deg;
    pid->feedback = feedback_deg;

    /*
     * 角度误差：
     * 先普通相减，再处理 0/360 度回绕。
     */
    pid->error = target_deg - feedback_deg;
    pid->error = vfoc_pid_wrap_angle_error_deg(pid->error);

    /*
     * 积分累计
     */
    pid->integral += pid->error * dt_s;

    /*
     * 积分限幅
     */
    pid->integral = vfoc_pid_clamp_float(pid->integral,
                                         pid->integral_min,
                                         pid->integral_max);

    /*
     * 微分项
     */
    pid->derivative = (pid->error - pid->last_error) / dt_s;

    /*
     * PID 输出
     */
    pid->output = (pid->kp * pid->error) +
                  (pid->ki * pid->integral) +
                  (pid->kd * pid->derivative);

    /*
     * 输出限幅
     */
    pid->output = vfoc_pid_clamp_float(pid->output,
                                       pid->output_min,
                                       pid->output_max);

    /*
     * 保存误差
     */
    pid->last_error = pid->error;

    return pid->output;
}


static float vfoc_clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

/**
 * @brief 把角度限制到 0 ~ 360 度
 */
static float vfoc_wrap_360_deg(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/**
 * @brief 把角度误差限制到 -180 ~ +180 度
 *
 * 这个非常重要。
 *
 * 例如：
 * target = 10°
 * current = 350°
 *
 * 普通误差：
 * error = 10 - 350 = -340°
 *
 * 但实际只需要正向转 20°。
 */
static float vfoc_wrap_angle_error_deg(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }

    while (error_deg < -180.0f)
    {
        error_deg += 360.0f;
    }

    return error_deg;
}

void vfoc_position_ctrl_init(vfoc_position_pid_ctrl_t *ctrl,
                             float kp,
                             float ki,
                             float kd,
                             float torque_limit_v)
{
    if (ctrl == NULL)
    {
        ESP_LOGE(TAG, "ctrl is NULL");
        return;
    }

    ctrl->kp = kp;
    ctrl->ki = ki;
    ctrl->kd = kd;

    ctrl->target_angle_deg = 0.0f;
    ctrl->current_angle_deg = 0.0f;

    ctrl->error_deg = 0.0f;
    ctrl->last_error_deg = 0.0f;

    ctrl->integral = 0.0f;
    ctrl->derivative = 0.0f;

    ctrl->uq = 0.0f;

    /*
     * 这个就是你现在的“力矩大小限制”。
     * 没有电流采样时，先用 Uq 电压限幅近似控制力矩。
     */
    ctrl->torque_limit_v = torque_limit_v;

    ctrl->integral_limit = 100.0f;
}

void vfoc_position_set_target(vfoc_position_pid_ctrl_t *ctrl,
                              float target_angle_deg)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->target_angle_deg = vfoc_wrap_360_deg(target_angle_deg);
}

void vfoc_position_set_torque_limit(vfoc_position_pid_ctrl_t *ctrl,
                                    float torque_limit_v)
{
    if (ctrl == NULL)
    {
        return;
    }

    if (torque_limit_v < 0.0f)
    {
        torque_limit_v = -torque_limit_v;
    }

    ctrl->torque_limit_v = torque_limit_v;
}

/**
 * @brief 位置控制计算
 *
 * @param ctrl 位置控制器
 * @param current_angle_deg 当前机械角度，0~360 度
 * @param dt_s 控制周期，单位秒
 * @return float 输出 Uq，单位可以理解成 V
 */
float vfoc_position_ctrl_calc(vfoc_position_pid_ctrl_t *ctrl,
                              float current_angle_deg,
                              float dt_s)
{
    if (ctrl == NULL)
    {
        return 0.0f;
    }

    if (dt_s <= 0.0f)
    {
        return ctrl->uq;
    }

    ctrl->current_angle_deg = vfoc_wrap_360_deg(current_angle_deg);

    /*
     * 1. 计算角度误差
     */
    ctrl->error_deg = ctrl->target_angle_deg - ctrl->current_angle_deg;

    /*
     * 2. 处理 0/360 度回绕
     */
    ctrl->error_deg = vfoc_wrap_angle_error_deg(ctrl->error_deg);

    /*
     * 3. 积分
     */
    ctrl->integral += ctrl->error_deg * dt_s;

    ctrl->integral = vfoc_clamp_float(ctrl->integral,
                                      -ctrl->integral_limit,
                                      ctrl->integral_limit);

    /*
     * 4. 微分
     */
    ctrl->derivative = (ctrl->error_deg - ctrl->last_error_deg) / dt_s;

    /*
     * 5. PID 输出 Uq
     */
    ctrl->uq = ctrl->kp * ctrl->error_deg +
               ctrl->ki * ctrl->integral +
               ctrl->kd * ctrl->derivative;

    /*
     * 6. Uq 限幅
     *
     * 这个限幅就是你的力矩大小控制。
     */
    ctrl->uq = vfoc_clamp_float(ctrl->uq,
                                -ctrl->torque_limit_v,
                                ctrl->torque_limit_v);

    ctrl->last_error_deg = ctrl->error_deg;

    return ctrl->uq;
}