可以。你现在的 FOC 工程是 **float 版本**，前面代码里也是自己写 float 的 Park、Clarke、SVPWM，不再用 IQmath，所以这个 `foc_pid.c` 我也按 `float` 实现。你当前编码器任务里已经有 `set_vfoc_theta_m(angle)` 这种把角度喂给 FOC 的思路，所以后面 PID 可以接速度环/位置环。 

下面这版适合你当前阶段：**通用 PID + 角度环专用 PID**。

---

## 1. 先在 `vik_foc.h` 里加这个结构体和函数声明

```c id="6fx8cu"
#ifndef __VIK_FOC_H__
#define __VIK_FOC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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


/**
 * @brief 初始化 PID 控制器
 */
void vfoc_pid_init(vfoc_pid_t *pid,
                   float kp,
                   float ki,
                   float kd,
                   float output_min,
                   float output_max,
                   float integral_min,
                   float integral_max);

/**
 * @brief 设置 PID 参数
 */
void vfoc_pid_set_param(vfoc_pid_t *pid,
                        float kp,
                        float ki,
                        float kd);

/**
 * @brief 清空 PID 历史状态
 */
void vfoc_pid_reset(vfoc_pid_t *pid);

/**
 * @brief 普通 PID 计算
 */
float vfoc_pid_calc(vfoc_pid_t *pid,
                    float target,
                    float feedback,
                    float dt_s);

/**
 * @brief 角度环 PID 计算，单位：degree
 */
float vfoc_pid_calc_angle_deg(vfoc_pid_t *pid,
                              float target_deg,
                              float feedback_deg,
                              float dt_s);

#ifdef __cplusplus
}
#endif

#endif
```

---

## 2. `foc_pid.c` 完整实现

你这个文件就写成下面这样：

```c id="s7hmhu"
/**
 * @file foc_pid.c
 * @author vik (ufo281@outlook.com)
 * @brief FOC PID 控制器实现
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 */

#include "vik_foc.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "FOC_PID";

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
```

---

## 3. 怎么用？先从速度环开始比较简单

比如你后面要做：

```text id="cnokqv"
目标转速 rpm
    ↓
速度 PID
    ↓
输出 Uq
    ↓
FOC / SVPWM 输出
```

可以这样：

```c id="r6lit4"
static vfoc_pid_t speed_pid;

void foc_pid_test_init(void)
{
    /*
     * 先随便给一组保守参数，后面慢慢调。
     *
     * output_min/output_max：
     * 这里假设 PID 输出的是 Uq 电压。
     * 比如你 12V 母线，学习阶段可以先限制在 -3V ~ +3V。
     *
     * integral_min/integral_max：
     * 积分不要给太大，先限制小一点。
     */
    vfoc_pid_init(&speed_pid,
                  0.01f,     /* kp */
                  0.00f,     /* ki */
                  0.00f,     /* kd */
                  -3.0f,     /* output_min */
                  3.0f,      /* output_max */
                  -100.0f,   /* integral_min */
                  100.0f);   /* integral_max */
}
```

然后周期调用：

```c id="xzd36s"
float target_rpm = 300.0f;
float current_rpm = 0.0f;
float uq = 0.0f;
float dt_s = 0.001f;   /* 1ms */

uq = vfoc_pid_calc(&speed_pid, target_rpm, current_rpm, dt_s);
```

---

## 4. 如果做角度位置环，用这个

比如目标位置是 `90°`，AS5600 当前角度是 `350°`：

```c id="yyg8n1"
static vfoc_pid_t angle_pid;

void angle_pid_init(void)
{
    vfoc_pid_init(&angle_pid,
                  0.02f,     /* kp */
                  0.00f,     /* ki */
                  0.00f,     /* kd */
                  -200.0f,   /* output_min: 目标速度下限 rpm */
                  200.0f,    /* output_max: 目标速度上限 rpm */
                  -100.0f,
                  100.0f);
}
```

周期调用：

```c id="jsxio6"
float target_angle_deg = 90.0f;
float current_angle_deg = 0.0f;
float target_rpm = 0.0f;
float dt_s = 0.001f;

target_rpm = vfoc_pid_calc_angle_deg(&angle_pid,
                                     target_angle_deg,
                                     current_angle_deg,
                                     dt_s);
```

这个意思是：

```text id="4g9coz"
角度环 PID 输出目标速度
速度环 PID 输出 Uq
Uq 进入 FOC
```

也就是经典的：

```text id="h88r6d"
位置环 → 速度环 → 电流/电压环 → SVPWM
```

---

## 5. 你现在阶段建议先这样用

你现在还在 AS5600 + 开环/半闭环阶段，建议顺序是：

```text id="j8lwf5"
第一步：只读 AS5600 角度，确认稳定
第二步：根据角度差计算速度 rpm
第三步：加速度 PID，输出 Uq
第四步：再做位置 PID
```

先不要一上来三环全开。你现在这个 `foc_pid.c` 可以先放进去，第一阶段只用：

```c id="jwzsdu"
vfoc_pid_init()
vfoc_pid_calc()
```

等速度闭环跑稳了，再用：

```c id="uy4o7w"
vfoc_pid_calc_angle_deg()
```
