可以，**你现在先实现 FOC 位置控制 + 力矩大小控制**，最适合走这个版本：

```text
AS5600 机械角度
    ↓
位置 PID
    ↓
输出 Uq
    ↓
限制 Uq 最大值 = 力矩大小限制
    ↓
根据编码器角度做反 Park
    ↓
SVPWM / MCPWM 输出
```

你现在代码里已经有 AS5600 角度读取、`set_vfoc_theta_m(angle)`、MCPWM/SVPWM 相关代码，方向是对的。你的现有 FOC 代码里也已经说明了 `d/q` 坐标里 `q` 通常用于控制转矩，而你当前没有电流采样，所以这里先把 `Uq` 当作“电压型力矩指令”使用。

---

## 先说结论：力矩大小现在怎么控？

真正的力矩控制是：

```text
目标力矩
    ↓
目标 Iq 电流
    ↓
电流 PID
    ↓
输出 Uq
```

但是你现在还没有相电流采样，所以先做简化版：

```text
Uq 越大，电机推力越大
Uq 越小，电机推力越小
```

也就是：

```c
float torque_limit_v = 2.0f;
```

这个 `torque_limit_v` 就是你现在的“力矩大小限制”。

例如：

```text
torque_limit_v = 0.5V   很软，手一掰就动
torque_limit_v = 1.0V   有一点保持力
torque_limit_v = 2.0V   力矩明显变大
torque_limit_v = 3.0V   学习阶段已经偏大，要小心发热
```

---

## 最小版本：位置 PID 直接输出 Uq

这个是你现在最容易跑起来的版本：

```text
目标角度 target_angle_deg
当前角度 current_angle_deg
误差 error_deg
    ↓
位置 PID
    ↓
Uq
    ↓
FOC 输出
```

比如你想让电机保持在 `90°`：

```c
target_angle_deg = 90.0f;
```

电机偏离 90° 后，PID 会输出一个正/负 `Uq`，把电机拉回目标角度。

---

## 1. 先加一个位置控制结构体

可以放到 `vik_foc.h`：

```c
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
} vfoc_position_ctrl_t;
```

再加函数声明：

```c
void vfoc_position_ctrl_init(vfoc_position_ctrl_t *ctrl,
                             float kp,
                             float ki,
                             float kd,
                             float torque_limit_v);

void vfoc_position_set_target(vfoc_position_ctrl_t *ctrl,
                              float target_angle_deg);

void vfoc_position_set_torque_limit(vfoc_position_ctrl_t *ctrl,
                                    float torque_limit_v);

float vfoc_position_ctrl_calc(vfoc_position_ctrl_t *ctrl,
                              float current_angle_deg,
                              float dt_s);
```

---

## 2. 实现位置 PID，输出 Uq

可以新建一个文件，比如：

```text
foc_position.c
```

代码如下：

```c
#include "vik_foc.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "FOC_POS";

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

void vfoc_position_ctrl_init(vfoc_position_ctrl_t *ctrl,
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

void vfoc_position_set_target(vfoc_position_ctrl_t *ctrl,
                              float target_angle_deg)
{
    if (ctrl == NULL)
    {
        return;
    }

    ctrl->target_angle_deg = vfoc_wrap_360_deg(target_angle_deg);
}

void vfoc_position_set_torque_limit(vfoc_position_ctrl_t *ctrl,
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
float vfoc_position_ctrl_calc(vfoc_position_ctrl_t *ctrl,
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
```

---

## 3. 机械角度转电角度

AS5600 读出来的是机械角度。FOC 用的是电角度。

你要这样转：

```c
#define MOTOR_POLE_PAIRS 7
#define FOC_PI 3.14159265358979323846f

float vfoc_mech_deg_to_elec_rad(float mech_deg)
{
    float elec_deg;

    /*
     * 机械角度 -> 电角度
     */
    elec_deg = mech_deg * MOTOR_POLE_PAIRS;

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

    /*
     * 角度制 -> 弧度制
     */
    return elec_deg * FOC_PI / 180.0f;
}
```

这里 `MOTOR_POLE_PAIRS` 要改成你电机真实极对数。你之前代码注释里写过 `pole_pairs = 7`，所以这里先用 7。

---

## 4. FOC 输出时，Id = 0，Uq = 位置环输出

核心逻辑是：

```c
float current_angle_deg = 0.0f;
float theta_e_rad = 0.0f;
float uq = 0.0f;

/* 读取 AS5600 当前机械角度 */
motor_encoder_get_angle(&current_angle_deg);

/* 位置 PID 输出 Uq */
uq = vfoc_position_ctrl_calc(&pos_ctrl, current_angle_deg, dt_s);

/* 机械角度转电角度 */
theta_e_rad = vfoc_mech_deg_to_elec_rad(current_angle_deg);

/* FOC 控制 */
Id = 0;
Iq 或 Uq = uq;
```

注意，你现在如果没有电流采样，这里的 `Iq` 实际不是电流，是 `Uq 电压指令`。

所以你可以先理解成：

```text
Id = 0
Uq = 控制推力大小
```

---

## 5. 你现在应该先做这个接口

建议你在 FOC 输出模块里封装一个函数：

```c
void vfoc_voltage_foc_run(float mech_angle_deg,
                          float uq,
                          float vbus);
```

它内部做：

```text
机械角度 -> 电角度 rad
Ud = 0
Uq = uq
反 Park
反 Clarke
SVPWM
设置 PWM duty
```

伪代码结构如下：

```c
void vfoc_voltage_foc_run(float mech_angle_deg,
                          float uq,
                          float vbus)
{
    float theta_e_rad;

    park_parm_t dq;
    clark_parm_t ab;
    motor_uvw_t uvw;

    theta_e_rad = vfoc_mech_deg_to_elec_rad(mech_angle_deg);

    /*
     * 电压型 FOC：
     * d 轴电压给 0
     * q 轴电压就是力矩指令
     */
    dq.Id = 0.0f;
    dq.Iq = uq;
    dq.theta_e = theta_e_rad;

    /*
     * 反 Park：
     * dq -> alpha/beta
     */
    ab = park_inv_transform(&dq);

    /*
     * SVPWM：
     * alpha/beta -> 三相 PWM
     */
    vfoc_svpwm_output(&ab, vbus);
}
```

你具体的结构体名字可能和这里不一样，但思路就是这个。

你之前代码里已经有 float 版反 Park、反 Clarke、SVPWM 零序注入、PWM duty 转换这些函数，只是很多还在 `app_main()` 文件里，后面建议挪出来变成正式的 FOC 模块。 

---

## 6. 最小控制任务

你可以先写一个任务，100Hz 或 200Hz 先跑，不要一开始就 10kHz。

```c
static vfoc_position_ctrl_t pos_ctrl;

static void foc_position_task(void *arg)
{
    float current_angle_deg = 0.0f;
    float uq = 0.0f;

    const float dt_s = 0.005f;   /* 5ms，200Hz */

    while (1)
    {
        if (motor_encoder_get_angle(&current_angle_deg) == ESP_OK)
        {
            /*
             * 位置 PID 输出 Uq
             */
            uq = vfoc_position_ctrl_calc(&pos_ctrl,
                                         current_angle_deg,
                                         dt_s);

            /*
             * 电压型 FOC 输出
             */
            vfoc_voltage_foc_run(current_angle_deg,
                                 uq,
                                 12.0f);

            ESP_LOGI("FOC_POS",
                     "target=%.2f cur=%.2f err=%.2f uq=%.3f",
                     pos_ctrl.target_angle_deg,
                     current_angle_deg,
                     pos_ctrl.error_deg,
                     uq);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

初始化：

```c
void foc_position_control_start(void)
{
    /*
     * 先用 P 控制，不要一开始就加 I 和 D。
     */
    vfoc_position_ctrl_init(&pos_ctrl,
                            0.03f,   /* kp */
                            0.00f,   /* ki */
                            0.00f,   /* kd */
                            1.0f);   /* torque_limit_v */

    /*
     * 目标位置，比如 90 度
     */
    vfoc_position_set_target(&pos_ctrl, 90.0f);

    /*
     * 创建任务
     */
    xTaskCreatePinnedToCore(foc_position_task,
                            "foc_pos_task",
                            4096,
                            NULL,
                            6,
                            NULL,
                            1);
}
```

---

## 7. 调试参数建议

你先这样调：

```c
kp = 0.01f;
ki = 0.0f;
kd = 0.0f;
torque_limit_v = 0.5f;
```

如果电机拉不动，再加：

```c
torque_limit_v = 1.0f;
```

如果能拉动但回位很慢，加 `kp`：

```c
kp = 0.02f;
kp = 0.03f;
kp = 0.05f;
```

暂时不要加 `ki`。位置控制一开始加积分很容易：

```text
抖动
过冲
来回抽搐
积分越积越大
```

---

## 8. 很关键：要做电角度零点对齐

位置控制想正常，必须解决这个问题：

```text
AS5600 的 0 度
不一定等于
电机转子磁场 d 轴 0 度
```

所以你需要一个偏移：

```c
float elec_zero_offset_deg;
```

完整电角度应该是：

```c
elec_deg = (mech_deg * pole_pairs) + elec_zero_offset_deg;
```

也就是：

```c
float vfoc_mech_deg_to_elec_rad_with_offset(float mech_deg, float elec_zero_offset_deg)
{
    float elec_deg = mech_deg * MOTOR_POLE_PAIRS + elec_zero_offset_deg;

    while (elec_deg >= 360.0f)
    {
        elec_deg -= 360.0f;
    }

    while (elec_deg < 0.0f)
    {
        elec_deg += 360.0f;
    }

    return elec_deg * FOC_PI / 180.0f;
}
```

如果这个偏移没调对，现象会是：

```text
目标 90 度，电机不去 90 度
电机发抖
电机往奇怪方向使劲
力矩很小
电机锁不住
```

---

## 9. 你现在的实现路线

你按这个顺序做，最稳：

```text
第一步：AS5600 角度稳定读取
第二步：用当前角度做电角度，固定 Uq = 0.5V 测试电机是否有力矩
第三步：确认正负 Uq 对应正反方向
第四步：加位置 P 控制
第五步：调 kp 和 torque_limit_v
第六步：再考虑速度环
第七步：最后有电流采样后做真正 Iq 力矩闭环
```

---

## 你现在先别做“三环”

不要一开始搞成：

```text
位置环 -> 速度环 -> 电流环
```

你现在先做：

```text
位置 P 环 -> Uq -> 电压型 FOC
```

这就是最小闭环。

最小版本本质是：

```c
error = target_angle - current_angle;
error = wrap_to_180(error);

uq = kp * error;
uq = limit(uq, -torque_limit_v, torque_limit_v);

vfoc_voltage_foc_run(current_angle, uq, 12.0f);
```

这几行先跑起来，你就能感受到：

```text
torque_limit_v 越大，电机越硬
kp 越大，回到目标位置越快
kp 太大，电机会抖
torque_limit_v 太大，电机会发热
```

这就是你现在要实现的 **FOC 位置控制 + 力矩大小控制**。
