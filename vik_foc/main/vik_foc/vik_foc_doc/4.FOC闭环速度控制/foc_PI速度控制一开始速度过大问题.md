你这个现象**不是玄学**，我给你按“小白能懂”的方式讲清楚：

你设的是：

```c
目标速度 = 60 rpm
UQ_LIMIT = 3.2V
kp = 0.02
ki = 0.06
kd = 0
```

代码里也是这么写的，最后 `uq` 会被限制到 `±3.2V`。

---

# 你现在发生了什么？

看你启动阶段 LOG：

```text
uq: -3.20, 目标60, 实际0, 误差60
uq: -3.20, 目标60, 实际0, 误差60
uq: -3.20, 目标60, 实际0, 误差60
...
uq: -3.20, 目标60, 实际204.86, 误差-144.86
uq: -3.20, 目标60, 实际247.42, 误差-187.42
```

意思就是：

```text
一上电时，速度检测一直是 0
控制器一看：目标 60，实际 0，差 60
于是疯狂给 Uq
Uq 一直顶到最大 -3.20V
电机被最大力矩猛推
等电机真的转起来/速度检测出来后
速度已经冲到 200~250rpm 了
```

你图里的红色实际速度，先从 0 突然冲到 240 多 rpm，就是这个原因。

---

# 最核心原因：你一启动就在“满油门冲”

可以把 `Uq` 想成油门。

你想让车跑 60km/h，但程序一启动发现车速是 0，于是直接：

```text
油门踩到底
油门踩到底
油门踩到底
油门踩到底
```

等车真的动起来的时候，速度已经冲过 60，跑到 240 了。

这就叫：**启动阶段 Uq 饱和 + 积分累积 + 没有速度斜坡。**

---

# 还有一个隐藏大 BUG：第一次 dt_s 是错的

你代码里：

```c
uint64_t last_stamp_time_us = 0;
```

然后一进循环：

```c
stamp_time_us = esp_timer_get_time();
dt_s = (stamp_time_us - last_stamp_time_us) / 1000000.0f;
```

问题是 `last_stamp_time_us` 初始是 0。

假设任务启动时，系统已经运行了 1.6 秒，那么第一次：

```text
dt_s = 1.6 秒
```

不是你以为的：

```text
dt_s = 0.001 秒
```

然后你又做积分：

```c
ki_err_sum += err_motor_rpm * dt_s;
```

如果误差是 60rpm，第一次积分直接变成：

```text
ki_err_sum += 60 × 1.6 = 96
```

I 项输出：

```text
I = 0.06 × 96 = 5.76V
```

光积分项就已经超过 `UQ_LIMIT = 3.2V` 了。

所以你的电机一上电就直接 `uq = -3.20V`，不是偶然，是代码决定的。

---

# 第二个原因：积分已经憋了一肚子力

你启动时实际速度一直是 0，误差一直是 60。

你的积分一直在加：

```c
ki_err_sum += err_motor_rpm * dt_s;
```

假设它持续 2 秒：

```text
ki_err_sum += 60 × 2 = 120
```

I 项输出：

```text
I = 0.06 × 120 = 7.2V
```

但是你的 Uq 最大只有 3.2V，所以实际输出被卡住：

```text
Uq = -3.2V
```

表面上看 Uq 只有 -3.2V，但积分内部已经憋到了 7V、8V。

等速度冲到 240rpm 时，误差已经变成负数：

```text
err = 60 - 240 = -180
```

但是积分还很大，所以控制器不会马上收力，它还会继续推一会儿。
这就是为啥你看到速度冲很高，然后慢慢降下来。

---

# 第三个原因：你的目标速度是阶跃，不是斜坡

你现在是：

```c
float exp_motor_rpm = 60.0f;
```

一上电目标立刻是 60rpm。

对电机来说，这是一个突然命令：

```text
0 rpm → 60 rpm
```

速度闭环最怕这种突然阶跃，尤其你现在没有电流环，速度环直接输出 Uq。

正确做法应该是：

```text
0 → 5 → 10 → 15 → 20 ... → 60rpm
```

让目标速度慢慢爬上去。

---

# 所以你这个问题不是一个点，是 4 个点叠加

```text
1. 第一次 dt_s 错了，积分一上来就爆
2. 目标速度直接从 0 跳到 60
3. 没有积分限幅，积分憋太大
4. Uq 没有斜坡，一下子给到 -3.2V
```

所以才会：

```text
启动 Uq 直接 -3.20V
实际速度先为 0
突然冲到 240rpm
然后慢慢回到 60rpm 附近
```

---

# 你现在先按这个顺序改

## 1. 修复 dt_s 第一次异常

把这个：

```c
uint64_t last_stamp_time_us = 0;
```

改成：

```c
uint64_t last_stamp_time_us = esp_timer_get_time();
```

然后 while 里加保护：

```c
stamp_time_us = esp_timer_get_time();
dt_s = (float)(stamp_time_us - last_stamp_time_us) / 1000000.0f;
last_stamp_time_us = stamp_time_us;

/* 防止第一次、调试卡顿、系统异常导致 dt_s 离谱 */
if ((dt_s <= 0.0f) || (dt_s > 0.01f))
{
    dt_s = M0_FOC_DT_S;
}
```

这一步非常重要。

---

## 2. 加目标速度斜坡

不要直接用 `exp_motor_rpm = 60` 参与计算。

加一个斜坡目标：

```c
float exp_motor_rpm_cmd = 60.0f;   /* 最终目标速度 */
float exp_motor_rpm_ramp = 0.0f;   /* 慢慢爬升后的目标速度 */
```

加这个函数：

```c
static float ramp_float(float now, float target, float max_step)
{
    float diff = target - now;

    if (diff > max_step)
    {
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        diff = -max_step;
    }

    return now + diff;
}
```

while 里：

```c
#define RPM_RAMP_PER_S  80.0f

exp_motor_rpm_ramp = ramp_float(
    exp_motor_rpm_ramp,
    exp_motor_rpm_cmd,
    RPM_RAMP_PER_S * dt_s
);
```

然后误差用这个：

```c
err_motor_rpm = exp_motor_rpm_ramp - now_motor_rpm;
```

这样启动时不是直接追 60，而是慢慢追。

---

## 3. 加 Uq 斜坡

你现在 Uq 可以瞬间从 0 变成 -3.2V，这太猛了。

加：

```c
float uq_target = 0.0f;
float uq_out = 0.0f;

#define UQ_RAMP_PER_S  6.0f
```

算完 PID 后，不要直接输出：

```c
uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
```

而是：

```c
uq_target = limit_float(uq_target, -UQ_LIMIT, UQ_LIMIT);

uq_out = ramp_float(
    uq_out,
    uq_target,
    UQ_RAMP_PER_S * dt_s
);
```

最后输出：

```c
vfoc_set_svpwm(uq_out, M0_TEST_UD, MOTOR_DRV_VBUS);
```

这样 Uq 会慢慢增加，不会一脚踹出去。

---

## 4. 积分必须限幅

你现在这个现象就是积分风up的典型表现，所以必须加。

简单写法：

```c
#define SPEED_I_OUT_LIMIT  0.5f
```

然后：

```c
float p_out = kp * err_motor_rpm;

float i_out = ki * ki_err_sum;
i_out = limit_float(i_out, -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);

/* 反推积分，防止 ki_err_sum 内部继续无限变大 */
if (ki > 0.000001f)
{
    ki_err_sum = i_out / ki;
}

uq_target = p_out + i_out;
```

`SPEED_I_OUT_LIMIT = 0.5f` 的意思是：

```text
积分项最多只能贡献 ±0.5V
```

这样它不会憋出 7V、8V 的隐藏力。

---

# 给你一版可以直接替换的核心控制逻辑

你先用这个，不要加 D：

```c
#define SPEED_KP              0.02f
#define SPEED_KI              0.02f
#define SPEED_I_OUT_LIMIT     0.5f

#define RPM_RAMP_PER_S        80.0f
#define UQ_RAMP_PER_S         6.0f

/* 60rpm 测试阶段，UQ_LIMIT 不要太大 */
#define SPEED_UQ_LIMIT        1.2f

static float ramp_float(float now, float target, float max_step)
{
    float diff = target - now;

    if (diff > max_step)
    {
        diff = max_step;
    }
    else if (diff < -max_step)
    {
        diff = -max_step;
    }

    return now + diff;
}
```

在 `m0_foc_control_task()` 里加这些变量：

```c
float exp_motor_rpm_cmd = 60.0f;
float exp_motor_rpm_ramp = 0.0f;

float uq_target = 0.0f;
float uq_out = 0.0f;

float p_out = 0.0f;
float i_out = 0.0f;
float ki_err_sum = 0.0f;

uint64_t last_stamp_time_us = esp_timer_get_time();
```

while 里面核心逻辑改成：

```c
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

stamp_time_us = esp_timer_get_time();
dt_s = (float)(stamp_time_us - last_stamp_time_us) / 1000000.0f;
last_stamp_time_us = stamp_time_us;

if ((dt_s <= 0.0f) || (dt_s > 0.01f))
{
    dt_s = M0_FOC_DT_S;
}

/* 1. 读取实际速度 */
now_motor_rpm = get_vfoc_mech_rpm();

/* 2. 目标速度斜坡 */
exp_motor_rpm_ramp = ramp_float(
    exp_motor_rpm_ramp,
    exp_motor_rpm_cmd,
    RPM_RAMP_PER_S * dt_s
);

/* 3. 速度误差 */
err_motor_rpm = exp_motor_rpm_ramp - now_motor_rpm;

/* 4. 死区要放在积分之前 */
if (fabsf(err_motor_rpm) < SPEED_DEADBAND_RPM)
{
    err_motor_rpm = 0.0f;

    /* 误差很小时，积分慢慢释放 */
    ki_err_sum *= 0.98f;
}
else
{
    ki_err_sum += err_motor_rpm * dt_s;
}

/* 5. PI控制 */
p_out = SPEED_KP * err_motor_rpm;

i_out = SPEED_KI * ki_err_sum;
i_out = limit_float(i_out, -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);

if (SPEED_KI > 0.000001f)
{
    ki_err_sum = i_out / SPEED_KI;
}

uq_target = p_out + i_out;

/* 6. 方向修正，保留你原来的负号 */
uq_target *= -1.0f;

/* 7. Uq限幅 */
uq_target = limit_float(uq_target, -SPEED_UQ_LIMIT, SPEED_UQ_LIMIT);

/* 8. Uq斜坡，防止启动猛冲 */
uq_out = ramp_float(
    uq_out,
    uq_target,
    UQ_RAMP_PER_S * dt_s
);

/* 9. 输出 */
vfoc_set_svpwm(uq_out,
               M0_TEST_UD,
               MOTOR_DRV_VBUS);

pwm_duty = vfoc_get_pwm_duty();

m0_fd6287_set_duty(pwm_duty.duty_Ua,
                   pwm_duty.duty_Ub,
                   pwm_duty.duty_Uc);
```

打印建议改成：

```c
if (++log_cnt >= 100)
{
    log_cnt = 0;

    ESP_LOGI(TAG,
             "uq:%.2f,cmd:%.2f,ramp:%.2f,rpm:%.2f,err:%.2f,p:%.2f,i:%.2f,dt:%.5f",
             uq_out,
             exp_motor_rpm_cmd,
             exp_motor_rpm_ramp,
             now_motor_rpm,
             err_motor_rpm,
             p_out,
             i_out,
             dt_s);
}
```

---

# 为什么我建议 60rpm 时先把 UQ_LIMIT 降到 1.2V？

你现在目标才 60rpm，但 Uq 允许到 3.2V。
这就像你想让车慢慢挪一下，但油门最大允许踩很深。

你的 LOG 已经证明：

```text
Uq = -3.2V 时，电机会冲到 240rpm
```

所以 60rpm 阶段根本不需要 3.2V。

先用：

```c
#define SPEED_UQ_LIMIT  1.2f
```

如果启动不起来，再加到：

```c
1.5f
```

如果还不行，再：

```c
2.0f
```

不要一开始就 3.2V。

---

# 最后用一句话总结

你现在启动超调到 240rpm，是因为：

```text
程序一上来看到实际速度是0，
就用最大 Uq = -3.2V 猛推电机，
同时积分还在不断憋力，
等电机转起来时已经冲过头了。
```

解决办法不是加 Kd，而是：

```text
修 dt_s 第一次异常
目标速度加斜坡
Uq 输出加斜坡
积分限幅
低速时降低 Uq 最大值
```

你先把 `SPEED_UQ_LIMIT` 设成 `1.2V`，目标速度用斜坡爬到 60rpm，启动冲到 240rpm 这个问题会明显改善。
