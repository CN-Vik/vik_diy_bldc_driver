你这个最新 LOG 里，问题已经很明确了：**电机能转了，但控制环还是在“长期饱和 + 实时性被 LOG 打爆 + 速度反馈跳变”状态，所以会偶尔卡顿、震动大。**

最关键证据有三个：

1. `uq` 限幅前还是几十伏、几百伏，甚至出现 `-580.65`、`-1430`、`-1808` 这种离谱尖峰；限幅后一直卡在 `-2.20V`。这说明速度环实际一直在顶满输出。
2. LOG 里出现 `task_wdt`，而且当前运行任务是 `CPU 1: m0_foc_task`，说明你的 FOC 任务把 CPU1 卡住了，偶尔卡顿和这个高度相关。
3. 代码里还在 while 每圈 `ESP_LOGI(" uq: %.2f")`，而且 `ki = 0.06f`、`kp = 0.02f`、目标速度直接 `100rpm`，这几个组合起来一定容易抖。

---

## 先给结论：你现在必须改这 5 件事

### 1. 彻底删除每圈打印

你现在这个必须删：

```c
ESP_LOGI(TAG,
        " uq: %.2f\r\n",
        uq
);
```

FOC 控制循环里不能 1ms 打一次 LOG。你现在实际 `dt_s` 已经到 `0.00408s` 左右，说明 1ms 控制环被拖成了 4ms 左右。这个会直接导致卡顿、WDT、速度环不稳定。

---

### 2. 速度环不要每 1ms 算一次，改成 10ms 算一次

FOC 输出可以 1ms 更新，但速度 PI 建议 10ms 更新一次。
你用的是角度差分速度，1ms 算速度太抖。

---

### 3. `ki = 0.06f` 太大，先降 20～60 倍

现在先用：

```c
kp = 0.006f;
ki = 0.0015f;
kd = 0.0f;
```

你现在 `ki = 0.06f`，而且 `ki_err_sum += err_motor_rpm;` 没有限幅，所以积分越积越大，`uq` 限幅前会一路从 `-30`、`-40`、`-100` 继续涨上去。

---

### 4. 积分必须防饱和

现在你是：

```c
ki_err_sum += err_motor_rpm;
```

这个写法不推荐。应该改成：

```c
ki_err_sum += err_motor_rpm * dt_s;
ki_err_sum = limit_float(ki_err_sum, -100.0f, 100.0f);
```

并且当 `Uq` 已经饱和时，不要继续疯狂积分。

---

### 5. 目标速度要加斜坡，不能一上来就是 100rpm

你现在：

```c
float exp_motor_rpm = 100.0f;
```

启动瞬间就是 100rpm 阶跃。低速 FOC 闭环阶段很容易抖。

应该变成：

```text
0rpm -> 慢慢爬到 100rpm
```

---

# 直接替换你的速度环 while 主体

你先别继续调原来的 PID 了。把 `m0_foc_control_task()` 里面的控制部分改成下面这个版本。

参数先用这组：

```c
#define SPEED_KP               0.006f
#define SPEED_KI               0.0015f
#define SPEED_KD               0.0f

#define SPEED_LOOP_DIV         10      // 1ms进来一次，10次算一次速度环，也就是100Hz
#define SPEED_UQ_LIMIT         1.2f    // 先别用2.2V，太猛
#define SPEED_I_LIMIT          0.45f   // 积分最多贡献±0.45V
#define SPEED_DEADBAND_RPM     3.0f

#define TARGET_RPM_RAMP_PER_S  150.0f  // 目标速度每秒最多变化150rpm
#define UQ_RAMP_PER_S          15.0f   // Uq每秒最多变化15V
#define RPM_LPF_TAU_S          0.05f   // 速度滤波时间常数50ms
```

新增这个斜坡函数：

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

然后把你的 `m0_foc_control_task()` 改成这种结构：

```c
static void m0_foc_control_task(void *arg)
{
    pwm_duty_t pwm_duty;

    float uq_out = 0.0f;           // 最终输出Uq
    float uq_target = 0.0f;        // 速度环计算出来的目标Uq

    float exp_motor_rpm_cmd = 100.0f;   // 最终期望速度
    float exp_motor_rpm_ramp = 0.0f;    // 斜坡后的期望速度

    float rpm_raw = 0.0f;          // 原始速度
    float rpm_filt = 0.0f;         // 滤波后的速度
    bool rpm_filt_init = false;

    float err_motor_rpm = 0.0f;

    float speed_integral = 0.0f;   // 速度积分，单位 rpm*s
    float p_out = 0.0f;
    float i_out = 0.0f;

    uint32_t speed_loop_cnt = 0;
    uint32_t log_cnt = 0;

    uint64_t stamp_time_us = 0;
    uint64_t last_stamp_time_us = esp_timer_get_time();

    float dt_s = M0_FOC_DT_S;
    float speed_dt_s = M0_FOC_DT_S * SPEED_LOOP_DIV;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        stamp_time_us = esp_timer_get_time();
        dt_s = (float)(stamp_time_us - last_stamp_time_us) / 1000000.0f;
        last_stamp_time_us = stamp_time_us;

        /*
         * 防止串口打印、调试、系统卡顿导致 dt_s 异常。
         * 正常应该接近 0.001s。
         */
        if ((dt_s <= 0.0f) || (dt_s > 0.01f))
        {
            dt_s = M0_FOC_DT_S;
        }

        /*
         * 1. 读取原始速度
         */
        rpm_raw = get_vfoc_mech_rpm();

        /*
         * 2. 速度低通滤波
         */
        if (rpm_filt_init == false)
        {
            rpm_filt = rpm_raw;
            rpm_filt_init = true;
        }
        else
        {
            float alpha = dt_s / (RPM_LPF_TAU_S + dt_s);
            rpm_filt += alpha * (rpm_raw - rpm_filt);
        }

        /*
         * 3. 目标速度斜坡
         */
        exp_motor_rpm_ramp = ramp_float(
            exp_motor_rpm_ramp,
            exp_motor_rpm_cmd,
            TARGET_RPM_RAMP_PER_S * dt_s
        );

        /*
         * 4. 速度PI环：不要1ms算一次，10ms算一次
         */
        speed_loop_cnt++;
        if (speed_loop_cnt >= SPEED_LOOP_DIV)
        {
            speed_loop_cnt = 0;

            err_motor_rpm = exp_motor_rpm_ramp - rpm_filt;

            /*
             * 4.1 速度死区
             */
            if (fabsf(err_motor_rpm) < SPEED_DEADBAND_RPM)
            {
                err_motor_rpm = 0.0f;

                /*
                 * 误差很小时，积分慢慢释放。
                 */
                speed_integral *= 0.98f;
            }
            else
            {
                /*
                 * 4.2 正确积分：误差 * 时间
                 */
                speed_integral += err_motor_rpm * speed_dt_s;
            }

            /*
             * 4.3 P项
             */
            p_out = SPEED_KP * err_motor_rpm;

            /*
             * 4.4 I项
             */
            i_out = SPEED_KI * speed_integral;

            /*
             * 4.5 限制积分输出，而不是让积分无限长大
             */
            i_out = limit_float(i_out, -SPEED_I_LIMIT, SPEED_I_LIMIT);

            if (SPEED_KI > 0.000001f)
            {
                speed_integral = i_out / SPEED_KI;
            }

            /*
             * 4.6 速度环输出目标Uq
             */
            uq_target = p_out + i_out;

            /*
             * 4.7 限制速度环最大输出
             */
            uq_target = limit_float(uq_target, -SPEED_UQ_LIMIT, SPEED_UQ_LIMIT);

            /*
             * 注意：
             * 你原来代码里用了 uq *= -1。
             * 如果你现在这个方向是对的，就保留这个负号。
             * 如果方向反了，改这里。
             */
            uq_target *= -1.0f;
        }

        /*
         * 5. Uq输出斜坡限制
         *    防止Uq突然跳变导致电机一顿一顿。
         */
        uq_out = ramp_float(
            uq_out,
            uq_target,
            UQ_RAMP_PER_S * dt_s
        );

        /*
         * 6. 输出SVPWM
         */
        vfoc_set_svpwm(uq_out,
                       M0_TEST_UD,
                       MOTOR_DRV_VBUS);

        pwm_duty = vfoc_get_pwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);

        /*
         * 7. 低频打印，禁止每圈打印
         *    200ms打印一次即可。
         */
        log_cnt++;
        if (log_cnt >= 200)
        {
            log_cnt = 0;

            ESP_LOGI(TAG,
                     "cmd:%.1f,ramp:%.1f,rpm_raw:%.1f,rpm_filt:%.1f,err:%.1f,uq:%.2f,p:%.2f,i:%.2f,dt:%.5f",
                     exp_motor_rpm_cmd,
                     exp_motor_rpm_ramp,
                     rpm_raw,
                     rpm_filt,
                     err_motor_rpm,
                     uq_out,
                     p_out,
                     i_out,
                     dt_s);
        }
    }
}
```

---

## 为什么这个版本能解决你现在的问题？

### 第一，`uq` 不会再变成几十伏、几百伏

你现在 LOG 里限幅前 `uq` 会从 `-30` 慢慢涨到 `-100`，还会出现 `-580` 这种尖峰。

新版本里：

```c
uq_target = limit_float(uq_target, -SPEED_UQ_LIMIT, SPEED_UQ_LIMIT);
```

而且积分输出也限制：

```c
i_out = limit_float(i_out, -SPEED_I_LIMIT, SPEED_I_LIMIT);
```

所以控制器内部不会再疯狂积累。

---

### 第二，电机不会突然一脚猛踹

新版本多了 Uq 斜坡：

```c
uq_out = ramp_float(uq_out, uq_target, UQ_RAMP_PER_S * dt_s);
```

这能明显减少：

```text
偶尔卡一下
突然震一下
Uq阶跃导致电机顿挫
```

---

### 第三，速度反馈更平滑

现在你速度可能在跳，比如 LOG 里有一段是：

```text
目标 100rpm
实际 167.56rpm
误差 -67.56rpm
```

这个速度反馈一跳，速度环就会突然反打，所以震动大。

新版本加了：

```c
rpm_filt += alpha * (rpm_raw - rpm_filt);
```

速度环用的是 `rpm_filt`，不是直接用 `rpm_raw`。

---

### 第四，实时性会恢复

你的 `task_wdt` 必须先解决。现在 WDT 明确报了 `IDLE1` 没机会运行，CPU1 当前就是 `m0_foc_task`。

删掉每圈 `ESP_LOGI` 后，控制环执行时间会大幅下降，卡顿会明显减少。

---

# 参数怎么调

你先按这个顺序调，不要乱调三项 PID。

## 第 1 步：先只用 P

```c
#define SPEED_KP  0.004f
#define SPEED_KI  0.0f
```

目标速度先用：

```c
float exp_motor_rpm_cmd = 50.0f;
```

看现象：

```text
能稳定转，但是达不到50rpm：正常
声音变小：正确
偶尔卡顿减少：正确
```

---

## 第 2 步：慢慢加 Kp

```text
0.004 -> 0.006 -> 0.008 -> 0.010
```

如果出现：

```text
速度一快一慢
电机明显来回顶
震动变大
```

说明 Kp 大了，退回上一档。

---

## 第 3 步：再加一点 Ki

```c
#define SPEED_KI  0.001f
```

再试：

```text
0.001 -> 0.0015 -> 0.002
```

不要再用 `0.06`，这个对你现在太大了。

---

# 还有一个非常重要的验证

你现在最新 LOG 里，有时显示：

```text
目标100，实际0，误差100
```

有时又显示：

```text
目标100，实际167.56，误差-67.56
```

这说明速度反馈可能不平滑，甚至可能有跳变。

你要单独打印这几个值，200ms 一次：

```c
ESP_LOGI(TAG,
         "angle:%.2f,rpm_raw:%.2f,rpm_filt:%.2f,uq:%.2f",
         get_vfoc_theta_m_deg(),
         rpm_raw,
         rpm_filt,
         uq_out);
```

正常应该看到：

```text
rpm_raw 有点抖
rpm_filt 平滑很多
uq_out 缓慢变化
```

如果 `rpm_raw` 经常从 `0` 突然跳到 `160`、再跳回 `0`，那你还要继续优化 `get_vfoc_mech_rpm()`，否则速度环永远会抖。

---

## 你当前代码的优化优先级

按这个顺序改：

```text
1. 删掉 while 每圈 ESP_LOGI
2. 速度环改成 10ms 算一次
3. ki 从 0.06 降到 0.001~0.002
4. 加积分限幅
5. 加目标速度斜坡
6. 加 Uq 输出斜坡
7. 加 rpm 低通滤波
8. UQ_LIMIT 先从 2.2 降到 1.2
```

你现在不是 SVPWM 算法错，主要是**速度闭环调得太猛，而且实时性被打印打爆了**。先按上面这版改，卡顿和震动会明显下降。
