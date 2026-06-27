正常。**用了 SVPWM 后震动变大，优先怀疑不是“SVPWM 本身错”，而是你的开环参数和控制方式不适合。**

尤其你现在是：

```c
#define M0_TEST_RPM    300.0f
#define M0_TEST_UQ     2.0f
#define M0_FOC_CTRL_FREQ_HZ 1000
```

也就是：**一上电就让电机按 300RPM 开环转，Uq 给 2V，控制周期只有 1kHz**。你代码里自己也写了，300~800RPM 建议控制频率提高到 5kHz，800~1300RPM 建议 10kHz，并且高速必须斜坡启动。

---

## 先给结论

你现在震动变大，最可能是这几个原因：

```text
1. 开环启动太猛：直接 300RPM，没有速度斜坡
2. Uq 偏大：SVPWM 后等效利用率更高，开环更容易抖
3. 控制频率偏低：1kHz 对 300RPM/7极对数已经比较粗
4. 仍在控制循环里高频打印 ESP_LOGI，严重破坏实时性
5. PWM 还是边沿对齐 MCPWM_TIMER_COUNT_MODE_UP，不是中心对齐
6. 可能三相顺序/电角度方向和电机实际不匹配
```

---

## 第一件事：把参数降下来

先不要用：

```c
#define M0_TEST_RPM    300.0f
#define M0_TEST_UQ     2.0f
```

改成：

```c
#define M0_TEST_RPM    30.0f
#define M0_TEST_UQ     0.6f
```

能轻微稳定转以后，再加到：

```c
#define M0_TEST_RPM    100.0f
#define M0_TEST_UQ     1.0f
```

你现在直接 300RPM，其实对开环 FOC 来说已经不算“温柔启动”了。你的电机极对数是 7，对应电角速度是机械角速度的 7 倍。300RPM 机械转速等于 5 转/秒，电角频率就是：

```text
5 × 7 = 35Hz
```

也就是你一上电就让定子磁场按 **35Hz 电角频率** 转起来。转子如果没跟上，就会表现为抖动、震动、啸叫、失步。

---

## 第二件事：必须加启动斜坡

不要直接给目标 RPM。你现在是这样：

```c
vfoc_open_loop_svpwm_run(M0_TEST_RPM,
                         M0_TEST_UQ,
                         MOTOR_DRV_VBUS,
                         M0_FOC_DT_S);
```

建议先改成任务里慢慢爬升：

```c
static void m0_foc_control_task(void *arg)
{
    spwm_duty_t pwm_duty;

    float rpm_now = 0.0f;
    float uq_now = 0.0f;

    const float rpm_target = M0_TEST_RPM;
    const float uq_target = M0_TEST_UQ;

    /*
     * 每 1ms 调用一次。
     * rpm 每次加 0.05，1 秒加 50rpm。
     * uq 每次加 0.001，1 秒加 1V。
     */
    const float rpm_step = 0.05f;
    const float uq_step = 0.001f;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (rpm_now < rpm_target)
        {
            rpm_now += rpm_step;
            if (rpm_now > rpm_target)
            {
                rpm_now = rpm_target;
            }
        }

        if (uq_now < uq_target)
        {
            uq_now += uq_step;
            if (uq_now > uq_target)
            {
                uq_now = uq_target;
            }
        }

        vfoc_open_loop_svpwm_run(rpm_now,
                                 uq_now,
                                 MOTOR_DRV_VBUS,
                                 M0_FOC_DT_S);

        pwm_duty = vfoc_get_spwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);
    }
}
```

这个改完以后，震动一般会明显变小。

---

## 第三件事：把控制循环里的日志关掉

你现在 `vfoc_open_loop_spwm_run()` 里面有：

```c
ESP_LOGI(TAG, "UVW: %.3f, %.3f, %.3f, sum=%.3f", ...);
```

这个函数每 1ms 跑一次，也就是 **1秒打印1000次**。这对电机控制是灾难，会导致控制周期抖动，角度更新不均匀，电机必然抖。你的 `dt_s` 注释里也写了，如果 `dt_s` 不稳定，电机容易抖动、啸叫、转速不稳、启动失败。

先直接注释掉：

```c
// ESP_LOGI(TAG, "UVW: %.3f, %.3f, %.3f, sum=%.3f", ...);
```

或者 100 次打印一次：

```c
static uint32_t log_cnt = 0;

if ((log_cnt++ % 100) == 0)
{
    ESP_LOGI(TAG, "duty: %.3f %.3f %.3f",
             vfoc_dt.motor_drv_val.spwm_duty_val.duty_Ua,
             vfoc_dt.motor_drv_val.spwm_duty_val.duty_Ub,
             vfoc_dt.motor_drv_val.spwm_duty_val.duty_Uc);
}
```

---

## 第四件事：确认你真的在跑 SVPWM

你当前上传的 `m0_foc_control_task()` 里面还是调用：

```c
vfoc_open_loop_spwm_run(...)
```

如果你只是把 `vfoc_spwm_calc_duty()` 内部换成 SVPWM，那外面名字不重要；但如果你新增的是：

```c
vfoc_open_loop_svpwm_run(...)
```

那这里必须改成：

```c
vfoc_open_loop_svpwm_run(M0_TEST_RPM,
                         M0_TEST_UQ,
                         MOTOR_DRV_VBUS,
                         M0_FOC_DT_S);
```

否则你可能以为自己在跑 SVPWM，但实际还在跑旧流程。

---

## 第五件事：SVPWM 理论上不应该让震动突然变大

重点来了。

**标准 centered zero-sequence SVPWM 在没有过调制、没有限幅的情况下，不改变线电压。**

它只是三相一起加公共偏移：

```c
ua += offset;
ub += offset;
uc += offset;
```

相间电压仍然不变：

```text
Uab = Ua - Ub
Ubc = Ub - Uc
Uca = Uc - Ua
```

所以如果你从 SPWM 切到 SVPWM 后，震动明显变大，通常说明：

```text
1. 你的 SVPWM duty 缩放比例有问题
2. 你的 Uq 太大导致开环失步
3. 控制周期被日志打乱
4. 三相接线顺序或角度方向不匹配
5. duty 输出和硬件驱动逻辑反了
```

---

## 第六件事：检查三相顺序

如果电机抖、不顺滑、不怎么转，试试交换任意两相的软件输出，比如：

```c
m0_fd6287_set_duty(pwm_duty.duty_Ua,
                   pwm_duty.duty_Uc,
                   pwm_duty.duty_Ub);
```

或者把电角度方向反过来：

```c
vfoc_dt.motor_par.theta_e = -vfoc_dt.motor_par.theta_m *
                            vfoc_dt.motor_par.pole_pairs;
```

然后再归一化。

如果交换相序或者反向角度后明显变顺，说明之前的三相方向和电机实际相序不匹配。

---

## 第七件事：你的 PWM 现在还是边沿对齐

你当前 MCPWM 是：

```c
.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
```

这个是边沿对齐。电机控制里更推荐中心对齐：

```c
.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
```

但是这一步先别急着改。你先做这三件事：

```text
1. 降低 RPM/Uq
2. 加启动斜坡
3. 关掉 1kHz 日志
```

这三个处理完，如果还震，再改中心对齐 PWM。

---

## 你现在最推荐的测试配置

先这样：

```c
#define M0_TEST_RPM                50.0f
#define M0_TEST_UQ                 0.8f
#define M0_FOC_CTRL_FREQ_HZ        1000
```

然后任务里加斜坡。

稳定后再测试：

```c
#define M0_TEST_RPM                100.0f
#define M0_TEST_UQ                 1.0f
```

再之后：

```c
#define M0_TEST_RPM                300.0f
#define M0_TEST_UQ                 1.5f
```

不要一开始就 `300RPM + 2V`。

---

## 一句话判断

你现在不是“SVPWM 导致震动变大”，更像是：

```text
SVPWM 把你的开环控制问题暴露得更明显了。
```

先改：

```text
关日志 + 降 Uq/RPM + 加启动斜坡
```

这三个优先级最高。
