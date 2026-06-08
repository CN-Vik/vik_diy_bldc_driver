你这个现象**非常正常**，本质原因是：**刚上电时，位置环误差太大，PD 一下子给了很大的 `Uq`，电机瞬间大力矩拉回目标点，所以猛震一下。**

你这份代码里最关键的问题有 4 个：

---

## 1. 280° 到 10°，你的误差其实是 90°

你的函数：

```c
err_angle = angle_error_deg(exp_angle, now_angle);
```

假设：

```c
exp_angle = 10;
now_angle = 280;
```

计算过程是：

```c
err = 10 - 280 = -270;
err < -180，所以 err += 360;
err = 90;
```

也就是说，程序认为：**从 280° 走到 10°，最近路径是正方向转 90°。**

这个逻辑是对的。

但是问题是：**90° 对于位置闭环来说已经是一个很大的阶跃误差。**

---

## 2. 你的启动瞬间 `uq` 会非常大

你现在代码里是：

```c
float kp = 0.35f;
float kd = 0.86f;

kd_err_parm = err_angle - last_err_angle;

uq = (kp * err_angle) + (kd * kd_err_parm);
uq *= -1;
```

刚启动时：

```c
last_err_angle = 0;
err_angle = 90;
kd_err_parm = 90 - 0 = 90;
```

所以：

```c
uq = 0.35 * 90 + 0.86 * 90
   = 31.5 + 77.4
   = 108.9V
```

然后：

```c
uq *= -1;
uq = -108.9V;
```

你母线电压才 12V，但控制器第一拍就想输出 **108.9V 等效力矩电压**。

就算你的 `vfoc_set_svpwm()` 里面做了限制，结果也大概率是直接打满，所以电机会猛地一抽。

---

## 3. 你定义了 `UQ_LIMIT`，但没有用

你代码里有：

```c
#define UQ_LIMIT 4.0f
```

但是后面没有：

```c
uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
```

所以这个限幅现在等于没生效。

这个必须加。

最小修改版：

```c
uq = (kp * err_angle) + (kd * kd_err_parm);
uq *= -1;

/* 限制 Uq 最大输出，防止启动瞬间力矩过大 */
uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
```

但是只加这个还不够，最好继续做下面几个保护。

---

## 4. 你这个 D 项启动时会“踢一脚”

D 项本质看的是误差变化：

```c
kd_err_parm = err_angle - last_err_angle;
```

刚上电时，`last_err_angle = 0`，但是当前误差突然变成 90°，所以 D 项会认为：

> 误差瞬间变化了 90°，赶紧大力刹/推！

这就叫 **微分冲击 / derivative kick**。

所以刚上电时建议：

```c
last_err_angle = err_angle;
```

不要让第一拍 D 项参与控制。

---

# 推荐解决方案：目标角度不要一步给 10°，而是从当前角度慢慢爬过去

你现在是：

```c
float exp_angle = 10.0f;
```

电机上电在 280°，目标直接是 10°，相当于你一上电就命令它：

> 立刻从 280° 拉到 10°。

更稳的做法是：

```c
真实目标角度 target_angle = 10°
软目标角度 soft_exp_angle = 当前角度
然后 soft_exp_angle 慢慢向 target_angle 靠近
```

也就是：

```c
电机实际位置：280°
软目标位置：280°
目标位置：10°

然后软目标：
280 -> 281 -> 282 -> ... -> 359 -> 0 -> 1 -> ... -> 10
```

这样电机不会猛冲，而是平滑回到 10°。

---

# 你可以这样改

先加一个函数：

```c
/**
 * @brief 让当前给定角度 current_deg 按最短路径慢慢靠近 target_deg
 *
 * @param current_deg 当前软目标角度
 * @param target_deg  最终目标角度
 * @param max_step_deg 每个控制周期最多变化多少度
 * @return float 更新后的软目标角度
 */
static float angle_move_towards_deg(float current_deg,
                                    float target_deg,
                                    float max_step_deg)
{
    float err = angle_error_deg(target_deg, current_deg);

    /* 限制每次最多走 max_step_deg，防止目标角度阶跃 */
    err = limit_float(err, -max_step_deg, max_step_deg);

    current_deg += err;

    /* 保证角度始终在 0~360 度 */
    LIMIT_EXP_MECH_360(current_deg);

    return current_deg;
}
```

然后你的任务里改成这种结构：

```c
static void m0_foc_control_task(void *arg)
{
    pwm_duty_t pwm_duty;

    float target_angle = 10.0f;       /* 最终希望到达的位置 */
    float soft_exp_angle = 0.0f;      /* 软目标位置，用来慢慢靠近 target_angle */

    float now_angle = 0.0f;
    float err_angle = 0.0f;
    float last_err_angle = 0.0f;

    float uq = 0.0f;
    float last_uq = 0.0f;

    float kp = 0.05f;     /* 先小一点，不要一上来 0.35 */
    float kd = 0.00f;     /* 初期建议先关闭 D 项 */

    bool first_flag = true;

    uint32_t start_cnt = 0;
    uint32_t log_cnt = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        now_angle = get_vfoc_theta_m_deg();

        /*
         * 第一次进入闭环时：
         * 不要直接把目标设成 10 度。
         * 先让软目标等于当前角度。
         * 这样启动瞬间误差为 0，不会猛地打一脚。
         */
        if (first_flag)
        {
            first_flag = false;

            soft_exp_angle = now_angle;

            err_angle = angle_error_deg(soft_exp_angle, now_angle);
            last_err_angle = err_angle;

            uq = 0.0f;
            last_uq = 0.0f;

            continue;
        }

        start_cnt++;

        /*
         * 软目标慢慢靠近最终目标。
         *
         * 控制周期是 1ms。
         * max_step_deg = 0.05 度/周期
         * 等效速度 = 0.05 * 1000 = 50 度/秒
         *
         * 也就是从 280 度到 10 度，大约走 90 度，
         * 需要 90 / 50 = 1.8 秒左右。
         */
        soft_exp_angle = angle_move_towards_deg(soft_exp_angle,
                                                target_angle,
                                                0.05f);

        err_angle = angle_error_deg(soft_exp_angle, now_angle);

        /*
         * 小误差死区，防止目标附近来回抖动
         */
        if (fabsf(err_angle) < POS_DEADBAND_DEG)
        {
            err_angle = 0.0f;
        }

        /*
         * 先不要用 D 项。
         * 等 P 环调顺了，再考虑加 D。
         */
        float p_out = kp * err_angle;
        float d_out = kd * (err_angle - last_err_angle);

        uq = p_out + d_out;

        /*
         * 如果发现电机方向反了，就改这里的符号。
         */
        uq *= -1.0f;

        /*
         * 启动阶段 Uq 限幅再小一点。
         * 前 500ms 只允许 ±0.8V。
         * 后面才允许到 ±2.0V 或更高。
         */
        float uq_limit_now = 0.8f;

        if (start_cnt > 500)
        {
            uq_limit_now = 2.0f;
        }

        /*
         * Uq 绝对值限幅，防止输出过猛。
         */
        uq = limit_float(uq, -uq_limit_now, uq_limit_now);

        /*
         * Uq 斜率限幅。
         * 每 1ms 最多变化 0.02V。
         * 这样不会从 0V 瞬间跳到 2V。
         */
        uq = limit_float(uq,
                         last_uq - 0.02f,
                         last_uq + 0.02f);

        vfoc_set_svpwm(uq,
                       M0_TEST_UD,
                       MOTOR_DRV_VBUS);

        pwm_duty = vfoc_get_pwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);

        last_err_angle = err_angle;
        last_uq = uq;

        if (++log_cnt >= 100)
        {
            log_cnt = 0;

            ESP_LOGI(TAG,
                     "uq:%.2f, target:%.2f, soft:%.2f, now:%.2f, err:%.2f",
                     uq,
                     target_angle,
                     soft_exp_angle,
                     now_angle,
                     err_angle);
        }
    }
}
```

---

# 你现在这句也要注意

你代码里有：

```c
exp_angle++;
```

这个是在每 1ms 加 1 度。

也就是说目标角度速度是：

```c
1度 / 1ms = 1000度/s
```

换算成机械转速：

```c
1000 / 360 * 60 = 166.7 rpm
```

所以你现在的 `exp_angle` 不是固定 10°，而是在一直跑。

如果你想做固定位置控制，应该先把这个删掉：

```c
// exp_angle++;
```

否则它不是“保持 10°位置”，而是“目标角度一直转圈”。

---

# 最推荐你的调试顺序

第一步，先只用 P，不用 D：

```c
float kp = 0.02f;
float kd = 0.0f;
```

第二步，限制 Uq：

```c
#define UQ_LIMIT 1.0f
```

第三步，启动时软目标等于当前角度：

```c
soft_exp_angle = now_angle;
```

第四步，软目标慢慢靠近最终目标：

```c
soft_exp_angle = angle_move_towards_deg(soft_exp_angle, target_angle, 0.02f);
```

第五步，确认方向对不对。

如果电机离目标越来越远，就改：

```c
uq *= -1.0f;
```

或者把它去掉。

---

# 一句话总结

你这个刚上电猛震，不是 FOC 本身的问题，而是**位置环启动时目标角度阶跃太大 + Uq 没限幅 + D 项第一拍冲击太大**。

最稳的解决方法是：

```c
上电先读取当前角度
软目标 = 当前角度
Uq = 0
然后软目标慢慢滑向真正目标角度
同时 Uq 做幅值限幅和斜率限幅
```

这样电机上电会“慢慢回零”，不会“啪”地猛抽一下。
