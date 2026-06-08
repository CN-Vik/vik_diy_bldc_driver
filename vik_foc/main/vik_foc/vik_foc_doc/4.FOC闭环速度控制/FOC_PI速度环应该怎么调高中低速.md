可以，**高中低速控制不是写三套完全不同的算法**，而是：

```text
同一个速度 PI 控制框架
+
不同速度段使用不同的参数
```

也就是所谓的 **分段调参 / gain scheduling**。

你现在代码已经有速度 PI、`UQ_LIMIT`、`SPEED_I_OUT_LIMIT`、`ramp_float()`、`dt_s`、`get_vfoc_mech_rpm()` 这些基础了。当前代码里 FOC 控制频率还是 `1kHz`，`UQ_LIMIT = 3.2f`，`SPEED_I_OUT_LIMIT = 1.5f`，目标速度现在写死在 `exp_motor_rpm = 89.0f`。

---

# 1. 先记住核心思路

你要做高中低速，不是这样：

```c
if (低速) 写一套控制
if (中速) 写一套控制
if (高速) 写一套控制
```

而是这样：

```text
目标速度
  ↓
根据目标速度选择一组参数
  ↓
速度 PI
  ↓
Uq 限幅
  ↓
Uq 斜坡
  ↓
SVPWM
```

低速、中速、高速主要区别是：

| 速度段 | 特点                    | 参数倾向                 |
| --- | --------------------- | -------------------- |
| 低速  | 速度反馈抖、齿槽感明显、容易咔咔      | 小 Uq、小 Ki、强滤波、慢斜坡    |
| 中速  | 比较好控                  | 中等 Uq、中等 PI、适中滤波     |
| 高速  | 反电动势大、需要更高 Uq、角度延迟影响大 | 更高 Uq、更高控制频率、目标斜坡必须有 |

---

# 2. 我建议你先这样分段

你现在还是学习阶段，可以先分成三段：

```text
低速：0 ~ 120 rpm
中速：120 ~ 500 rpm
高速：500 ~ 1300 rpm
```

先别急着搞 10000rpm。你代码注释里也写了，300~800rpm 建议提高 FOC 控制频率到 5kHz，800~1300rpm 建议提高到 10kHz。

---

# 3. 推荐初始参数表

你先用这套，不一定最终最优，但适合你现在调试：

| 档位 |   rpm 范围 |          Kp |          Ki |     Uq限制 |     I项限制 |          目标斜坡 |      Uq斜坡 | 速度滤波 |
| -- | -------: | ----------: | ----------: | -------: | -------: | ------------: | --------: | ---: |
| 低速 |    0~120 | 0.010~0.015 | 0.010~0.020 |     1.2V | 0.4~0.6V |  60~100 rpm/s |   4~8 V/s |    强 |
| 中速 |  120~500 | 0.015~0.025 | 0.020~0.040 | 2.0~3.0V | 0.8~1.2V | 200~400 rpm/s | 10~20 V/s |    中 |
| 高速 | 500~1300 | 0.010~0.020 | 0.010~0.030 | 3.0~5.0V | 1.0~1.5V | 400~800 rpm/s | 20~40 V/s |    弱 |

注意：**高速不是把 Kp/Ki 无限加大。**
高速更关键的是：

```text
Uq 够不够
FOC控制频率够不够
角度反馈延迟够不够小
速度反馈是否稳定
```

---

# 4. 先定义一个速度档位参数结构体

你可以加这个：

```c
typedef struct
{
    float rpm_min;           /* 这个档位的最小速度 */
    float rpm_max;           /* 这个档位的最大速度 */

    float kp;                /* 速度环比例系数 */
    float ki;                /* 速度环积分系数 */

    float uq_limit;          /* 这个速度段允许的最大Uq */
    float i_out_limit;       /* 积分项最大输出电压 */

    float rpm_ramp_per_s;    /* 目标速度每秒最大变化量 */
    float uq_ramp_per_s;     /* Uq每秒最大变化量 */

    float rpm_lpf_alpha;     /* 速度滤波系数，越小越平滑 */
    float deadband_rpm;      /* 速度死区 */
} speed_ctrl_profile_t;
```

然后定义三组参数：

```c
static const speed_ctrl_profile_t speed_profiles[] =
{
    /*
     * 低速档：0~120rpm
     * 特点：速度反馈抖，齿槽感明显，所以参数要软。
     */
    {
        .rpm_min = 0.0f,
        .rpm_max = 120.0f,
        .kp = 0.012f,
        .ki = 0.015f,
        .uq_limit = 1.2f,
        .i_out_limit = 0.5f,
        .rpm_ramp_per_s = 80.0f,
        .uq_ramp_per_s = 6.0f,
        .rpm_lpf_alpha = 0.06f,
        .deadband_rpm = 3.0f,
    },

    /*
     * 中速档：120~500rpm
     * 特点：比较好控，可以适当提高Uq和PI力度。
     */
    {
        .rpm_min = 120.0f,
        .rpm_max = 500.0f,
        .kp = 0.018f,
        .ki = 0.030f,
        .uq_limit = 2.5f,
        .i_out_limit = 1.0f,
        .rpm_ramp_per_s = 300.0f,
        .uq_ramp_per_s = 15.0f,
        .rpm_lpf_alpha = 0.10f,
        .deadband_rpm = 3.0f,
    },

    /*
     * 高速档：500~1300rpm
     * 特点：需要更高Uq，但Ki不要过猛，否则容易过冲。
     */
    {
        .rpm_min = 500.0f,
        .rpm_max = 1300.0f,
        .kp = 0.014f,
        .ki = 0.020f,
        .uq_limit = 4.0f,
        .i_out_limit = 1.2f,
        .rpm_ramp_per_s = 600.0f,
        .uq_ramp_per_s = 30.0f,
        .rpm_lpf_alpha = 0.15f,
        .deadband_rpm = 2.0f,
    },
};
```

---

# 5. 根据目标速度选择档位

加这个函数：

```c
static const speed_ctrl_profile_t *get_speed_profile(float target_rpm)
{
    float rpm_abs = fabsf(target_rpm);

    if (rpm_abs < 120.0f)
    {
        return &speed_profiles[0];     /* 低速 */
    }
    else if (rpm_abs < 500.0f)
    {
        return &speed_profiles[1];     /* 中速 */
    }
    else
    {
        return &speed_profiles[2];     /* 高速 */
    }
}
```

---

# 6. 控制逻辑建议这样写

你的速度环核心可以改成这个结构：

```c
float target_rpm_cmd = 300.0f;       /* 用户真正想要的目标速度 */
float target_rpm_ramp = 0.0f;        /* 斜坡后的目标速度 */

float rpm_raw = 0.0f;
float rpm_filt = 0.0f;
bool rpm_filt_init = false;

float uq_target = 0.0f;
float uq_out = 0.0f;

float speed_integral = 0.0f;
float kp_out = 0.0f;
float ki_out = 0.0f;

while (1)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /*
     * 你现在是1kHz控制周期，先固定1ms，避免dt_s再出错。
     */
    float dt_s = M0_FOC_DT_S;

    /*
     * 1. 根据目标速度选择低/中/高速参数
     */
    const speed_ctrl_profile_t *profile = get_speed_profile(target_rpm_cmd);

    /*
     * 2. 目标速度斜坡
     *    不能让目标速度突然从0跳到300/500/1000。
     */
    target_rpm_ramp = ramp_float(
        target_rpm_ramp,
        target_rpm_cmd,
        profile->rpm_ramp_per_s * dt_s
    );

    /*
     * 3. 读取原始速度
     */
    rpm_raw = get_vfoc_mech_rpm();

    /*
     * 4. 速度滤波
     */
    if (rpm_filt_init == false)
    {
        rpm_filt = rpm_raw;
        rpm_filt_init = true;
    }
    else
    {
        rpm_filt += profile->rpm_lpf_alpha * (rpm_raw - rpm_filt);
    }

    /*
     * 5. 速度误差
     */
    float err_rpm = target_rpm_ramp - rpm_filt;

    /*
     * 6. 死区处理
     */
    if (fabsf(err_rpm) < profile->deadband_rpm)
    {
        err_rpm = 0.0f;

        /*
         * 误差很小时，积分慢慢释放，避免在目标附近来回顶。
         */
        speed_integral *= 0.98f;
    }
    else
    {
        speed_integral += err_rpm * dt_s;
    }

    /*
     * 7. P项
     */
    kp_out = profile->kp * err_rpm;

    /*
     * 8. I项 + 积分限幅
     */
    ki_out = profile->ki * speed_integral;
    ki_out = limit_float(ki_out,
                         -profile->i_out_limit,
                         profile->i_out_limit);

    /*
     * 9. 反推积分，防止积分内部无限变大
     */
    if (profile->ki > 0.000001f)
    {
        speed_integral = ki_out / profile->ki;
    }

    /*
     * 10. PI输出
     */
    uq_target = kp_out + ki_out;

    /*
     * 11. 保留你当前方向修正
     */
    uq_target *= -1.0f;

    /*
     * 12. 当前档位Uq限幅
     */
    uq_target = limit_float(uq_target,
                            -profile->uq_limit,
                            profile->uq_limit);

    /*
     * 13. Uq斜坡
     *     避免Uq突然变化导致咔咔、顿挫。
     */
    uq_out = ramp_float(
        uq_out,
        uq_target,
        profile->uq_ramp_per_s * dt_s
    );

    /*
     * 14. 输出SVPWM
     */
    vfoc_set_svpwm(uq_out,
                   M0_TEST_UD,
                   MOTOR_DRV_VBUS);

    pwm_duty = vfoc_get_pwm_duty();

    m0_fd6287_set_duty(pwm_duty.duty_Ua,
                       pwm_duty.duty_Ub,
                       pwm_duty.duty_Uc);

    if (++log_cnt >= 100)
    {
        log_cnt = 0;

        ESP_LOGI(TAG,
                 "cmd:%.1f,ramp:%.1f,raw:%.1f,filt:%.1f,err:%.1f,uq:%.2f,p:%.2f,i:%.2f,kp:%.3f,ki:%.3f",
                 target_rpm_cmd,
                 target_rpm_ramp,
                 rpm_raw,
                 rpm_filt,
                 err_rpm,
                 uq_out,
                 kp_out,
                 ki_out,
                 profile->kp,
                 profile->ki);
    }
}
```

---

# 7. 具体调参流程

## 第一步：先调低速档

目标速度：

```c
target_rpm_cmd = 60.0f;
```

先用：

```c
kp = 0.010f;
ki = 0.010f;
uq_limit = 1.0f;
i_out_limit = 0.4f;
```

看现象。

如果到不了 60rpm，但是 Uq 已经接近 `-1.0V`：

```text
说明Uq限制太小，加 uq_limit
```

改：

```c
uq_limit = 1.2f;
```

如果速度能到，但是波动大、咔咔：

```text
说明控制太猛，减 kp / ki，或者加强滤波
```

改：

```c
kp = 0.008f;
ki = 0.008f;
rpm_lpf_alpha = 0.04f;
```

---

## 第二步：调中速档

目标速度：

```c
target_rpm_cmd = 300.0f;
```

先用：

```c
kp = 0.018f;
ki = 0.030f;
uq_limit = 2.5f;
i_out_limit = 1.0f;
```

观察：

```text
如果实际速度低于目标，而且 uq_out 长期顶到 uq_limit：
    加大 uq_limit，比如 2.5 -> 3.0

如果实际速度低于目标，但 uq_out 没顶满：
    加大 Ki 或 i_out_limit

如果速度上下波动明显：
    减小 Kp 或 Ki，或者加速度滤波
```

---

## 第三步：调高速档

目标速度先不要一下给很高，按这个来：

```text
500rpm
700rpm
900rpm
1100rpm
1300rpm
```

你现在代码 FOC 控制周期是 1kHz。按你代码注释，高速阶段建议提高到 10kHz。

如果还保持 1kHz，我建议你先别超过：

```text
300 ~ 500rpm
```

如果要稳定做 800rpm、1000rpm、1300rpm，建议改：

```c
#define M0_FOC_CTRL_FREQ_HZ  5000
```

甚至：

```c
#define M0_FOC_CTRL_FREQ_HZ  10000
```

对应这些也会自动变：

```c
#define M0_FOC_DT_S          (1.0f / M0_FOC_CTRL_FREQ_HZ)
#define M0_CTRL_ALARM_COUNT  (M0_CTRL_TIMER_RES_HZ / M0_FOC_CTRL_FREQ_HZ)
```

---

# 8. 怎么判断该调哪个参数？

你看 LOG 就够了。

## 情况 A：实际速度上不去，Uq 已经顶满

例如：

```text
target = 300
rpm = 220
uq = -2.50
```

而当前档位 `uq_limit = 2.5V`。

结论：

```text
不是 PI 不努力，是 Uq 不够。
```

处理：

```text
加 uq_limit
或者检查电角度、极对数、相序、供电能力
```

---

## 情况 B：实际速度上不去，Uq 没顶满

例如：

```text
target = 300
rpm = 250
uq = -1.2
uq_limit = 2.5
```

结论：

```text
PI 输出不够。
```

处理：

```text
加 Ki
加 i_out_limit
适当加 Kp
```

---

## 情况 C：速度来回跳，Uq 正负来回跳

例如：

```text
uq = -1.5, +1.2, -1.8, +1.0
```

结论：

```text
控制太猛 / 速度反馈太抖。
```

处理：

```text
减 Kp
减 Ki
减 i_out_limit
加速度滤波
加 Uq 斜坡
```

---

## 情况 D：启动冲很高，然后慢慢回落

结论：

```text
启动目标变化太快 / Uq太猛 / 积分太大。
```

处理：

```text
降低 rpm_ramp_per_s
降低 uq_ramp_per_s
降低 i_out_limit
降低 uq_limit
```

---

# 9. 最后给你一个推荐起步配置

你可以直接先用这三档：

```c
/* 低速 0~120rpm */
kp = 0.012f;
ki = 0.015f;
uq_limit = 1.2f;
i_out_limit = 0.5f;
rpm_ramp_per_s = 80.0f;
uq_ramp_per_s = 6.0f;
rpm_lpf_alpha = 0.06f;

/* 中速 120~500rpm */
kp = 0.018f;
ki = 0.030f;
uq_limit = 2.5f;
i_out_limit = 1.0f;
rpm_ramp_per_s = 300.0f;
uq_ramp_per_s = 15.0f;
rpm_lpf_alpha = 0.10f;

/* 高速 500~1300rpm */
kp = 0.014f;
ki = 0.020f;
uq_limit = 4.0f;
i_out_limit = 1.2f;
rpm_ramp_per_s = 600.0f;
uq_ramp_per_s = 30.0f;
rpm_lpf_alpha = 0.15f;
```

---

# 一句话总结

你要实现高中低速控制，本质是：

```text
一套速度 PI 控制
三套参数
目标速度斜坡
Uq斜坡
速度滤波
不同速度段不同 Uq 限制和 PI 参数
```

低速要“软”，中速要“稳”，高速要“控制频率高 + Uq够 + 斜坡启动”。

--------
--------
-------
对，理解得很对。

**高中低速的控制策略大方向一样，都是速度 PI，但是参数不一样。**

不是说低速用一套算法、中速换一套算法、高速再换一套算法，而是：

```text
同一个速度环 PI 框架
不同速度段切换不同参数
```

也就是：

```text
低速：一组 Kp / Ki / Uq限制 / 积分限幅 / 滤波参数
中速：一组 Kp / Ki / Uq限制 / 积分限幅 / 滤波参数
高速：一组 Kp / Ki / Uq限制 / 积分限幅 / 滤波参数
```

这就叫 **分段 PID 参数**，也可以叫 **gain scheduling，增益调度**。

---

## 为啥高中低速参数不能完全一样？

因为电机在不同速度下，特性不一样。

### 低速时

低速比如：

```text
30rpm、60rpm、100rpm
```

低速的问题是：

```text
速度反馈抖
齿槽感明显
AS5600速度计算容易跳
Uq稍微大一点就一顿一顿
```

所以低速要“软”：

```text
Kp 小一点
Ki 小一点
Uq 限制小一点
积分限幅小一点
速度滤波强一点
Uq斜坡慢一点
```

比如：

```c
kp = 0.010f;
ki = 0.010f;
uq_limit = 1.2f;
i_out_limit = 0.4f;
```

---

### 中速时

中速比如：

```text
150rpm、300rpm、500rpm
```

这个阶段电机已经转起来了，速度反馈通常比低速稳定。

所以可以稍微“硬”一点：

```text
Kp 可以稍微大
Ki 可以稍微大
Uq 限制可以加大
积分限幅可以加大
速度滤波不用太强
```

比如：

```c
kp = 0.018f;
ki = 0.030f;
uq_limit = 2.5f;
i_out_limit = 1.0f;
```

---

### 高速时

高速比如：

```text
800rpm、1000rpm、1300rpm
```

高速时最大问题不是 PID 有多猛，而是：

```text
反电动势变大，需要更高 Uq
电角度变化快，FOC控制频率要提高
编码器延迟影响变大
```

所以高速要：

```text
Uq 限制更大
目标速度斜坡不能太猛
控制频率最好提高
Kp/Ki 不一定继续加大，反而要防止震荡
```

比如：

```c
kp = 0.014f;
ki = 0.020f;
uq_limit = 4.0f;
i_out_limit = 1.2f;
```

你当前代码里 FOC 控制周期还是 `1kHz`，而你代码注释里也写了中高速建议提高控制频率，所以高速前最好先考虑把 FOC 控制频率提到 `5kHz` 或更高。

---

## Uq 限幅也要分段

这个非常重要。

不能所有速度都用同一个 `UQ_LIMIT = 3.2V`。

比如你之前目标才 60rpm，但一启动 Uq 直接打到 `-3.2V`，结果速度冲到 200 多 rpm。低速目标根本不需要这么大的 Uq。

所以建议：

```text
低速 Uq 限制小
中速 Uq 限制中等
高速 Uq 限制大
```

例如：

```c
低速：uq_limit = 1.2f;
中速：uq_limit = 2.5f;
高速：uq_limit = 4.0f;
```

这样低速不会猛冲，高速又有足够电压顶上去。

---

## 积分限幅也要分段

也对，**积分限幅也应该不一样。**

积分项的作用是消除长期速度误差。

低速时积分太大，会出现：

```text
来回顶
咔咔
正反拉扯
低速震动
```

所以低速积分限幅要小：

```c
低速：i_out_limit = 0.4f ~ 0.6f;
```

中速需要更强的补偿：

```c
中速：i_out_limit = 0.8f ~ 1.2f;
```

高速需要克服反电动势和负载：

```c
高速：i_out_limit = 1.0f ~ 1.5f;
```

但也不能无限大，否则速度过冲会严重。

---

## 更准确地说，高中低速要分这几类参数

不是只分 PID。

你应该分这些：

```c
typedef struct
{
    float kp;              // P参数
    float ki;              // I参数
    float uq_limit;        // 总Uq限制
    float i_out_limit;     // 积分输出限制
    float rpm_ramp;        // 目标速度斜坡
    float uq_ramp;         // Uq输出斜坡
    float rpm_lpf_alpha;   // 速度滤波强度
    float deadband_rpm;    // 速度死区
} speed_profile_t;
```

也就是：

```text
PID参数不一样
Uq限制不一样
积分限幅不一样
速度斜坡不一样
Uq斜坡不一样
滤波强度不一样
死区也可以不一样
```

---

## 你可以先这么理解

```text
低速档：
    轻轻给油，慢慢调，防止咔咔

中速档：
    正常给油，正常调，兼顾响应和稳定

高速档：
    给够电压，但不要乱加Kp/Ki，重点是控制频率和斜坡
```

所以最终就是：

```text
低速：软
中速：稳
高速：够力但别炸
```

---

## 一句话总结

对，你理解得对：

```text
高中低速用的是同一个 PI 速度环，
但是 Kp、Ki、Uq限幅、积分限幅、速度滤波、斜坡参数都应该分段设置。
```

这不是复杂化，而是电机控制里很常见的做法。你现在先把低速 `60~100rpm` 调稳，再调中速 `200~500rpm`，最后再考虑高速。
