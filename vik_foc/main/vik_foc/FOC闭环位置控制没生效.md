你这个**有闭环的形状，但闭环效果很差甚至没有**，核心原因不是 SVPWM 本身，而是这几个点：

## 1. `kp=6.5` 太离谱，Uq 直接打爆了

你现在是：

```c
err_v = target_v - curent_v;
uq = kp * err_v;
```

而 `target_v = 50`，假设当前角度是 0°，那就是：

```c
uq = 6.5 * 50 = 325V
```

但你的母线才 `12V`。后面的 SVPWM 会把电压自动压到线性区极限，所以你的闭环 P 控制基本变成了：

```c
误差只要稍微大一点 -> 直接满输出
```

不是线性闭环，而是“猛推、过冲、抖动、乱震”。你的代码里确实是 `kp=6.5`，然后直接 `uq = kp * err_v`，再送进 `vfoc_set_svpwm()`。
而 SVPWM 里超过最大电压矢量会被等比例缩小，状态变成饱和。

建议先这样：

```c
#define POS_KP          0.02f   // 单位：V/deg，先从 0.01 ~ 0.05 试
#define UQ_LIMIT        1.2f    // 初期先限制在 ±0.8V ~ ±2V
#define POS_DEADBAND    0.5f    // 小误差死区
```

比如误差 50°：

```c
uq = 0.02 * 50 = 1.0V
```

这才像一个能调的闭环。

---

## 2. 编码器角度 100ms 才更新一次，控制环 1ms 跑一次

你的 FOC 控制任务是 GPTimer 1ms 通知一次，也就是 1kHz 控制。

但是 AS5600 角度任务里：

```c
vTaskDelay(pdMS_TO_TICKS(100));
```

也就是**100ms 才更新一次角度**。

这就很致命了：

```text
FOC控制环：1ms 算一次
角度反馈：100ms 变一次
```

所以控制器看到的是“过期角度”。电机已经动了，但你的 `curent_v` 还是旧值，Uq 继续猛推，等 100ms 后角度突然更新，又开始反向猛推，表现就是：**没闭环感、抖动、冲过头、来回震**。

建议先改成：

```c
vTaskDelay(pdMS_TO_TICKS(2));   // 先 2ms
```

或者更好：**不要单独 100ms 任务更新角度**，直接在 `m0_foc_control_task()` 里读取 AS5600 当前角度。初期可以 1ms 或 2ms 读一次，串口打印降低到 100ms 打一次。

---

## 3. 角度误差没有处理 0°/360° 回绕

你现在是：

```c
err_v = target_v - curent_v;
```

这对普通数值没问题，但角度是圆的。

例如：

```text
target = 10°
current = 350°
```

真实最近误差应该是：

```text
+20°
```

但你的代码算出来是：

```text
10 - 350 = -340°
```

电机会往反方向猛转 340°，这肯定不像闭环。

要加一个角度误差归一化函数：

```c
static float angle_error_deg(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;

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
```

---

## 4. 你现在缺少“FOC 电角度零点校准”

你现在的电角度是：

```c
电角度 = 机械角度 * 极对数
```

代码里也是直接 `get_vfoc_theta_m_deg() * pole_pairs`，然后转弧度。

但是这还不够。FOC 有感控制必须知道：

```text
AS5600 的 0°位置 和 转子磁场 d轴 的对应关系
```

也就是要有：

```c
theta_e = direction * pole_pairs * (theta_m - mech_zero) + elec_zero;
```

你现在 AS5600 做的零点校准更像“机械角度零点”，不是严格的 FOC 电角度零点。`vfoc_set_svpwm()` 里虽然会根据当前机械角度算电角度，然后做 Park 逆变换和 SVPWM 输出，但如果电角度零点错了，Uq 就不一定在真正的 q 轴上，电机会表现为没力、反转、抖动。

简单判断方法：

```text
给一个很小的 Uq，比如 +0.5V
如果电机往目标方向靠近，方向大概率对
如果远离目标，说明方向反了
如果只抖不动，可能电角度零点不对/相序不对
```

方向反了可以先临时改：

```c
uq = -uq;
```

或者在电角度计算里加方向：

```c
#define FOC_SENSOR_DIR   (-1.0f)   // 或 +1.0f
```

---

## 5. 建议你先这样改核心控制代码

先不要搞复杂 PID，先把 P 位置闭环跑顺。

```c
#define POS_KP              0.02f     // V/deg，先小一点
#define UQ_LIMIT            1.2f      // 初期限制 ±1.2V
#define POS_DEADBAND_DEG    0.5f      // 小误差死区

static float limit_float(float x, float min, float max)
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

static float angle_error_deg(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;

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

static void m0_foc_control_task(void *arg)
{
    pwm_duty_t pwm_duty;

    float uq = 0.0f;
    float target_v = 50.0f;
    float current_v = 0.0f;
    float err_v = 0.0f;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        current_v = get_vfoc_theta_m_deg();

        /*
         * 角度是圆形变量，必须算最短路径误差。
         */
        err_v = angle_error_deg(target_v, current_v);

        /*
         * 小误差死区，避免到位后轻微抖动。
         */
        if (fabsf(err_v) < POS_DEADBAND_DEG)
        {
            uq = 0.0f;
        }
        else
        {
            uq = POS_KP * err_v;
            uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
        }

        /*
         * 如果发现电机远离目标，把 uq 改成 -uq，
         * 或者修正编码器方向/电角度方向。
         */
        vfoc_set_svpwm(uq,
                       M0_TEST_UD,
                       MOTOR_DRV_VBUS);

        pwm_duty = vfoc_get_pwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);
    }
}
```

另外你这里最好把：

```c
#elifdef USE_FOC_SVPWM
```

改成更稳妥的写法：

```c
#elif defined(USE_FOC_SVPWM)
```

不然不同编译器/标准下可能不识别，导致 SVPWM 分支根本没编进去。对应位置就在你的控制任务宏判断这里。

---

## 你现在优先排查顺序

第一步，把 AS5600 角度更新从 100ms 改成 2ms，或者直接在控制环里读。

第二步，把 `kp=6.5` 改成 `0.02f` 左右，并且给 `uq` 加 ±1.2V 限幅。

第三步，加 `angle_error_deg()`，解决 0/360° 跨界问题。

第四步，测试方向：目标 50°，手转当前角度，观察 `err_v`、`uq` 是否符合预期。如果电机越推越远，先把 `uq = -uq` 验证方向。

第五步，再做电角度零点校准。没有这个，真正的有感 FOC 很容易只抖不稳。


--------
---------
--------

你现在这个“还没闭环效果”，我判断**优先不是调 PID**，而是先排这 4 个硬问题。尤其第 1 个很关键。

## 1. 你可能根本没打开功率驱动使能

你的 `m0_fd6287_pwm.c` 里定义了：

```c
void vbus_en_io_init(void);
void vbus_enable(bool enable);
```

但是你 `app_main()` 里只做了：

```c
motor_encoder_init();
vfoc_init();
m0_fd6287_mcpwm_init();
m0_fd6287_foc_start();
```

没看到调用：

```c
vbus_en_io_init();
vbus_enable(true);
```

也就是说：**PWM 在变，但 FD6287/MOS 驱动可能没真正输出功率**。你代码里确实有 VBUS_EN_GPIO 和 `vbus_enable()`，但启动流程里没有看到使能调用。 

在 `app_main()` 里改成这样：

```c
void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC position closed loop");

    motor_encoder_init();

    vfoc_init();

    vbus_en_io_init();
    vbus_enable(true);       // 关键：打开功率驱动

    m0_fd6287_mcpwm_init();

    m0_fd6287_foc_start();
}
```

先确认 `VBUS_EN_GPIO = 12` 是不是你板子真实的 FD6287 使能脚。

---

## 2. 先不要用 `#ifdef`，直接强制跑 SVPWM

你现在控制任务里是：

```c
#ifdef USE_FOC_SPWM
    vfoc_open_loop_spwm_run(...);
#elifdef USE_FOC_SVPWM
    vfoc_set_svpwm(uq, M0_TEST_UD, MOTOR_DRV_VBUS);
#endif
```

如果宏没定义对，或者分支没进，你最终拿到的 duty 可能一直是旧值/默认值。你的核心控制代码就是从角度误差算 `uq`，再调用 `vfoc_set_svpwm()`。

调试阶段先别搞宏，直接写死：

```c
vfoc_set_svpwm(uq,
               M0_TEST_UD,
               MOTOR_DRV_VBUS);
```

也就是临时改成：

```c
while (1)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    curent_v = get_vfoc_theta_m_deg();
    target_v = 50.0f;

    err_v = angle_error_deg(target_v, curent_v);

    uq = 0.02f * err_v;
    uq = m0_limit_float(uq, -1.2f, 1.2f);

    vfoc_set_svpwm(uq, 0.0f, MOTOR_DRV_VBUS);

    pwm_duty = vfoc_get_pwm_duty();

    m0_fd6287_set_duty(pwm_duty.duty_Ua,
                       pwm_duty.duty_Ub,
                       pwm_duty.duty_Uc);
}
```

等闭环有效了，再恢复宏。

---

## 3. 你的 AS5600 角度反馈还是太慢

你现在 AS5600 角度任务里是：

```c
set_vfoc_theta_m_deg(angle);
vTaskDelay(pdMS_TO_TICKS(100));
```

这代表**100ms 才刷新一次角度**。但你的 FOC 控制任务是 1ms 周期，也就是控制器跑 100 次，角度才更新 1 次。这样闭环会非常迟钝，甚至看起来不像闭环。

先改成：

```c
vTaskDelay(pdMS_TO_TICKS(2));
```

并且不要 2ms 打一次日志。日志要降频：

```c
static int log_cnt = 0;

if (++log_cnt >= 50)
{
    log_cnt = 0;
    ESP_LOGI(TAG, "motor_angle = %.2f deg", angle);
}
```

---

## 4. 你还缺“FOC 电角度零点/方向校准”

你现在的电角度是：

```c
elec_deg = get_vfoc_theta_m_deg() * vfoc_dt.motor_par.pole_pairs;
```

然后转弧度给 Park 逆变换。代码确实是这么算的。

但真正有感 FOC 不是简单：

```c
电角度 = 机械角度 * 极对数
```

还要有：

```c
电角度 = 机械角度 * 极对数 + 电角度零偏
```

也就是：

```c
theta_e = pole_pairs * theta_m + theta_e_offset;
```

如果这个 offset 错了，Uq 就不在真正的 q 轴上，表现就是：

```text
不跟位置
乱抖
没力
一推就反
越控越远
```

先做一个最小判断：

```c
vfoc_set_svpwm(0.8f, 0.0f, 12.0f);
```

如果电机只抖、不稳定吸住某个位置，说明**相序/电角度/零点还没对**。

如果给：

```c
vfoc_set_svpwm(0.8f, 0.0f, 12.0f);
```

和：

```c
vfoc_set_svpwm(-0.8f, 0.0f, 12.0f);
```

两个方向效果都不对，优先查相序、PWM 三相对应、电角度零偏。

---

## 你现在直接按这个版本试

把控制任务先改成这个，别加积分、别加 D：

```c
#include <math.h>

#define POS_KP              0.02f
#define UQ_LIMIT            1.2f
#define POS_DEADBAND_DEG    1.0f

static float angle_error_deg(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;

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

static float limit_float_local(float x, float min, float max)
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

static void m0_foc_control_task(void *arg)
{
    pwm_duty_t pwm_duty;

    float uq = 0.0f;
    float target_v = 50.0f;
    float current_v = 0.0f;
    float err_v = 0.0f;

    uint32_t log_cnt = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        current_v = get_vfoc_theta_m_deg();

        err_v = angle_error_deg(target_v, current_v);

        if (fabsf(err_v) < POS_DEADBAND_DEG)
        {
            uq = 0.0f;
        }
        else
        {
            uq = POS_KP * err_v;
            uq = limit_float_local(uq, -UQ_LIMIT, UQ_LIMIT);
        }

        /*
         * 如果电机越控越远，先临时改成：
         * uq = -uq;
         */
        vfoc_set_svpwm(uq, 0.0f, MOTOR_DRV_VBUS);

        pwm_duty = vfoc_get_pwm_duty();

        m0_fd6287_set_duty(pwm_duty.duty_Ua,
                           pwm_duty.duty_Ub,
                           pwm_duty.duty_Uc);

        if (++log_cnt >= 100)
        {
            log_cnt = 0;
            ESP_LOGI(TAG,
                     "target=%.2f cur=%.2f err=%.2f uq=%.3f duty=%.3f %.3f %.3f",
                     target_v,
                     current_v,
                     err_v,
                     uq,
                     pwm_duty.duty_Ua,
                     pwm_duty.duty_Ub,
                     pwm_duty.duty_Uc);
        }
    }
}
```

## 你看日志判断

正常闭环应该是这样：

```text
target=50 cur=10 err=40 uq=0.8
target=50 cur=30 err=20 uq=0.4
target=50 cur=45 err=5  uq=0.1
target=50 cur=49 err=1  uq≈0
```

如果你看到：

```text
cur 一直不变
```

说明 AS5600 没实时更新，或者 `set_vfoc_theta_m_deg()` 没跑。

如果你看到：

```text
uq 有变化，duty 有变化，但电机不动
```

优先查 `vbus_enable(true)`、FD6287 使能脚、三相 PWM 是否真的到驱动输入。

如果你看到：

```text
cur 越来越远
```

先把：

```c
uq = -uq;
```

验证方向。

如果你看到：

```text
电机只抖，不吸目标
```

就是相序/电角度零点没校准，不是 PID 参数问题。


-----
-----
----
你和灯哥这版**最大差别不是 PWM 频率、SPWM/SVPWM，也不是 ESP-IDF 和 Arduino 的区别**，而是这 5 个核心逻辑。

我这边打开你给的 DengFOC 官网链接时返回 502，但你贴出来的代码和网上整理的 DengFOC 笔记内容一致：它的关键点就是 `_electricalAngle()` 里有 `DIR * PP * 机械角度 - zero_electric_angle`，并且上电时先用固定电角度对齐转子，再记录 `zero_electric_angle`。([dengfoc.com][1])

---

# 1. 灯哥有“电角度零点校准”，你现在没有

灯哥代码最关键的是这个：

```c
setPhaseVoltage(3, 0, _3PI_2);
delay(3000);
zero_electric_angle = _electricalAngle();
setPhaseVoltage(0, 0, _3PI_2);
```

然后电角度这样算：

```c
float _electricalAngle()
{
    return _normalizeAngle((float)(DIR * PP) * getAngle_Without_track() - zero_electric_angle);
}
```

这一步的意思是：

```text
先用固定电角度把转子强行吸到一个已知位置
然后读取 AS5600 当前机械角度
算出“传感器机械角度”和“真实电角度”的偏差
以后每次都减掉这个偏差
```

而你现在是：

```c
elec_deg = get_vfoc_theta_m_deg() * vfoc_dt.motor_par.pole_pairs;
return FOC_DEG_TO_RAD(elec_deg);
```

你这里只有：

```text
电角度 = 机械角度 × 极对数
```

但是缺了：

```text
电角度 = DIR × 机械角度 × 极对数 - zero_electric_angle
```

你的 `get_vfoc_theta_e_rad()` 目前确实只做了机械角度乘极对数，然后归一化再转弧度，没有 `DIR`，也没有 `zero_electric_angle`。

这个差别非常致命。因为 FOC 的 `Uq` 必须打在正确的 q 轴方向上。如果电角度零点错了，哪怕你位置环 `err` 算对了，`Uq` 也可能打歪，表现就是：

```text
没力
只抖
不吸目标
越控越远
到某些角度才有力
```

---

# 2. 灯哥有 DIR 方向，你现在没有

灯哥有：

```c
int PP = 7;
int DIR = -1;
```

并且两个地方都用了 `DIR`：

```c
_electricalAngle():
DIR * PP * getAngle_Without_track()

位置误差:
motor_target - DIR * Sensor_Angle
```

你现在的位置误差是：

```c
err_v = target_v - curent_v;
uq = kp * err_v;
```

你没有传感器方向补偿。你的控制任务确实是直接 `target_v - curent_v`，然后 `kp * err_v`，再送进 `vfoc_set_svpwm()`。

如果 AS5600 角度增加方向和电机正方向相反，那么你现在就是：

```text
目标在前面，电机却往后推
越推误差越大
闭环变成正反馈
```

这时候你会觉得“怎么没有闭环效果”，实际是方向反了。

---

# 3. 灯哥的 Uq 有限幅，你原来 Uq 直接爆了

灯哥代码是：

```c
float Kp = 0.133;

setPhaseVoltage(
    _constrain(Kp * (motor_target - DIR * Sensor_Angle) * 180 / PI, -6, 6),
    0,
    _electricalAngle()
);
```

也就是说：

```text
Uq 最大只允许 ±6V
```

并且他的 `Kp = 0.133` 是按这个思路来的：

```text
6V / 45° ≈ 0.133 V/deg
```

网上整理的 DengFOC 笔记里也说明了这个逻辑：`Uqmax=±6V`，所以要用 `_constrain()` 把 `Kp*e` 限制在 `[-6, 6]`。([博客园][2])

你原来是：

```c
float kp = 6.5;
uq = kp * err_v;
```

假设误差 50°：

```text
uq = 6.5 × 50 = 325V
```

你的母线才 12V，这个输出早就失真/饱和了。你代码里的 SVPWM 虽然会做限幅，但这会让位置环变成“只要有误差就满输出”，不是正常 P 控制。你的工程里母线是 `12.0f`，控制周期是 1kHz，测试 Uq 是 0.8V，这些参数本来是偏安全的，但 `kp=6.5` 会把闭环输出打爆。

---

# 4. 灯哥的位置角度是弧度累计角度，你现在是 0~360° 单圈角度

灯哥这里：

```c
float Sensor_Angle = getAngle();
```

`getAngle()` 通常是带圈数累计的角度，单位是弧度。也就是说它可以是：

```text
0 rad
6.28 rad
12.56 rad
-6.28 rad
```

可以表示多圈位置。

但是他的电角度用的是：

```c
getAngle_Without_track()
```

也就是单圈机械角度，用来算电角度。

这里分得很清楚：

```text
位置控制误差：用累计机械角度 getAngle()
FOC 电角度：用单圈机械角度 getAngle_Without_track()
```

你现在 `AS5600` 读出来的是：

```c
as5600_get_angle_degrees(as5600, &degrees);
*angle_deg = (float)degrees;
```

也就是 0~360° 单圈角度。

所以你现在做：

```c
target_v = 50;
curent_v = get_vfoc_theta_m_deg();
err_v = target_v - curent_v;
```

只适合单圈内的小范围测试。一旦跨过 0/360°，误差就会炸。例如：

```text
target = 10°
current = 350°
你算出来：err = -340°
真实最近误差：+20°
```

灯哥用累计角度避免了这个问题。你如果暂时只做 0~360° 单圈位置闭环，至少要加最短角度误差函数。

---

# 5. 灯哥每次 loop 都读角度，你现在 100ms 才更新一次角度

灯哥是：

```c
float Sensor_Angle = getAngle();
setPhaseVoltage(...);
```

也就是每次控制前都读当前角度。

你现在是一个单独任务读取 AS5600：

```c
set_vfoc_theta_m_deg(angle);
vTaskDelay(pdMS_TO_TICKS(100));
```

也就是 100ms 更新一次角度。你的 AS5600 任务确实是这样写的。

但你的 FOC 控制环是 1ms：

```c
#define M0_FOC_CTRL_FREQ_HZ 1000
```

也就是：

```text
FOC 控制 1ms 跑一次
角度反馈 100ms 更新一次
```

这个差距太大，闭环基本会变成“拿旧角度猛推”。至少改成 1~2ms 读一次，或者直接在 `m0_foc_control_task()` 里读 AS5600。

---

# 最核心差异总结

| 项目     | 灯哥成功版                                    | 你当前版本                | 影响               |
| ------ | ---------------------------------------- | -------------------- | ---------------- |
| 电角度    | `DIR * PP * angle - zero_electric_angle` | `PP * angle`         | 你缺电角度零偏，FOC 轴可能错 |
| 上电对齐   | 固定电角度吸住 3s，再记录零偏                         | 目前没有等效流程             | 很可能只抖不控          |
| 方向 DIR | 有 `DIR=-1`                               | 没有                   | 可能正反馈，越控越远       |
| Uq 限幅  | `-6V ~ +6V`                              | 原来 `kp=6.5` 可能输出几百 V | 直接饱和             |
| 角度单位   | 位置用弧度累计角度                                | 你用 0~360° 度数         | 跨 0/360° 会错      |
| 反馈刷新   | loop 内实时读                                | 100ms 任务更新           | 反馈太慢             |
| PWM    | LEDC 30kHz 8bit SPWM                     | MCPWM 20kHz SVPWM    | 这不是主要问题          |

---

# 你应该优先补这两个函数

## 1. 增加电角度零偏

在 `vik_foc.c` 里加：

```c
#define VFOC_SENSOR_DIR   (-1.0f)   // 先按灯哥 DIR=-1 试，也可以改 +1
#define VFOC_ALIGN_ANGLE  (4.71238898038f)  // 3PI/2

static float s_zero_electric_angle = 0.0f;

static float normalize_angle_rad(float angle)
{
    float a = fmodf(angle, FOC_2PI);

    if (a < 0.0f)
    {
        a += FOC_2PI;
    }

    return a;
}

static float get_vfoc_theta_m_rad(void)
{
    return FOC_DEG_TO_RAD(get_vfoc_theta_m_deg());
}
```

然后把你的 `get_vfoc_theta_e_rad()` 改成：

```c
float get_vfoc_theta_e_rad(void)
{
    float theta_m_rad;
    float theta_e_rad;

    theta_m_rad = get_vfoc_theta_m_rad();

    theta_e_rad = VFOC_SENSOR_DIR *
                  vfoc_dt.motor_par.pole_pairs *
                  theta_m_rad -
                  s_zero_electric_angle;

    return normalize_angle_rad(theta_e_rad);
}
```

这个就对应灯哥的：

```c
DIR * PP * getAngle_Without_track() - zero_electric_angle
```

---

## 2. 增加“固定电角度输出”函数，用于对齐

你现在的 `vfoc_set_svpwm()` 会自动调用 `get_vfoc_theta_e_rad()`，这不适合做上电对齐。对齐时要像灯哥一样，**强制给一个固定电角度**。

加一个函数：

```c
void vfoc_set_svpwm_with_angle(float uq,
                               float ud,
                               float theta_e_rad,
                               float vbus)
{
    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_t svpwm_status;

    vfoc_dt.park_val.Uq = uq;
    vfoc_dt.park_val.Ud = ud;

    /*
     * 关键：这里不用传感器角度，而是使用指定电角度。
     */
    vfoc_dt.motor_par.theta_e = normalize_angle_rad(theta_e_rad);

    l_temp_clark_v = park_inv_transform(&vfoc_dt);

    vfoc_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    svpwm_status = vfoc_svpwm_calc_duty_uab(
                       &l_temp_clark_v,
                       vbus,
                       &vfoc_dt.motor_drv_val.pwm_duty_val
                   );

    if ((svpwm_status != VFOC_STATUS_OK) &&
        (svpwm_status != VFOC_STATUS_SATURATED))
    {
        ESP_LOGE(TAG, "SVPWM align error, status=%d", (int)svpwm_status);
    }
}
```

然后做一个对齐流程：

```c
void vfoc_align_sensor(void)
{
    pwm_duty_t pwm_duty;
    float theta_m_rad;
    float raw_electric_angle;

    /*
     * 1. 固定电角度输出 3V，把转子吸到指定电角度方向。
     */
    vfoc_set_svpwm_with_angle(3.0f,
                              0.0f,
                              VFOC_ALIGN_ANGLE,
                              MOTOR_DRV_VBUS);

    pwm_duty = vfoc_get_pwm_duty();
    m0_fd6287_set_duty(pwm_duty.duty_Ua,
                       pwm_duty.duty_Ub,
                       pwm_duty.duty_Uc);

    vTaskDelay(pdMS_TO_TICKS(3000));

    /*
     * 2. 读取此时机械角度，计算电角度零偏。
     */
    theta_m_rad = FOC_DEG_TO_RAD(get_vfoc_theta_m_deg());

    raw_electric_angle = normalize_angle_rad(
        VFOC_SENSOR_DIR *
        vfoc_dt.motor_par.pole_pairs *
        theta_m_rad
    );

    s_zero_electric_angle = raw_electric_angle;

    /*
     * 3. 停止输出。
     */
    vfoc_set_svpwm_with_angle(0.0f,
                              0.0f,
                              VFOC_ALIGN_ANGLE,
                              MOTOR_DRV_VBUS);

    pwm_duty = vfoc_get_pwm_duty();
    m0_fd6287_set_duty(pwm_duty.duty_Ua,
                       pwm_duty.duty_Ub,
                       pwm_duty.duty_Uc);

    ESP_LOGI(TAG, "zero_electric_angle = %.4f rad", s_zero_electric_angle);
}
```

注意：这个函数要在 `m0_fd6287_mcpwm_init()` 之后、正式闭环任务启动之前调用。因为它要真的输出 PWM。

启动顺序建议改成：

```c
motor_encoder_init();

vfoc_init();

vbus_en_io_init();
vbus_enable(true);

m0_fd6287_mcpwm_init();

/*
 * 等 AS5600 任务先更新一次角度。
 */
vTaskDelay(pdMS_TO_TICKS(100));

vfoc_align_sensor();

m0_fd6287_foc_start();
```

---

# 你现在最该先做的实验

先不要急着调 PID。你先按灯哥逻辑补齐：

```text
1. DIR
2. zero_electric_angle
3. 上电固定电角度对齐
4. Uq 限幅
5. AS5600 刷新周期改到 1~2ms
```

然后位置环先用：

```c
Kp = 0.02f ~ 0.05f;
Uq_limit = 1.0f ~ 2.0f;
```

不要一上来用 `6.5`。

你现在和灯哥最大的差别一句话总结就是：

**灯哥控制的是“校准后的真实电角度 + 正确方向 + 限幅 Uq”；你现在控制的是“未经电角度零点校准的机械角度×极对数 + 未确认方向 + 可能饱和的 Uq”。**

所以你代码看起来像闭环，但电机不一定收到正确方向的 q 轴力矩。

[1]: https://dengfoc.com/?utm_source=chatgpt.com "DengFOC官方文档"
[2]: https://www.cnblogs.com/fortunely/p/18986181?utm_source=chatgpt.com "FOC算法笔记- 明明1109"


---------
---------
----------

可以，现在已经进入“调手感/刚度”的阶段了。

你现在想要的**明显回弹、难扭动**，本质上就是提高这个位置闭环的“等效弹簧刚度”：

```c
uq = Kp * 位置误差
```

也就是两个参数：

```c
Kp       // 误差转成 Uq 的比例，决定回弹强不强
UQ_LIMIT // 最大 q 轴电压，决定最大扭矩上限
```

你之前工程里母线是 `12V`，PWM 是 20kHz，控制周期是 1kHz，代码里原来测试 Uq 是 `0.8V`，这本来就是很温柔的测试参数。

---

## 先按这个档位调

不要一下子拉很大，建议这样加：

```c
#define POS_KP              0.04f
#define UQ_LIMIT            2.0f
#define POS_DEADBAND_DEG    0.5f
```

如果还软，再上：

```c
#define POS_KP              0.06f
#define UQ_LIMIT            3.0f
```

再想更硬：

```c
#define POS_KP              0.08f
#define UQ_LIMIT            4.0f
```

你的控制代码保持这样：

```c
err_v = angle_error_deg(target_v, current_v);

if (fabsf(err_v) < POS_DEADBAND_DEG)
{
    uq = 0.0f;
}
else
{
    uq = POS_KP * err_v;
    uq = limit_float_local(uq, -UQ_LIMIT, UQ_LIMIT);
}

vfoc_set_svpwm(uq, 0.0f, MOTOR_DRV_VBUS);
```

---

## 调参顺序

你想要“难扭动”的感觉，优先调：

```c
UQ_LIMIT
```

比如从：

```c
1.2V -> 2.0V -> 3.0V -> 4.0V
```

这个会明显增加最大保持力。

然后再调：

```c
POS_KP
```

比如：

```c
0.02 -> 0.04 -> 0.06 -> 0.08
```

`Kp` 决定你偏离目标角度一点点时，电机会不会立刻强烈回弹。

---

## 推荐你先用这个版本

```c
#define POS_KP              0.05f
#define UQ_LIMIT            3.0f
#define POS_DEADBAND_DEG    0.3f
```

这个大概率会比你现在硬很多，但还不算特别危险。

如果要仿灯哥那种比较明显的手感，可以试：

```c
#define POS_KP              0.10f
#define UQ_LIMIT            5.0f
#define POS_DEADBAND_DEG    0.2f
```

但这个要注意电机和驱动发热。

---

## 很重要：电压模式不是电流/力矩闭环

你现在这个是：

```text
位置环 P -> Uq 电压
```

不是：

```text
位置环 -> 速度环 -> 电流环 -> Iq
```

真正的扭矩主要跟 `Iq` 有关，但你现在没有采样相电流，所以只能用 `Uq` 粗略增加扭矩。你的 `vfoc_set_svpwm()` 也是把 `Uq/Ud` 经过 Park 逆变换、SVPWM 转成三相 duty 输出。

所以现象会是：

```text
Uq 增大 -> 手感更硬
但电流、发热、啸叫也会变大
```

---

## 你这个 2208 电机本身保持力不会特别夸张

你代码注释里电机参数写的是：

```text
2208
12V
2A
14W
7 极对数
电阻 21.2Ω
```

这种小云台/无刷电机，电阻比较大，电流不会像低阻大电机那样猛，所以“锁住很难拧动”的极限有限。

如果母线 12V，电阻 21.2Ω，粗略估算静止堵转相电流也不会特别大：

```text
I ≈ U / R
```

所以你会感觉它能回弹，但不一定像伺服电机那样死死锁住。

---

## 判断调大是否安全

每次加大后，摸这几个地方：

```text
1. 电机是否快速发热
2. FD6287/MOS 是否发热
3. 电源电流是否明显变大
4. 是否有尖锐啸叫
5. 手扭时是否抖动严重
```

如果出现：

```text
电机很烫
驱动很烫
电源电流飙升
电机尖叫
位置附近疯狂抖
```

就把 `UQ_LIMIT` 降下来。

---

## 你下一步建议参数

先直接用这个：

```c
#define POS_KP              0.05f
#define UQ_LIMIT            3.0f
#define POS_DEADBAND_DEG    0.5f
```

如果手感还软：

```c
#define POS_KP              0.08f
#define UQ_LIMIT            4.0f
#define POS_DEADBAND_DEG    0.3f
```

如果开始抖：

```c
POS_KP 降一点
POS_DEADBAND_DEG 加大一点
```

一句话：**想要更硬，先加 `UQ_LIMIT`，再加 `POS_KP`；没有电流采样前，不建议长期超过 4~5V 的 Uq 保持。**
