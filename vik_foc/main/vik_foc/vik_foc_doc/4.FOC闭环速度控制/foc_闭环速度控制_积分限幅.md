你现在 **PI 没咔咔，加了 Kd 就咔咔咔**，原因很明确：

> **速度环里的 D 项会把速度反馈噪声、采样抖动、dt 抖动全部放大，最后让 Uq 瞬间正负跳变，电机就会咔咔咔。**

你当前代码里是这样算的：

```c
kd_err_parm = err_motor_rpm - last_err_motor_rpm;

uq = (kp * err_motor_rpm)
   + (ki * ki_err_sum * dt_s)
   + (kd * kd_err_parm / dt_s);
```

而且你现在速度目标是 `600rpm`，`UQ_LIMIT = 3.2f`，`ki_err_sum += err_motor_rpm` 没有积分限幅，最后才把 `uq` 限制到 `±3.2V`。

---

# 1. 为啥一加 Kd 就咔咔咔？

## 核心原因：`/ dt_s` 会把误差变化放大很多倍

假设你的控制周期是 1ms：

```c
dt_s = 0.001f;
```

如果速度反馈只抖了 `10rpm`，那么 D 项里的变化率就是：

```c
10 / 0.001 = 10000
```

如果你设置：

```c
kd = 0.001f;
```

那么 D 项输出就是：

```c
0.001 * 10000 = 10V
```

但是你的 `UQ_LIMIT` 只有：

```c
#define UQ_LIMIT 3.2f
```

所以实际会变成：

```text
Uq = +3.2V
Uq = -3.2V
Uq = +3.2V
Uq = -3.2V
```

这个就是电机咔咔咔的直接来源。

---

## 你的速度值本身就是容易抖的

你的速度来自：

```c
now_motor_rpm = get_vfoc_mech_rpm();
```

这种速度一般是通过：

```text
角度差 / 时间
```

算出来的。低速、中速阶段，角度采样有一点点跳动，速度值就会跳。

PI 对这种跳动还能忍，D 项对这种跳动非常敏感。

所以速度环里面：

```text
P：看当前速度误差
I：慢慢补偿长期误差
D：专门看误差变化速度
```

D 项最怕的就是速度反馈毛刺。速度一毛刺，D 项就猛打一脚。

---

# 2. 速度环有必要加 Kd 吗？

**你现在这个阶段不建议加 Kd。**

FOC 速度环一般用：

```text
速度环：PI
电流环：PI
位置环：P / PD / PID
```

速度环里 D 项不是不能用，而是要求比较高：

```text
1. 速度反馈必须很干净
2. 控制周期必须稳定
3. D项必须加低通滤波
4. D输出必须限幅
5. Kd 必须非常非常小
```

你现在是初学阶段，而且没有电流环，速度环直接输出 `Uq`，所以最推荐：

```c
kd = 0.0f;
```

这个不是偷懒，是正常做法。

---

# 3. 你现在有必要做积分限幅吗？

**非常有必要。**

你现在代码是：

```c
ki_err_sum += err_motor_rpm;
```

然后：

```c
uq = P + I + D;
uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
```

问题是：

```text
Uq 已经被限制到 ±3.2V 了
但是 ki_err_sum 还在一直累加
```

这会导致积分越积越大，最后出现：

```text
启动猛冲
速度过冲
转速一高一低
突然卡顿一下
电机反向顶一下
停下来后还继续用力
```

这个现象叫 **积分饱和 / 积分 windup**。

---

# 4. 你的积分还有一个小问题

你现在是：

```c
ki_err_sum += err_motor_rpm;

uq = kp * err_motor_rpm
   + ki * ki_err_sum * dt_s;
```

更标准的写法应该是：

```c
ki_err_sum += err_motor_rpm * dt_s;

i_out = ki * ki_err_sum;
```

也就是：

```text
积分量 = 误差 × 时间
```

你现在是先累加误差，最后再乘一次 `dt_s`，在 `dt_s` 稳定时勉强能跑，但如果控制周期有抖动，就不太规范。

---

# 5. 最简单的积分限幅怎么做？

推荐你限幅的是 **I 项输出电压**，而不是随便限制 `ki_err_sum`。

比如你的 `UQ_LIMIT = 3.2V`，那可以让积分项最多贡献：

```c
#define SPEED_I_OUT_LIMIT  1.0f
```

意思是：

```text
总 Uq 最大 ±3.2V
其中积分最多只能占 ±1.0V
剩下主要由 P 项负责
```

---

## 推荐修改版

把原来的：

```c
ki_err_sum += err_motor_rpm;

if (fabsf(err_motor_rpm) < SPEED_DEADBAND_RPM)
{
    err_motor_rpm = 0.0f;
}

kd_err_parm = err_motor_rpm - last_err_motor_rpm;

uq = (kp * err_motor_rpm) +(ki * ki_err_sum * dt_s)+ (kd * kd_err_parm / dt_s);
uq *= -1;

uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);
```

改成这个：

```c
#define SPEED_I_OUT_LIMIT  1.0f   // 积分项最多贡献 ±1.0V
#define SPEED_D_OUT_LIMIT  0.3f   // 如果以后非要用D，D项最多贡献 ±0.3V

/*
 * 1. 获取当前速度
 */
now_motor_rpm = get_vfoc_mech_rpm();

/*
 * 2. 计算速度误差
 */
err_motor_rpm = exp_motor_rpm - now_motor_rpm;

/*
 * 3. 速度死区
 *    注意：死区要放在积分前面。
 */
if (fabsf(err_motor_rpm) < SPEED_DEADBAND_RPM)
{
    err_motor_rpm = 0.0f;

    /*
     * 误差很小时，让积分慢慢释放，避免残留积分导致来回顶。
     */
    ki_err_sum *= 0.98f;
}
else
{
    /*
     * 4. 正确积分：误差 * 时间
     */
    ki_err_sum += err_motor_rpm * dt_s;
}

/*
 * 5. P项
 */
float p_out = kp * err_motor_rpm;

/*
 * 6. I项
 */
float i_out = ki * ki_err_sum;

/*
 * 7. 限制I项输出，防止积分饱和
 */
i_out = limit_float(i_out, -SPEED_I_OUT_LIMIT, SPEED_I_OUT_LIMIT);

/*
 * 8. 反推积分值，防止 ki_err_sum 内部继续无限变大
 */
if (ki > 0.000001f)
{
    ki_err_sum = i_out / ki;
}

/*
 * 9. D项，当前建议先不用
 */
float d_out = 0.0f;

/*
 * 如果你非要测试D，必须这样限幅。
 * 但我建议速度环先保持 kd = 0。
 */
kd_err_parm = err_motor_rpm - last_err_motor_rpm;

if (dt_s > 0.0001f)
{
    d_out = kd * kd_err_parm / dt_s;
    d_out = limit_float(d_out, -SPEED_D_OUT_LIMIT, SPEED_D_OUT_LIMIT);
}

/*
 * 10. PID输出
 */
uq = p_out + i_out + d_out;

/*
 * 11. 保留你的方向修正
 */
uq *= -1.0f;

/*
 * 12. 总Uq限幅
 */
uq = limit_float(uq, -UQ_LIMIT, UQ_LIMIT);

/*
 * 13. 更新上一次误差
 */
last_err_motor_rpm = err_motor_rpm;
```

---

# 6. 参数建议

你现在代码里：

```c
float kp = 0.02f;
float ki = 0.06f;
float kd = 0.0f;
```

对于你这个“速度环直接输出 Uq”的控制方式，我建议先这样：

```c
float kp = 0.006f;
float ki = 0.003f;
float kd = 0.0f;
```

`UQ_LIMIT = 3.2V` 可以先保留，但如果震动大，先降到：

```c
#define UQ_LIMIT  2.0f
```

更稳。

---

# 7. 非要加 Kd，应该怎么加？

先说结论：**速度环暂时不要加 D。**

但你要学习，可以这样加，不能用原始误差直接做 D。

## 错误写法

```c
d_out = kd * (err_motor_rpm - last_err_motor_rpm) / dt_s;
```

这个最容易咔咔咔。

---

## 稍微好一点的写法：D 项限幅

```c
float d_out = kd * (err_motor_rpm - last_err_motor_rpm) / dt_s;
d_out = limit_float(d_out, -0.2f, 0.2f);
```

`D` 项最多只允许影响 `±0.2V`，不要让它一脚踹出几伏。

---

## 更推荐：对速度做滤波，不对原始速度做 D

先做速度低通：

```c
static float rpm_filt = 0.0f;
static bool rpm_filt_init = false;

if (rpm_filt_init == false)
{
    rpm_filt = now_motor_rpm;
    rpm_filt_init = true;
}
else
{
    float alpha = 0.05f;  // 越小越平滑
    rpm_filt += alpha * (now_motor_rpm - rpm_filt);
}
```

然后误差用滤波后的速度：

```c
err_motor_rpm = exp_motor_rpm - rpm_filt;
```

这样即使你以后加一点点 D，也不会那么容易炸。

---

# 8. 你当前最推荐的最终结构

你的速度环现在先别做 PID，做这个：

```text
速度误差
  ↓
死区
  ↓
P项
  ↓
I项 + 积分限幅
  ↓
Uq总限幅
  ↓
Uq斜坡
  ↓
SVPWM
```

也就是：

```text
速度环 PI 就够了
Kd = 0
```

---

# 9. 最终建议

你现在这样设置：

```c
#define UQ_LIMIT             2.0f
#define SPEED_I_OUT_LIMIT    0.8f

float kp = 0.006f;
float ki = 0.003f;
float kd = 0.0f;
```

等这个能稳定跑，再尝试：

```c
#define UQ_LIMIT             3.2f
#define SPEED_I_OUT_LIMIT    1.0f

float kp = 0.008f;
float ki = 0.004f;
float kd = 0.0f;
```

**不要急着加 Kd。速度环 PI 没咔咔，就说明方向是对的。你一加 D 就咔咔，本质就是 D 项把速度采样噪声放大成了 Uq 抖动。**

你的积分限幅一定要做，最简单可靠的方式就是：**限制 I 项输出电压，比如限制到 ±0.8V 或 ±1.0V，然后反推 `ki_err_sum = i_out / ki`。**
