`Kp = 6.5` **有一定道理，但不能直接照抄**。它科学不科学，关键看这几个前提：

```text
1. 你的误差单位是不是“弧度”
2. PID 输出是不是 Uq 电压，单位 V
3. 你有没有做 Uq 限幅
4. 你的电机、驱动、电源能不能承受这个电压
```

---

## 灯哥这个思路大概是这样来的

位置 P 控制最简单可以写成：

$$
U_q = K_p \cdot e_\theta
$$
其中：

```text
Uq       = q 轴电压，也可以先理解成“力矩大小”
Kp       = 位置比例系数
eθ       = 目标角度 - 当前角度
```

如果你的母线电压是：

```c
VBUS = 12V
```

然后希望当角度误差是 `45°` 时，输出大约一半母线电压：

```c
Uq = 6V
```

因为：

```text
45° = π / 4 rad ≈ 0.785 rad
```

所以：

```c
Kp = 6.0f / 0.785f = 7.64f
```

所以如果他说 `Kp = 6.5`，大概就是这个量级。

也就是说：

```text
Kp = 6.5 V/rad
```

这个数字不是乱拍脑袋，它背后的意思是：

```text
误差大概 45° 时，Uq 输出大约 5V 左右
```

因为：

```c
Uq = 6.5f * 0.785f = 5.10V
```

所以从这个角度看，**Kp = 6.5 不是完全不科学**。

---

## 但是！如果你的误差单位是“度”，那 Kp=6.5 就离谱了

如果你代码里是这样：

```c
error_deg = target_deg - current_deg;
uq = kp * error_deg;
```

那 `Kp = 6.5` 就会变成：

```text
误差 1°，Uq = 6.5V
误差 2°，Uq = 13V
误差 45°，Uq = 292.5V
```

这肯定不对。

所以必须先明确：

```text
Kp = 6.5 适合 “误差单位是弧度”
不适合 “误差单位是角度 degree”
```

如果你用的是角度制，那么等效 Kp 应该是：

```c
Kp_deg = Kp_rad * PI / 180
```

比如：

```c
Kp_rad = 6.5f;
Kp_deg = 6.5f * 3.1415926f / 180.0f;
```

结果是：

```c
Kp_deg ≈ 0.113f V/deg
```

也就是说：

```text
6.5 V/rad ≈ 0.113 V/deg
```

---

## 更科学的 Kp 设置方法

你可以按这个方法来，不要直接背参数。

### 方法 1：根据“最大允许力矩电压”和“最大角度误差”计算

公式：

```c
Kp = Uq_limit / error_limit_rad;
```

比如你现在是 12V 母线，但是学习阶段不建议一上来用 6V。你可以先限制：

```c
Uq_limit = 1.0V
```

假设你希望误差达到 `45°` 时，输出就达到最大力矩：

```c
error_limit_rad = 45° = 0.785 rad
```

那么：

```c
Kp = 1.0f / 0.785f = 1.27f
```

如果你想最大输出是 `2V`：

```c
Kp = 2.0f / 0.785f = 2.55f
```

如果你想最大输出是 `3V`：

```c
Kp = 3.0f / 0.785f = 3.82f
```

如果你想最大输出是 `6V`：

```c
Kp = 6.0f / 0.785f = 7.64f
```

所以可以得到这个表：

| Uq 限幅 | 45° 误差对应 Kp，单位 V/rad |
| ----: | -------------------: |
|  0.5V |                 0.64 |
|  1.0V |                 1.27 |
|  2.0V |                 2.55 |
|  3.0V |                 3.82 |
|  5.0V |                 6.37 |
|  6.0V |                 7.64 |

所以 `Kp = 6.5` 大概对应：

```text
45° 误差时输出 5.1V 左右
```

这对一些小电机可能已经偏猛了。

---

## 我建议你现在先这样设

你现在 ESP32 + AS5600 + FOC 学习阶段，先不要上来 `Kp = 6.5`。

建议先用：

```c
Kp = 1.0f;          // 单位 V/rad
torque_limit = 1.0f;
```

然后慢慢加：

```c
Kp = 1.5f;
Kp = 2.0f;
Kp = 3.0f;
```

力矩限制先这样：

```c
torque_limit_v = 0.5f;
torque_limit_v = 1.0f;
torque_limit_v = 1.5f;
torque_limit_v = 2.0f;
```

学习阶段我建议先别超过：

```c
torque_limit_v = 2.0f;
```

尤其是你没有电流采样的时候。

---

## 推荐你代码里统一用弧度误差

位置误差这样算：

```c
#define FOC_PI 3.14159265358979323846f

static float wrap_angle_error_rad(float error_rad)
{
    while (error_rad > FOC_PI)
    {
        error_rad -= 2.0f * FOC_PI;
    }

    while (error_rad < -FOC_PI)
    {
        error_rad += 2.0f * FOC_PI;
    }

    return error_rad;
}
```

然后位置 P 控制：

```c
float foc_position_p_control(float target_deg,
                             float current_deg,
                             float kp,
                             float torque_limit_v)
{
    float target_rad;
    float current_rad;
    float error_rad;
    float uq;

    target_rad = target_deg * FOC_PI / 180.0f;
    current_rad = current_deg * FOC_PI / 180.0f;

    error_rad = target_rad - current_rad;
    error_rad = wrap_angle_error_rad(error_rad);

    uq = kp * error_rad;

    if (uq > torque_limit_v)
    {
        uq = torque_limit_v;
    }
    else if (uq < -torque_limit_v)
    {
        uq = -torque_limit_v;
    }

    return uq;
}
```

调用示例：

```c
float target_angle_deg = 45.0f;
float current_angle_deg = 0.0f;
float uq = 0.0f;

float kp = 1.5f;
float torque_limit_v = 1.0f;

uq = foc_position_p_control(target_angle_deg,
                            current_angle_deg,
                            kp,
                            torque_limit_v);
```

---

## 最重要：Kp 和 torque_limit 要一起看

只看 `Kp` 没意义。

比如：

```c
Kp = 6.5f;
torque_limit_v = 1.0f;
```

那只要误差超过：

```c
1.0 / 6.5 = 0.154 rad ≈ 8.8°
```

输出就已经饱和到 `1V` 了。

所以这个时候 `Kp = 6.5` 实际效果是：

```text
误差超过 8.8°，都是满力矩 1V
误差小于 8.8°，才有线性调节
```

如果：

```c
Kp = 6.5f;
torque_limit_v = 6.0f;
```

那就比较猛了：

```text
45° 误差时 Uq ≈ 5.1V
```

电机会很硬，也可能抖、发热、啸叫。

---

## 更“科学”的调参流程

你可以这样做：

### 第一步：固定力矩限制

先固定：

```c
torque_limit_v = 1.0f;
```

### 第二步：从小 Kp 开始

```c
Kp = 0.5f;
```

观察电机能不能回目标位置。

### 第三步：逐渐增加 Kp

```c
Kp = 1.0f;
Kp = 1.5f;
Kp = 2.0f;
Kp = 3.0f;
```

观察：

```text
回位太慢：Kp 加大
来回抖动：Kp 太大
发热明显：torque_limit 太大
手感太软：torque_limit 加大
手感太硬：torque_limit 减小
```

### 第四步：如果 P 控制抖，加 D

位置控制只用 P，很容易像弹簧一样来回震。

你后面可以加一个速度阻尼项：

```c
Uq = Kp * angle_error_rad - Kd * velocity_rad_s;
```

这就像：

```text
Kp = 弹簧力度
Kd = 阻尼刹车
```

没有 `Kd`，它容易弹来弹去。

---

## 结论

`Kp = 6.5` **不是完全不科学**，但它隐含条件是：

```text
误差单位是 rad
输出单位是 V
Uq 有限幅
目标是让几十度误差时输出几伏电压
```

对你现在来说，更建议先这样：

```c
kp = 1.0f;              // V/rad
torque_limit_v = 1.0f;  // V
```

然后慢慢调到：

```c
kp = 2.0f ~ 4.0f;
torque_limit_v = 1.0f ~ 2.0f;
```

等你确认电机不会乱抖、不会发热，再考虑接近：

```c
kp = 6.5f;
torque_limit_v = 3.0f 或更高
```

一句话总结：

```text
科学设置 Kp 的方法不是“看别人填多少”，
而是先定你允许最大 Uq，
再定你希望多大角度误差达到这个 Uq，
然后 Kp = Uq_limit / error_limit_rad。
```
