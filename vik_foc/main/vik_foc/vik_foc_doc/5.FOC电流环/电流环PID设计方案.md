这是一个非常关键的问题。

**如果是工业级 FOC（TI、ST、Microchip、Infineon、ODrive、VESC 等），电流环几乎都是用 PI 控制器，而不是完整 PID。**

## 为什么不用 PID？

FOC 的 d/q 电流模型：

$$
V_q = Ri_q + L\frac{di_q}{dt}+\omega_e\lambda
$$

控制对象本身已经包含了一个积分（电感 L）。

如果 PID 再加一个 D（微分）：

```
PID

      ----P----
     |         |
e --->         +----> 输出
     |         |
      ----I----
     |         |
      ----D----
```

D 项会：

* 放大 ADC 噪声
* 放大电流采样毛刺
* 放大PWM纹波
* 导致输出抖动
* 高速时非常难调

而电流采样本来就是：

```
ADC
 ↓
运放
 ↓
PWM纹波
 ↓
采样误差
```

里面已经有不少高频噪声。

所以 D 项几乎没有收益。

---

## 为什么 PI 就够了？

FOC 电流环本质就是控制 RL 电路。

RL 系统：

```
      1
------------
Ls + R
```

它本身就是一阶系统。

PI 正好可以：

* 消除稳态误差
* 提高响应速度
* 保证稳定性

所以：

```
Iq_ref
   │
   ▼
 +--------+
 |   PI   |
 +--------+
   │
   ▼
Vq
```

工业上几乎都是这样。

---

## 标准FOC框图

```
Iq_ref
   │
   ▼
 PI(q)
   │
   ▼
Vq

Id_ref
   │
   ▼
 PI(d)
   │
   ▼
Vd
```

两个独立 PI：

```
Id PI
Iq PI
```

最后：

```
Vd,Vq
    │
Inverse Park
    │
αβ
    │
SVPWM
```

---

# PI公式

连续形式：

$$
u(t)=K_pe(t)+K_i\int e(t)dt
$$

离散形式（FOC常用）：

```c
integral += Ki * err * Ts;

out = Kp * err + integral;
```

其中：

```
Ts = 100us
```

如果 PWM：

```
10kHz

Ts = 0.0001
```

---

## 推荐加入积分限幅（Anti-Windup）

工业代码都会这样：

```c
integral += Ki * err * Ts;

if(integral > max)
    integral = max;

if(integral < -max)
    integral = -max;

out = Kp * err + integral;
```

否则：

突然堵转

↓

积分一直累计

↓

恢复以后

↓

疯狂过冲。

---

## 再加输出限幅

例如：

```c
out = Kp * err + integral;

if(out > VBUS)
    out = VBUS;

if(out < -VBUS)
    out = -VBUS;
```

通常不是直接限制到母线电压，而是根据调制方式限制可用电压矢量，例如 SVPWM 下会留出一定裕量，避免进入过调制。

---

# 增量式还是位置式？

FOC **几乎全部使用位置式 PI（Positional PI）**。

例如：

```c
float err = ref - feedback;

integral += Ki * err * Ts;

output = Kp * err + integral;
```

很少使用：

```
增量PI
```

因为：

FOC 每个 PWM 周期都会重新计算：

```
Iq
↓

PI

↓

Vq
```

直接计算当前输出即可，不需要关心上一次输出的增量。

---

## 电流环推荐结构

对于你的项目（ESP32 + 10 kHz PWM + 霍尔/磁编码器 FOC），建议采用如下流程：

```text
ADC采样Ia、Ib
      │
      ▼
Clarke
      │
      ▼
Park
      │
      ▼
得到 Id、Iq
      │
      ├────► PI(Id) ───► Vd
      │
      └────► PI(Iq) ───► Vq
                     │
                     ▼
                反Park
                     │
                     ▼
                  SVPWM
```

每个 PWM 周期执行一次 PI 控制即可。

---

## 我建议你实现的 PI 功能

结合你目前正在开发的 ESP32 FOC 项目，一个比较完整且实用的电流环 PI 模块应包括：

* 位置式 PI。
* 积分限幅（Anti-Windup）。
* 输出限幅（限制 `Vd`、`Vq` 的范围）。
* 支持浮点运算（ESP32 的单精度浮点性能足以应对 10 kHz 电流环）。
* `Id` 和 `Iq` 使用相同的 PI 结构体，各自维护独立的参数和积分状态。

这样已经接近很多工业 FOC 软件框架（如 TI C2000、ST MCSDK、VESC 等）的电流环实现方式。对于你当前的平衡车和 FOC 学习项目，这是一个合适且成熟的方案。


----------
-----------
---------

