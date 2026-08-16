别懵！七段式SVPWM看着公式多，其实**核心就4步固定流程**，全是加减乘除，连三角函数都不用。我结合文档里的标准实现，给你拆成「大白话讲解+逐行注释代码」，照着抄就能跑。

先给你吃定心丸：七段式是SVPWM里最通用、最不容易出问题的写法，工业界90%的FOC都用它，代码逻辑非常固定，不用自己瞎创新。

---

### 一、先搞懂输入输出

你不用管底层原理，先记住这个函数的「输入和输出」：

- **输入**：`U_alpha`、`U_beta`（两相静止坐标系电压，Park反变换输出的结果）
- **输入**：`U_dc`（母线电压，ADC采回来的直流母线电压）
- **输入**：`T_pwm`（PWM周期的计数器值，比如16kHz下STM32的ARR值）
- **输出**：`CCR_A`、`CCR_B`、`CCR_C`（三路PWM的比较值，直接丢给定时器寄存器就行）

整个算法就是：给我两个电压值，我算出来三路PWM占空比。

---

### 二、四步走：七段式SVPWM完整流程

#### 第1步：判断电压矢量落在哪个扇区（6个扇区6选1）

这一步**没有三角函数**，就是算3个中间值，看它们的正负，拼出一个编号，查表得到扇区。

对应文档第4.2节的公式：
$$
\begin{cases}
U_1 = U_\beta \
U_2 = \frac{\sqrt{3}U_\alpha - U_\beta}{2} \
U_3 = \frac{-\sqrt{3}U_\alpha - U_\beta}{2}
\end{cases}
$$

规则：值大于0就记1，小于0就记0。

- A = U1>0 ? 1 : 0
- B = U2>0 ? 1 : 0
- C = U3>0 ? 1 : 0

然后算 `N = A + 2*B + 4*C`，N一共6种有效取值，对应6个扇区：

| N值 | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| 扇区 | 2 | 6 | 1 | 4 | 3 | 5 |

> 
> 大白话：就像给你个坐标，判断它在六个三角形的哪一块里，用正负号判断比算角度快100倍。

#### 第2步：计算两个有效矢量的作用时间

每个扇区都由两个相邻的基础矢量合成，我们要算出这两个矢量各自作用多久，记为 `Tx` 和 `Ty`。

先算一个公共系数K：
$$ K = \frac{\sqrt{3} \times T_{pwm}}{U_{dc}} $$

然后每个扇区的Tx、Ty计算公式不一样，直接查表代入：

| 扇区 | Tx计算公式 | Ty计算公式 |
| --- | --- | --- |
| 扇区1 | U2 * K | U1 * K |
| 扇区2 | -U2 * K | -U3 * K |
| 扇区3 | U1 * K | -U3 * K |
| 扇区4 | -U1 * K | -U2 * K |
| 扇区5 | U3 * K | U2 * K |
| 扇区6 | -U3 * K | -U1 * K |

**过调制保护**：如果 `Tx + Ty > T_pwm`，说明电压超上限了，要等比例缩小，不然输出会失真：
$$ Tx = \frac{Tx}{Tx+Ty} \times T_{pwm} $$
$$ Ty = \frac{Ty}{Tx+Ty} \times T_{pwm} $$

#### 第3步：计算三相基准占空比（七段式核心）

这一步就是七段式和五段式的区别：**七段式把零矢量时间平分，对称分布在周期首尾和中间**，所以波形对称、谐波小。

对应文档第4.5节PWM模式1（中心对齐模式，最常用）的公式：
$$
\begin{cases}
T_a = \frac{T_{pwm} + Tx + Ty}{4} \
T_b = T_a - \frac{Tx}{2} \
T_c = T_b - \frac{Ty}{2}
\end{cases}
$$

> 
> 大白话：Ta、Tb、Tc是三个「时间基准值」，分别对应占空比从大到小的三个相，还没分配给ABC三相。

#### 第4步：按扇区把基准值分配给ABC三相

因为每个扇区里哪相电压最高、哪相最低是不一样的，所以要把Ta/Tb/Tc按扇区对应到CCR_A/CCR_B/CCR_C。

对应文档表4.4，直接查表赋值：

| 扇区 | CCR_A | CCR_B | CCR_C |
| --- | --- | --- | --- |
| 扇区1 | Ta | Tb | Tc |
| 扇区2 | Tb | Ta | Tc |
| 扇区3 | Tc | Ta | Tb |
| 扇区4 | Tc | Tb | Ta |
| 扇区5 | Tb | Tc | Ta |
| 扇区6 | Ta | Tc | Tb |

到这里就算完了，把三个CCR值丢给定时器，七段式SVPWM就输出了。

---

### 三、直接抄：带逐行注释的C语言代码

这是完全对应文档实现的标准七段式SVPWM函数，STM32上直接能用，变量名和上面讲的一一对应。

```
#include <stdint.h>

// 提前定义好根号3，用浮点就行，F4的FPU跑得飞快
#define SQRT3   1.73205080757f

/**
 * @brief  七段式SVPWM计算函数
 * @param  U_alpha:  alpha轴电压 (单位:V)
 * @param  U_beta:   beta轴电压  (单位:V)
 * @param  U_dc:     母线电压    (单位:V)
 * @param  T_pwm:    PWM周期对应的定时器计数值 (比如ARR=5250)
 * @param  pwm_ccr:  输出数组，存放三路CCR值 [CCR_A, CCR_B, CCR_C]
 */
void svpwm_7segment(float U_alpha, float U_beta, float U_dc, uint16_t T_pwm, uint16_t *pwm_ccr)
{
    float U1, U2, U3;
    uint8_t A, B, C, N, sector;
    float K;
    float Tx, Ty;
    float Ta, Tb, Tc;
    float temp;

    // ========== 第1步：计算3个中间量，判断扇区 ==========
    U1 = U_beta;
    U2 = (SQRT3 * U_alpha - U_beta) * 0.5f;
    U3 = (-SQRT3 * U_alpha - U_beta) * 0.5f;

    // 正负号判断
    A = (U1 > 0.0f) ? 1 : 0;
    B = (U2 > 0.0f) ? 1 : 0;
    C = (U3 > 0.0f) ? 1 : 0;

    // 计算N值，查表得到扇区号(1~6)
    N = A + 2*B + 4*C;
    switch(N)
    {
        case 3:  sector = 1; break;
        case 1:  sector = 2; break;
        case 5:  sector = 3; break;
        case 4:  sector = 4; break;
        case 6:  sector = 5; break;
        case 2:  sector = 6; break;
        default: sector = 1; break; // 异常保护
    }

    // ========== 第2步：计算矢量作用时间Tx、Ty ==========
    K = SQRT3 * T_pwm / U_dc; // 公共系数

    switch(sector)
    {
        case 1: Tx = U2 * K;  Ty = U1 * K;  break;
        case 2: Tx = -U2 * K; Ty = -U3 * K; break;
        case 3: Tx = U1 * K;  Ty = -U3 * K; break;
        case 4: Tx = -U1 * K; Ty = -U2 * K; break;
        case 5: Tx = U3 * K;  Ty = U2 * K;  break;
        case 6: Tx = -U3 * K; Ty = -U1 * K; break;
        default:Tx = 0; Ty = 0; break;
    }

    // 过调制处理：时间超了就等比例压缩
    if(Tx + Ty > T_pwm)
    {
        temp = Tx + Ty;
        Tx = Tx / temp * T_pwm;
        Ty = Ty / temp * T_pwm;
    }

    // ========== 第3步：七段式基准占空比计算 ==========
    Ta = (T_pwm + Tx + Ty) * 0.25f;  // 占空比最大的相
    Tb = Ta - Tx * 0.5f;             // 中间的相
    Tc = Tb - Ty * 0.5f;             // 占空比最小的相

    // ========== 第4步：按扇区分配给ABC三相 ==========
    switch(sector)
    {
        case 1:
            pwm_ccr[0] = (uint16_t)Ta;  // A相
            pwm_ccr[1] = (uint16_t)Tb;  // B相
            pwm_ccr[2] = (uint16_t)Tc;  // C相
            break;
        case 2:
            pwm_ccr[0] = (uint16_t)Tb;
            pwm_ccr[1] = (uint16_t)Ta;
            pwm_ccr[2] = (uint16_t)Tc;
            break;
        case 3:
            pwm_ccr[0] = (uint16_t)Tc;
            pwm_ccr[1] = (uint16_t)Ta;
            pwm_ccr[2] = (uint16_t)Tb;
            break;
        case 4:
            pwm_ccr[0] = (uint16_t)Tc;
            pwm_ccr[1] = (uint16_t)Tb;
            pwm_ccr[2] = (uint16_t)Ta;
            break;
        case 5:
            pwm_ccr[0] = (uint16_t)Tb;
            pwm_ccr[1] = (uint16_t)Tc;
            pwm_ccr[2] = (uint16_t)Ta;
            break;
        case 6:
            pwm_ccr[0] = (uint16_t)Ta;
            pwm_ccr[1] = (uint16_t)Tc;
            pwm_ccr[2] = (uint16_t)Tb;
            break;
        default:
            pwm_ccr[0] = T_pwm/2;
            pwm_ccr[1] = T_pwm/2;
            pwm_ccr[2] = T_pwm/2;
            break;
    }
}
```

---

### 四、小白必看：怎么对接STM32定时器

1. **定时器配置**：用高级定时器TIM1/TIM8，配置成**中央对齐模式1**，开启互补输出，打开死区。
2. **调用时机**：在PWM更新中断（或者ADC注入转换完成中断）里调用这个函数。
3. **赋值寄存器**：计算完之后直接写寄存器：```
uint16_t ccr[3];
svpwm_7segment(v_alpha, v_beta, v_bus, 5250, ccr);
TIM8->CCR1 = ccr[0];
TIM8->CCR2 = ccr[1];
TIM8->CCR3 = ccr[2];
```

---

### 五、小白调试技巧（对应文档8.11节）

不用接电机，先验证算法对不对：

1. 手动给一个固定角度，让角度慢慢累加，比如 `angle += 10` 每个周期。
2. 用Park反变换算出U_alpha、U_beta，喂给SVPWM函数。
3. 用示波器看三相PWM的低通滤波波形，应该是**三个相位差120°的马鞍波**，这就说明七段式SVPWM写对了。
4. 测线电压的滤波波形，应该是标准正弦波，就没问题了。

---

### 六、常见坑点提醒

1. **单位要统一**：U_alpha、U_beta、U_dc单位都是伏特，别一个用标幺值一个用实际值。
2. **PWM模式别搞反**：代码是PWM模式1（CNT<CCR有效），如果配置成模式2，占空比会反过来，电机不会转。
3. **中心对齐才是七段式**：边沿对齐模式出不来标准七段式波形，一定要开中心对齐。
4. **零矢量平分**：七段式的精髓就是零矢量拆成两半对称放，这也是它谐波比五段式小的原因。

需要我再补一个「定点版（Q15格式）」的SVPWM代码，给没有FPU的低端MCU用吗？



--------------------
---------------------
-----------------
你既然确定写**经典七段式SVPWM（Sector + T1/T2 + T0）**，那我们就完全按照书里的思路来，不走零序注入。

这东西第一次看懵很正常，因为书里面把**空间几何、公式推导、PWM波形**混在一起了。程序实现其实就是固定流程：

```
Uα/Uβ
  |
  ↓
判断扇区 Sector
  |
  ↓
计算 T1、T2
  |
  ↓
计算 T0
  |
  ↓
根据扇区排列七段式
  |
  ↓
得到 Ta Tb Tc
  |
  ↓
写PWM CCR
```

书里面第4章就是这个流程：基础矢量 → 扇区判断 → 作用时间 → 七段式 → PWM占空比。

---

# 一、先明确输入输出

FOC出来：

```c
float Ualpha;
float Ubeta;
```

例如：

```
Ualpha = 3.0V
Ubeta  = 2.0V
```

目标：

输出：

```c
float Ta;
float Tb;
float Tc;
```

范围：

```
0~1
```

对应：

```
0%~100% PWM
```

---

# 二、第一步：判断扇区

六个扇区：

```
        β

        II | I

      III | VI

      IV  | V


        α
```

不要搞角度判断，工程里面一般用：

计算三个量：

```c
float X;
float Y;
float Z;


X = Ubeta;

Y = 0.5f * Ubeta + 0.8660254f * Ualpha;

Z = 0.5f * Ubeta - 0.8660254f * Ualpha;
```

然后：

```c
int sector;


if(Y < 0)
{
    if(Z < 0)
        sector = 5;
    else
        sector = 4;
}
else
{
    if(Z >= 0)
        sector = 1;
    else
    {
        if(X <= 0)
            sector = 6;
        else
            sector = 2;
    }
}
```

这样得到：

```
sector=1~6
```

---

# 三、第二步：计算T1/T2

PWM周期：

假设：

```c
Ts=1.0
```

最后乘PWM周期即可。

定义：

```c
float T1;
float T2;
```

---

## 扇区1

对应：

```
Vref在 V1-V2之间
```

公式：

```
T1 = Ts*(√3*Ubeta/Vdc)

T2 = Ts*(√3/2*Ualpha - √3/2*Ubeta)/Vdc
```

代码：

```c
T1 = SQRT3 * Ubeta / Vdc;

T2 = (SQRT3*0.5f*Ualpha 
      -0.5f*SQRT3*Ubeta)
      /Vdc;
```

---

实际上工程里更常用：

先计算：

```
X
Y
Z
```

然后查表。

六个扇区：

## Sector 1

```
T1 = Z
T2 = Y
```

## Sector 2

```
T1 = Y
T2 = -X
```

## Sector 3

```
T1 = -Z
T2 = X
```

## Sector 4

```
T1 = -X
T2 = -Y
```

## Sector 5

```
T1 = X
T2 = Z
```

## Sector 6

```
T1 = -Y
T2 = -Z
```

所以：

代码：

```c
switch(sector)
{

case 1:
    T1=Z;
    T2=Y;
break;


case 2:
    T1=Y;
    T2=-X;
break;


case 3:
    T1=-Z;
    T2=X;
break;


case 4:
    T1=-X;
    T2=-Y;
break;


case 5:
    T1=X;
    T2=Z;
break;


case 6:
    T1=-Y;
    T2=-Z;
break;

}
```

这个比看公式简单。

---

# 四、第三步计算T0

总时间：

```
Ts=T1+T2+T0
```

所以：

```c
T0 = 1.0f - T1 - T2;
```

然后：

零矢量平均分：

```c
Ta0=T0/2;
```

---

# 五、第四步：七段式核心

重点来了。

七段：

```
V0
V1
V2
V7
V2
V1
V0
```

为什么？

因为：

* 中心对齐PWM
* 上升下降对称
* 开关次数少

---

假设：

sector=1

波形：

```
      T0/2    T1     T2    T0/2

V0 ---- V1 ---- V2 ---- V7
```

对应：

三个桥臂：

```
Ta最大
Tb中间
Tc最小
```

得到：

```c
Ta = T1 + T2 + T0/2;

Tb = T2 + T0/2;

Tc = T0/2;
```

---

六个扇区：

直接写：

```c
switch(sector)
{

case 1:

Ta=T1+T2+T0/2;
Tb=T2+T0/2;
Tc=T0/2;

break;



case 2:

Ta=T1+T0/2;
Tb=T1+T2+T0/2;
Tc=T0/2;

break;



case 3:

Ta=T0/2;
Tb=T1+T2+T0/2;
Tc=T2+T0/2;

break;



case 4:

Ta=T0/2;
Tb=T1+T0/2;
Tc=T1+T2+T0/2;

break;



case 5:

Ta=T2+T0/2;
Tb=T0/2;
Tc=T1+T2+T0/2;

break;



case 6:

Ta=T1+T2+T0/2;
Tb=T0/2;
Tc=T1+T0/2;

break;

}
```

---

# 六、最后输出PWM

你的ESP32 MCPWM：

比如：

```c
CCR_A = Ta * PWM_PERIOD;

CCR_B = Tb * PWM_PERIOD;

CCR_C = Tc * PWM_PERIOD;
```

完成。

---

# 七、完整框架

```c
void SVPWM(float Ualpha,float Ubeta)
{

float X,Y,Z;

float T1,T2,T0;

float Ta,Tb,Tc;

int sector;



X = Ubeta;

Y = 0.5f*Ubeta +0.8660254f*Ualpha;

Z = 0.5f*Ubeta -0.8660254f*Ualpha;



//sector

...



//T1 T2

switch(sector)
{
...
}



//zero vector

T0=1-T1-T2;



//duty

switch(sector)
{
...
}



//update pwm

PWM_A=Ta;
PWM_B=Tb;
PWM_C=Tc;

}
```

---

# 八、你写的时候最容易踩的坑

## 1. Ualpha/Ubeta单位

不要直接：

```
V
```

也可以。

但是：

```
T1/T2不能超过1
```

所以：

需要：

```c
限幅
```

---

## 2. 电压利用率

SVPWM最大：

```
Umax = Vdc/sqrt(3)
```

12V：

```
≈6.92V
```

刚好对应你之前算的。

---

## 3. 电机方向反

不是SVPWM错。

可能：

```
Park角度方向
encoder方向
Uq符号
```

错。

---

你现在的情况（ESP32 + FOC + MCPWM），我建议直接照这个结构写，不要照书里面一大堆几何推导抄。

七段式真正代码量其实就：

* 20行扇区判断
* 20行T1/T2
* 40行Duty映射

总共100行以内。你写出来之后，再拿示波器看三相PWM，会立刻理解书里的七段波形。


----------------
------------
------------

可以。你现在**不要一上来啃文档里的公式**，七段式 SVPWM 真正写代码其实可以压缩成 **4 步**：

> **① 判断扇区 → ② 算 T1/T2 → ③ 算 T0 → ④ 根据扇区把 T1/T2/T0 转成 Ta/Tb/Tc**

你上传的这本书把这部分也拆成了“扇区判断 → 矢量作用时间 → 七段式 → PWM占空比”几个章节，本身就是这个思路。

下面我不按书里的推导讲，**直接从“我要写 C 代码”这个角度讲**。

---

# 一、先把整个 SVPWM 想成这样

FOC 前面已经给你：

$$
U_\alpha,\quad U_\beta
$$

例如：

```text
Uα = 2.0V
Uβ = 1.0V
```

你现在要做的事情就是：

```text
              Uα Uβ
                │
                ▼
        ┌───────────────┐
        │ ① 判断扇区     │
        └───────┬───────┘
                ↓
             Sector 1
                │
                ▼
        ┌───────────────┐
        │ ② 算 T1、T2    │
        └───────┬───────┘
                ↓
        ┌───────────────┐
        │ ③ 算 T0        │
        │ T0=T-T1-T2    │
        └───────┬───────┘
                ↓
        ┌───────────────┐
        │ ④ 算 Ta Tb Tc  │
        └───────┬───────┘
                ↓
             PWM CCR
```

**就这四步。**

---

# 二、为什么要判断扇区？

三相逆变器一共可以产生 8 个开关状态：

```text
000
001
010
011
100
101
110
111
```

其中：

```text
000 → 零矢量 V0
111 → 零矢量 V7
```

剩下 6 个：

```text
100
110
010
011
001
101
```

就是六个基本有效矢量。

空间上：

```text
                  V2
                  ↑
             \    │    /
              \   │   /
           V3  \  │  /  V1
                \ │ /
        V4 ──────●────── V6
                / │ \
           V5  /  │  \
              /   │   \
             /    │    \
                  ↓
```

实际上就是一个六边形。

所以你的目标电压：

$$
\vec U_{\alpha\beta}
$$

一定落在其中一个 60° 扇区里面。

---

# 三、第一步：判断 Sector

这里我建议你**不要一开始使用特别复杂的数学判断**。

最容易理解的是直接算角度：

```c
theta = atan2f(U_beta, U_alpha);
```

然后：

```c
if (theta < 0)
    theta += 2.0f * PI;

sector = (int)(theta / (PI / 3.0f)) + 1;
```

这样：

```text
0°   ~ 60°   → Sector 1
60°  ~ 120°  → Sector 2
120° ~ 180°  → Sector 3
180° ~ 240°  → Sector 4
240° ~ 300°  → Sector 5
300° ~ 360°  → Sector 6
```

**你第一次写 SVPWM，我甚至建议你先这么写。**

虽然 `atan2f()` 比较耗计算量，但是：

> **先把算法跑通，再优化扇区判断。**

等你完全理解以后，再换成不用三角函数的快速判断。

---

# 四、第二步：计算 T1 和 T2

这一步其实是整个 SVPWM 最核心的地方。

假设：

```text
        V2
        ↑
       / \
      /   \
     /  U  \
    /   ↗   \
   /_________\
  V0    V1
```

目标矢量：

$$
U
$$

位于 V1 和 V2 之间。

所以一个 PWM 周期内：

```text
T1 时间 → V1
T2 时间 → V2
T0 时间 → 零矢量
```

要求：

$$
\boxed{T_1+T_2+T_0=T_{PWM}}
$$

---

# 五、其实 T1/T2 可以不用每个扇区重新推公式

这是一个特别重要的技巧。

假设：

$$
U_\alpha,U_\beta
$$

已经知道。

先计算目标矢量：

$$
U=\sqrt{U_\alpha^2+U_\beta^2}
$$

然后知道：

```text
sector
```

再计算这个矢量在扇区内部的角度：

$$
\theta_s=\theta-(sector-1)\frac{\pi}{3}
$$

于是：

$$
0\leq\theta_s<60^\circ
$$

然后：

$$
\boxed{
T_1=
\frac{\sqrt3 T_{PWM}}{U_{dc}}
U\sin(60^\circ-\theta_s)
}
$$

$$
\boxed{
T_2=
\frac{\sqrt3 T_{PWM}}{U_{dc}}
U\sin(\theta_s)
}
$$

然后：

$$
\boxed{
T_0=T_{PWM}-T_1-T_2
}
$$

这就是最直观的版本。

---

# 六、但是这里有一个工程问题

如果你真的这样写：

```c
atan2f()
sinf()
sinf()
```

每次 20kHz FOC 都算，当然也不是不行，但**没必要这么浪费 CPU**。

工程代码通常会利用：

$$
U_\alpha,U_\beta
$$

直接推导出 T1/T2。

但是：

> **你现在先别优化。**

先用数学直观版把波形跑出来。

---

# 七、第三步：T0 是什么？

这个非常简单：

$$
\boxed{
T_0=T_{PWM}-T_1-T_2
}
$$

例如：

```text
PWM = 50us

T1 = 15us
T2 = 20us
```

那么：

$$
T_0=50-15-20=15us
$$

---

# 八、七段式真正开始了

现在：

```text
T1 = 15us
T2 = 20us
T0 = 15us
```

七段式的核心就是：

$$
\boxed{T_0/2,\ T_1,\ T_2,\ T_0/2}
$$

然后利用中心对称：

```text
V0
 ↓
V1
 ↓
V2
 ↓
V7
 ↓
V2
 ↓
V1
 ↓
V0
```

所以完整七段：

```text
T0/2
 │
 V0
 │
T1
 │
 V1
 │
T2
 │
 V2
 │
T0
 │
 V7
 │
T2
 │
 V2
 │
T1
 │
 V1
 │
T0/2
 │
 V0
```

注意：

> **这里的中间 V7 实际上是完整的 T0。**

因为：

$$
T_0/2 + T_0/2=T_0
$$

---

# 九、但是 MCU 不需要你真的去输出这 7 段

这是很多文档把人看懵的地方。

你看到：

```text
V0 → V1 → V2 → V7 → V2 → V1 → V0
```

以为代码是不是要：

```c
输出V0();
delay(T0/2);

输出V1();
delay(T1);

输出V2();
delay(T2);

...
```

**不是。**

MCU 的 PWM 定时器会自动完成这些开关动作。

你真正需要算的是：

$$
\boxed{Ta,\ Tb,\ Tc}
$$

也就是三个 PWM 占空时间。

---

# 十、先只看 Sector 1

假设 Sector 1：

```text
V0 → V1 → V2 → V7 → V2 → V1 → V0
```

对应：

```text
V0 = 000
V1 = 100
V2 = 110
V7 = 111
```

于是：

```text
             A B C

V0          0 0 0
V1          1 0 0
V2          1 1 0
V7          1 1 1
```

看三相：

### A相

```text
0 → 1 → 1 → 1 → 1 → 1 → 0
```

### B相

```text
0 → 0 → 1 → 1 → 1 → 0 → 0
```

### C相

```text
0 → 0 → 0 → 1 → 0 → 0 → 0
```

所以你马上就可以得到：

$$
D_A
$$

$$
D_B
$$

$$
D_C
$$

---

# 十一、Sector 1 的 Ta/Tb/Tc

如果把一个完整 PWM 周期归一化为：

$$
T_{PWM}
$$

那么：

$$
\boxed{
T_a=\frac{T_0}{2}+T_1+T_2
}
$$

$$
\boxed{
T_b=\frac{T_0}{2}+T_2
}
$$

$$
\boxed{
T_c=\frac{T_0}{2}
}
$$

这三个公式你一定要记住。

---

# 十二、举个实际数字

假设：

```text
TPWM = 50us
T1   = 15us
T2   = 20us
```

那么：

$$
T_0=15us
$$

所以：

$$
T_0/2=7.5us
$$

于是：

$$
T_a=7.5+15+20=42.5us
$$

$$
T_b=7.5+20=27.5us
$$

$$
T_c=7.5us
$$

所以：

```text
A = 85%
B = 55%
C = 15%
```

这就是 PWM 要输出的三个占空比。

---

# 十三、这时候你应该突然发现一个规律

Sector 1：

```text
Ta = T0/2 + T1 + T2
Tb = T0/2 + T2
Tc = T0/2
```

那么：

```text
Ta > Tb > Tc
```

这非常符合空间矢量的直觉。

到了 Sector 2：

```text
Ta = T0/2 + T1
Tb = T0/2 + T1 + T2
Tc = T0/2
```

等等。

所以**六个扇区实际上只是 A/B/C 三相占空比排列不同。**

---

# 十四、最简单的七段式代码

如果你现在只是为了**理解 + 跑通**，可以直接这么写：

```c
void svpwm(float u_alpha,
           float u_beta,
           float vbus,
           float *duty_a,
           float *duty_b,
           float *duty_c)
{
    const float PI = 3.1415926f;
    const float SQRT3 = 1.7320508f;

    // PWM周期
    float T = 50e-6f;     // 20kHz

    // ① 求电压矢量幅值
    float U = sqrtf(u_alpha * u_alpha +
                    u_beta  * u_beta);

    // ② 求角度
    float theta = atan2f(u_beta, u_alpha);

    if (theta < 0.0f)
        theta += 2.0f * PI;

    // ③ 判断扇区
    int sector = (int)(theta / (PI / 3.0f)) + 1;

    if (sector > 6)
        sector = 6;

    // ④ 扇区内角度
    float theta_s =
        theta - (sector - 1) * PI / 3.0f;

    // ⑤ 计算 T1 T2
    float T1 =
        SQRT3 * T / vbus *
        U * sinf(PI / 3.0f - theta_s);

    float T2 =
        SQRT3 * T / vbus *
        U * sinf(theta_s);

    // ⑥ 过调制限制
    if (T1 + T2 > T)
    {
        float scale = T / (T1 + T2);

        T1 *= scale;
        T2 *= scale;
    }

    // ⑦ 零矢量时间
    float T0 = T - T1 - T2;

    float T0_2 = T0 * 0.5f;

    // ⑧ 七段式
    float Ta, Tb, Tc;

    switch (sector)
    {
        case 1:
            Ta = T0_2 + T1 + T2;
            Tb = T0_2 + T2;
            Tc = T0_2;
            break;

        case 2:
            Ta = T0_2 + T1;
            Tb = T0_2 + T1 + T2;
            Tc = T0_2;
            break;

        case 3:
            Ta = T0_2;
            Tb = T0_2 + T1 + T2;
            Tc = T0_2 + T2;
            break;

        case 4:
            Ta = T0_2;
            Tb = T0_2 + T1;
            Tc = T0_2 + T1 + T2;
            break;

        case 5:
            Ta = T0_2 + T2;
            Tb = T0_2;
            Tc = T0_2 + T1 + T2;
            break;

        case 6:
            Ta = T0_2 + T1 + T2;
            Tb = T0_2;
            Tc = T0_2 + T1;
            break;

        default:
            Ta = Tb = Tc = T * 0.5f;
            break;
    }

    // ⑨ 转成 Duty
    *duty_a = Ta / T;
    *duty_b = Tb / T;
    *duty_c = Tc / T;
}
```

**这就是完整的七段式 SVPWM。**

没有你想象的那么复杂。

---

# 十五、但是！你贴的那段代码有一个很重要的问题

你前面贴的代码里有：

```c
float K = SQRT3 * PWM_PERIOD / U_dc;

float X = K * V_ref1;
float Y = K * V_ref2;
float Z = K * V_ref3;
```

然后：

```c
case 1:
    T1 = -Z;
    T2 = X;
```

这种写法是**另一种“无三角函数快速计算 T1/T2”实现**。

它本身没问题，但是：

> **非常容易因为 αβ 坐标定义、扇区编号、Vref1/Vref2/Vref3 的符号定义不同而把自己绕进去。**

所以我非常建议你现在：

### 第一版

先写：

```text
atan2
 ↓
sector
 ↓
theta_sector
 ↓
sin
 ↓
T1/T2
 ↓
T0
 ↓
switch
 ↓
Ta/Tb/Tc
```

把示波器波形跑通。

### 第二版

再优化成：

```text
Uα/Uβ
 ↓
不用 atan2
 ↓
直接判断 sector
 ↓
不用 sin
 ↓
直接算 T1/T2
```

这样你会非常容易理解。

---

# 十六、再给你一个特别重要的认识

你现在可能一直在想：

> “七段式是不是要写七段 PWM？”

其实不是。

**七段式是数学意义上的开关状态序列。**

你的代码最终只需要：

```c
Ta
Tb
Tc
```

然后：

```text
Ta ─────→ PWM A
Tb ─────→ PWM B
Tc ─────→ PWM C
```

中心对齐定时器自己就会产生：

```text
V0 → V1 → V2 → V7 → V2 → V1 → V0
```

这样的对称开关序列。

所以：

$$
\boxed{
七段式SVPWM代码的核心不是“写七段”
}
$$

而是：

$$
\boxed{
T_1,T_2,T_0
\rightarrow
T_a,T_b,T_c
}
$$

---

# 十七、你现在做 ESP32，建议你按这个顺序搞

你现在不要直接把复杂优化代码塞进 FOC。

按照：

```text
第1阶段
Uα/Uβ
 ↓
atan2f
 ↓
Sector
 ↓
T1/T2
 ↓
T0
 ↓
Ta/Tb/Tc
```

先验证。

然后示波器看：

```text
A/B/C PWM
```

应该是中心对称的。

再第二阶段：

```text
去掉 atan2f
```

第三阶段：

```text
去掉 sinf
```

第四阶段：

```text
加入过调制
```

第五阶段：

```text
加入死区补偿
```

第六阶段：

```text
ADC 与 PWM 同步采样
```

这才是比较靠谱的工程路线。

---

### 最关键的四个公式，你现在先记这几个

$$
\boxed{T_0=T_{PWM}-T_1-T_2}
$$

Sector 1：

$$
\boxed{T_a=T_0/2+T_1+T_2}
$$

$$
\boxed{T_b=T_0/2+T_2}
$$

$$
\boxed{T_c=T_0/2}
$$

然后其他 5 个扇区，本质上就是**把这三个占空比按照空间矢量顺序轮换**。

**如果你愿意，我下一步可以直接拿你现在的 `U_alpha/U_beta/VBUS/PWM=20kHz`，从一个具体数值开始，手算一遍“Sector 1 → T1/T2 → T0 → Ta/Tb/Tc → ESP32 MCPWM比较值”，你看完基本就彻底明白七段式为什么这样算了。**

