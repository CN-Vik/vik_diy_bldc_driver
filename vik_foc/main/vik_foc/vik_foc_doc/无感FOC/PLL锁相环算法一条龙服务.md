FOC里的 **PLL锁相环（Phase Locked Loop）**，核心作用一句话：

> **根据反电动势（或编码器信号）的相位误差，自动调整估计角速度，使内部产生的电角度跟踪真实电角度。**

在无感FOC里，PLL通常接在 **SMO滑模观测器后面**。

---

## 1. 输入输出

输入：

```
SMO输出：
Eα
Eβ
```

这是估计的反电动势矢量：

$$
\vec E=
\begin{bmatrix}
E_\alpha\\
E_\beta
\end{bmatrix}
$$

输出：

```
θe  电角度
ωe  电角速度
```

给FOC Park变换：

$$
Id/Iq = Park(I_\alpha,I_\beta,\theta_e)
$$

---

# 2. PLL核心思想

假设真实反电势：

$$
E_\alpha=E_m cos(\theta_e)
$$

$$
E_\beta=E_m sin(\theta_e)
$$

PLL内部有一个估计角：

$$
\hat{\theta}
$$

如果：

$$
\hat{\theta}=\theta_e
$$

说明锁定。

---

# 3. 如何计算角度误差？

关键一步：

把反电势投影到PLL坐标系：

### d轴：

$$
E_d=
E_\alpha cos(\hat{\theta})
+
E_\beta sin(\hat{\theta})
$$

### q轴：

$$
E_q=
-E_\alpha sin(\hat{\theta})
+
E_\beta cos(\hat{\theta})
$$

其中：

* d轴 = 同方向
* q轴 = 垂直方向

锁定时：

$$
E_q=0
$$

因为：

真实角度和估计角度没有偏差。

---

如果：

估计角度慢了：

```
真实角度
    /
   /
  /
估计角度
```

会产生：

$$
E_q>0
$$

如果：

估计角度快了：

$$
E_q<0
$$

所以：

$$
E_q
$$

就是：

> **角度误差信号**

---

# 4. PI调节速度

PLL内部就是一个PI：

$$
\omega_e=
K_pE_q+
K_i\int E_q dt
$$

得到电角速度：

$$
\omega_e
$$

然后积分：

$$
\theta_e(k)
=
\theta_e(k-1)
+
\omega_e T_s
$$

得到新的角度。

---

# 5. 完整流程

```
        电机
         |
         |
      反电动势
         |
         v
       SMO
         |
     Eα Eβ
         |
         v
   +-------------+
   |     PLL     |
   |             |
   | Park变换    |
   |             |
   | 计算Eq      |
   |             |
   | PI调节      |
   |             |
   | 积分得到θ   |
   +-------------+
         |
         |
       θe
         |
         v
       FOC
```

---

# 6. 最核心公式（记住这几个）

### 反电势：

$$
E_\alpha=E_m cos\theta
$$

$$
E_\beta=E_m sin\theta
$$

### 误差：

$$
E_q=
-E_\alpha sin\hat{\theta}
+
E_\beta cos\hat{\theta}
$$

### PI：

$$
\omega=
K_pE_q+
K_i\int E_q
$$

### 积分：

$$
\theta=
\theta+\omega T_s
$$

---

# 7. 参数怎么调？

主要两个：

### Kp

决定：

* 响应速度
* 抗扰能力

大：

* 跟踪快
* 容易振荡

小：

* 平稳
* 跟踪慢

---

### Ki

决定：

* 消除稳态误差

大：

* 锁相快
* 容易低频摆动

---

实际FOC：

一般：

```
先调Kp
让角度能跟随

再增加Ki
消除残余误差
```

---

## 一句话总结PLL：

> **PLL就是一个“自动追角器”：把SMO输出的反电动势矢量和自己的估计角度比较，产生角度误差Eq，通过PI得到速度，再积分得到电角度。**

SMO负责：

> “我看到的反电动势在哪里”

PLL负责：

> “根据这个方向平滑算出电机现在转到多少度了”

所以无感FOC经典组合：

```
SMO → 提供方向
PLL → 提供稳定角度和速度
FOC → 控制电流和转矩
```

这也是工业无感FOC最常见架构。


-----------
-----------
下面给你一套**标准无感FOC用 SMO + PLL 锁相环算法**，输入滑模观测器输出的 **Eα/Eβ反电动势**，输出：

* 电角度 `theta_e`
* 电角速度 `omega_e`

可以直接嵌入你的 ESP32 FOC 工程。

---

## PLL结构体

```c
typedef struct
{
    // PLL参数
    float Kp;
    float Ki;

    // 输入反电动势
    float Ealpha;
    float Ebeta;

    // 输出
    float theta;      // 电角度 rad
    float omega;      // 电角速度 rad/s

    // 内部积分
    float integral;

    // 时间
    float Ts;

} pll_t;
```

---

# 初始化

```c
void PLL_Init(pll_t *pll, float Ts)
{
    pll->Kp = 200.0f;
    pll->Ki = 5000.0f;

    pll->theta = 0.0f;
    pll->omega = 0.0f;

    pll->integral = 0.0f;

    pll->Ts = Ts;
}
```

---

# PLL核心算法

```c
void PLL_Update(pll_t *pll, float Ealpha, float Ebeta)
{

    float sin_theta;
    float cos_theta;


    /*
     * 当前PLL估计角度
     */
    sin_theta = sinf(pll->theta);
    cos_theta = cosf(pll->theta);



    /*
     * 反Park变换
     *
     * 把反电势转换到PLL坐标系
     *
     */

    float Ed;
    float Eq;


    Ed =  Ealpha * cos_theta
        + Ebeta  * sin_theta;


    Eq = -Ealpha * sin_theta
        + Ebeta  * cos_theta;



    /*
     * 归一化
     *
     * 防止低速反电势太小导致PLL发散
     */

    float Emag;

    Emag = sqrtf(Ealpha*Ealpha +
                 Ebeta*Ebeta);


    if(Emag > 0.001f)
    {
        Eq = Eq / Emag;
    }
    else
    {
        Eq = 0;
    }



    /*
     * PI控制器
     *
     * Eq就是角度误差
     */

    pll->integral += 
            pll->Ki *
            Eq *
            pll->Ts;


    pll->omega =
            pll->Kp * Eq
            +
            pll->integral;



    /*
     * 角度积分
     */

    pll->theta +=
            pll->omega *
            pll->Ts;



    /*
     * 角度归一化
     */

    while(pll->theta > 2*PI)
    {
        pll->theta -= 2*PI;
    }


    while(pll->theta < 0)
    {
        pll->theta += 2*PI;
    }

}
```

---

# 在你的FOC里的调用位置

你的流程应该是：

```
ADC采样
 |
 v
Clarke
 |
 v
电流
 |
 v
SMO
 |
 +------+
 |      |
Eα,Eβ  |
        |
        v
       PLL
        |
        |
      theta_e
        |
        v
      Park变换
        |
        v
       Id/Iq
        |
        v
       PI
        |
        v
      SVPWM
```

代码：

```c
SMO_Update();

PLL_Update(
    &pll,
    smo.Ealpha,
    smo.Ebeta
);


theta_e = pll.theta;
```

然后：

```c
Park(
    Ialpha,
    Ibeta,
    theta_e
);
```

---

## 参数初始建议（你的20kHz FOC）

你的：

```
Ts = 1/20000
   = 50us
```

可以先：

```c
Kp = 200
Ki = 5000
```

如果：

### 角度抖动大

降低：

```
Kp ↓
Ki ↓
```

### 跟不上高速

增加：

```
Kp ↑
Ki ↑
```

---

## 注意一个关键点（你之前SMO遇到的问题）

SMO输出：

$$
E_\alpha,E_\beta
$$

不是角度！

它只是：

```
反电动势矢量
```

例如：

```
Eα = 3
Eβ = 4
```

方向：

$$
atan2(4,3)
$$

才是电角度。

PLL干的事情就是：

> 不直接 atan2，而是利用PI动态追踪这个方向，让角度连续、抗噪。

---

针对你现在的 **ESP32 + 2208-80T + 20kHz FOC + SMO**，这套PLL结构基本就是 TI、ST、VESC 类无感FOC里面常用的形式。你后面只需要重点调 **SMO增益 + PLL Kp/Ki**。


---------
----------
PLL调参的核心目标：

> **让PLL里的估计角度 θe 平滑、快速跟踪 SMO 输出的反电动势方向，同时不过度振荡。**

PLL主要就两个参数：

* **Kp：比例增益 → 决定跟踪速度**
* **Ki：积分增益 → 消除稳态角度误差**

---

## 1. 先理解PLL动态关系

PLL本质：

$$
E_q \rightarrow PI \rightarrow \omega_e \rightarrow 积分 \rightarrow \theta_e
$$

其中：

$$
E_q
$$

就是角度误差。

比如：

真实角度：

```
       /
      /
     /
```

PLL估计：

```
    /
   /
  /
```

有误差：

$$
\Delta\theta
$$

产生：

$$
E_q
$$

然后PLL加速追赶。

---

# 2. 调参顺序（重点）

不要同时调 Kp Ki。

正确顺序：

```
先调Kp
↓
再调Ki
```

---

# 第一步：关闭积分 Ki

先：

```c
Ki = 0;
```

只调：

```c
Kp
```

---

## Kp太小表现

现象：

* 电机启动后角度跟不上
* SMO角度和PLL角度差很大
* 高速时失步
* 电流波形畸变

例如：

```
真实角度:

/\/\/\/\/


PLL:

/\/_/\/_/_
```

说明：

PLL反应太慢。

处理：

增加：

```
Kp ↑
```

---

## Kp太大表现

现象：

* theta高速抖动
* 电流声音尖锐
* iq波动
* 电机震动

波形：

```
真实:

------

PLL:

~~~~~~
```

说明：

PLL响应过快，追着噪声跑。

处理：

降低：

```
Kp ↓
```

---

# 第二步：加入Ki

Kp调到：

> 能跟踪，但有一点静态误差

然后增加：

```c
Ki
```

---

## Ki太小

表现：

* 低速角度偏差大
* iq不稳定
* 有固定相位差

例如：

真实：

```
    /
   /
  /
```

PLL:

```
 /
/
```

一直差一个角度。

增加：

```
Ki ↑
```

---

## Ki太大

表现：

* 低频摆动
* 转速忽快忽慢
* 启动容易震荡

处理：

降低：

```
Ki ↓
```

---

# 3. 实际工程调参方法

你的参数：

```
FOC频率:
20kHz

Ts:
50us
```

建议流程：

## 阶段1：低速运行

比如：

1000RPM

设置：

```
Kp=50
Ki=0
```

观察：

theta是否稳定。

慢慢增加：

```
Kp:

50
100
150
200
300
```

直到：

* 不明显滞后
* 不明显震荡

---

## 阶段2：加积分

比如：

Kp=200

增加：

```
Ki:

1000
2000
5000
10000
```

直到：

* 相位误差消失
* iq稳定

---

# 4. 推荐范围（20kHz FOC）

你的：

ESP32
20kHz
BLDC小电机

可以从：

```c
Kp = 100~300

Ki = 1000~10000
```

开始。

比较常见：

```c
Kp = 200

Ki = 5000
```

---

# 5. 判断PLL是否调好的方法

看三个东西：

---

## ① 看Eq

正常：

```
Eq ≈ 0
```

如果：

```
Eq持续偏大
```

说明：

角度有误差。

---

## ② 看theta波形

正常：

```
0
 \
  \
   \
    \
     2π
```

连续增加。

异常：

```
0
\/\/\/\/
```

PLL震荡。

---

## ③ 看Iq

PLL好：

```
Iq:

------
```

PLL差：

```
~~~~~~~
```

因为角度错：

Park变换错。

---

# 6. 一个非常重要的经验

PLL不要调太快。

很多人喜欢：

```
Kp很大
Ki很大
```

结果：

SMO噪声直接进入角度。

工业做法：

> SMO负责提供趋势，PLL负责滤波和平滑。

所以：

宁愿PLL慢一点，也不要追噪声。

---

针对你现在的 **2208-80T（7对极）+ SMO + ESP32 + 20kHz FOC**：

我建议第一版：

```c
PLL:
Kp = 150
Ki = 3000
Ts = 50e-6
```

然后看：

1. 低速1000RPM能否锁定
2. 高速3000RPM是否失步
3. Eq是否接近0

另外你之前说 **SMO单独输出角度电机会嗡嗡响**，大概率不是PLL参数问题，而是 **SMO角度相位偏移 + PLL锁相前角度不连续**，加入PLL后通常会明显改善。



------------
-----------
在FOC（磁场定向控制）中，**PLL（锁相环）**主要用于**无感控制**（如配合滑模观测器SMO）或**旋转变压器解码**。它的核心作用是从含有高频噪声的观测信号（如估算出的反电动势 $E_\alpha, E_\beta$）中，提取出平滑的**转子电角度 $\hat{\theta}$** 和 **电角速度 $\hat{\omega}$**。

PLL的核心思想是通过**闭环控制**，让“估算角度”不断跟随“实际角度”，使两者的相位误差趋近于零。

算法由以下三个核心模块组成：

### 1. 鉴相器 (Phase Detector) —— 计算角度误差

鉴相器的作用是提取实际角度 $\theta$ 与估算角度 $\hat{\theta}$ 之间的偏差。在无感FOC中，通常利用估算的反电动势分量与估算角度进行叉乘计算：


$$e = -E_\alpha \cos\hat{\theta} - E_\beta \sin\hat{\theta}$$


根据三角函数展开，当误差很小时，该结果近似正比于角度误差 $\Delta\theta$：


$$e \approx E_m \sin(\theta - \hat{\theta}) \approx E_m (\theta - \hat{\theta})$$

### 2. 环路滤波器 (Loop Filter) —— 输出转速

环路滤波器通常是一个 **PI控制器**。它将鉴相器输出的误差 $e$ 作为输入进行调节，输出的结果就是估算的电角速度 $\hat{\omega}$。这里的PI控制器本质上起到了**低通滤波器**的作用，能够滤除反电动势信号中的高频滑动颤振噪声。


$$\hat{\omega} = K_p \cdot e + K_i \int e \, dt$$

### 3. 积分器 (VCO/Integrator) —— 输出角度

将PI输出的估算角速度 $\hat{\omega}$ 乘以控制周期 $\Delta t$ 进行离散积分，即可得到当前的估算电角度 $\hat{\theta}$。这个角度会反馈给鉴相器，形成闭环。


$$\hat{\theta} = \hat{\theta}_{prev} + \hat{\omega} \cdot \Delta t$$

---

### 💡 为什么用 PLL 而不是直接求反正切 (atan2)？

如果直接通过估算的反电动势求反正切计算角度（$\theta = \arctan(E_\beta / E_\alpha)$），信号中的高频谐波和计算噪声会直接反映在角度和速度上，导致电机运行不平稳。**PLL 将纯数学的几何求解变成了闭环调节系统**，利用积分的滞后特性完美过滤了高频噪声，提取出的 $\hat{\omega}$ 和 $\hat{\theta}$ 波形（例如你在 FreeMASTER 中观察到的波形）会非常平滑。

--------
-------
这是一个适合在单片机（如 STM32 或 ESP32）中运行的 C 语言实现。代码采用了浮点运算，包含了你所需的**鉴相器**、**PI 环路滤波器**和**积分器**三个核心部分。

```c
#include <math.h>

#define PI_F 3.14159265358979f

// PLL 结构体定义
typedef struct {
    float kp;           // PI 比例增益
    float ki;           // PI 积分增益
    float dt;           // 控制周期 (例如 100us -> 0.0001f)
    
    float speed_integ;  // 速度积分项缓存
    
    float speed_est;    // 输出：估算电角速度 (rad/s)
    float angle_est;    // 输出：估算电角度 (rad)
} FOC_PLL_t;

// PLL 初始化函数
void FOC_PLL_Init(FOC_PLL_t *pll, float kp, float ki, float dt) {
    pll->kp = kp;
    pll->ki = ki;
    pll->dt = dt;
    pll->speed_integ = 0.0f;
    pll->speed_est = 0.0f;
    pll->angle_est = 0.0f;
}

// PLL 更新函数 (在 FOC 中断/任务中高频调用)
// 输入: e_alpha, e_beta (滑模观测器等输出的反电动势)
void FOC_PLL_Update(FOC_PLL_t *pll, float e_alpha, float e_beta) {
    // 1. 鉴相器 (Phase Detector)：计算相位误差
    // 注：若为了极致性能，此处的 sin/cos 可替换为查表法或 CMSIS-DSP 的 arm_sin_cos_f32
    float sin_theta = sinf(pll->angle_est);
    float cos_theta = cosf(pll->angle_est);
    
    // 误差计算：e = -E_alpha * cos(theta) - E_beta * sin(theta)
    float err = -e_alpha * cos_theta - e_beta * sin_theta;
    
    // 2. 环路滤波器 (Loop Filter)：PI 控制器，输出转速
    pll->speed_integ += pll->ki * err * pll->dt;
    
    // (可选) 在此添加反积分饱和(Anti-windup)限幅，防止 speed_integ 跑飞
    // pll->speed_integ = clamp(pll->speed_integ, -MAX_SPEED, MAX_SPEED);
    
    pll->speed_est = pll->kp * err + pll->speed_integ;
    
    // 3. 积分器 (VCO/Integrator)：计算电角度
    pll->angle_est += pll->speed_est * pll->dt;
    
    // 4. 角度归一化 (限制在 -PI 到 PI 之间)
    if (pll->angle_est > PI_F) {
        pll->angle_est -= 2.0f * PI_F;
    } else if (pll->angle_est < -PI_F) {
        pll->angle_est += 2.0f * PI_F;
    }
}

```

### 💡 工程应用提示

1. **误差归一化（防止转速影响）**
因为反电动势的幅值 $E_m$ 会随着电机转速成比例增加，导致鉴相器的误差增益在不同转速下发生变化。为了让同一套 $K_p, K_i$ 参数适应全速段，可以对 `err` 进行归一化（除以反电动势幅值，或者使用符号函数 `sign()` 处理），剥离速度带来的增益变化。
2. **运算优化**
代码中的 `sinf` 和 `cosf` 在中断中执行开销较大。通常使用查表法配合线性插值，或者调用带有 DSP 指令集优化的数学库，以确保控制循环能在几十微秒内执行完毕。

--------
---------
# PLL调参教程

FOC无感控制中的PLL本质上是一个**二阶线性跟踪系统**。调参核心思想是将闭环传递函数映射为标准的二阶系统模型，通过设定目标带宽（响应速度）**和**阻尼比（超调量）直接计算出 $K_p$ 与 $K_i$。

---

### 第一步：数学建模与参数求解

在鉴相器计算中，若先对反电动势矢量进行**归一化处理**（即除以幅值 $E_m = \sqrt{E_\alpha^2 + E_\beta^2}$），则误差增益定为 $1$。

此时 PLL 闭环传递函数为：


$$G(s) = \frac{\hat{\theta}(s)}{\theta(s)} = \frac{K_p s + K_i}{s^2 + K_p s + K_i}$$

对照标准二阶系统特征方程 $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$，可以直接推导出参数公式：

* **$K_p = 2 \cdot \zeta \cdot \omega_n$**
* **$K_i = \omega_n^2$**

*其中：*

* $\zeta$ **（阻尼比）：** 通常固定取 **$0.707$**（等同于 $\frac{\sqrt{2}}{2}$），以获得极佳的动态响应且超调量最小。
* $\omega_n$ **（自然角频率/带宽）：** 单位为 rad/s，$\omega_n = 2\pi f_n$。$f_n$ 即为 PLL 的设计带宽。

---

### 第二步：确定设计带宽 $f_n$

PLL 带宽 $f_n$ 是**唯一需要根据实际工况选择的变量**：

1. **上限限制：** $f_n$ 必须远小于电流环带宽（通常取电流环带宽的 **$1/10 \sim 1/20$**）或采样频率的 $1/100$。
* *例：* 电流环周期 $100\ \mu\text{s}$（$10\text{ kHz}$），电流环带宽约 $500\text{ Hz}$，则 PLL 带宽 $f_n$ 建议设为 **$20 \sim 50\text{ Hz}$**。


2. **下限限制：** $f_n$ 必须高于电机最大加速度对应的转角变化频率，否则快速加减速时会产生相角滞后甚至失步。

---

### 第三步：工程计算示例（以 $f_n = 30\text{ Hz}$ 为例）

1. **计算角频率：** $\omega_n = 2\pi \times 30 \approx 188.5\text{ rad/s}$
2. **计算连续域 $K_p, K_i$：**
* $K_p = 2 \times 0.707 \times 188.5 \approx \mathbf{266.5}$
* $K_i = 188.5^2 \approx \mathbf{35530}$


3. **离散化（转换为代码参数）：**
设控制周期为 $T_s$（如 $100\ \mu\text{s}$）：
* 代码 $K_{p, discrete} = K_p = \mathbf{266.5}$
* 代码 $K_{i, discrete} = K_i \times T_s = 35530 \times 0.0001 = \mathbf{3.553}$



---

### 现场实验微调与现象排查

在 FreeMASTER 或示波器观察转速波形，按下表特征进行微调：

| 现象 / 症状 | 原因分析 | 调整策略 |
| --- | --- | --- |
| **突加负载或快速加减速时，估计角度严重滞后，电机抖动** | PLL 响应太慢（带宽 $f_n$ 过低） | 按比例同步增大 $K_p$ 和 $K_i$（即提高 $f_n$） |
| **平稳运行时，估算角速度 $\hat{\omega}$ 存在大量高频毛刺** | PLL 滤波能力不足（带宽 $f_n$ 过高） | 按比例同步减小 $K_p$ 和 $K_i$，或在 $\hat{\omega}$ 后加一阶低通滤波器 |
| **转速阶跃响应时超调大、震荡次数多** | 阻尼不足 | 单独适当加大 $K_p$ 或减小 $K_i$ |
| **高低速切换时，PLL 表现差异巨大** | 未对反电动势归一化 | 强制加入 $E_\alpha, E_\beta$ 幅值归一化算法 |

---

### 关键工程技巧

1. **必须做幅值归一化：** 反电动势幅值 $E_m$ 随转速成正比变化。若不进行归一化（即除以 $\sqrt{E_\alpha^2 + E_\beta^2}$），系统的实际增益会随转速剧烈变化，导致低速调好的参数在高速下自激振荡。
2. **速度一阶滤波（LPF）：** PI 滤波后的 $\hat{\omega}$ 往往仍带有一些噪声，建议在其输出端追加一个截止频率为 $2 \sim 3 \times f_n$ 的低通滤波器，再喂给 speed loop（速度环）。
3. **积分限幅（Anti-Windup）：** 必须对环路滤波器的积分项以及最终输出的 $\hat{\omega}$ 进行饱和限幅，限幅值设为电机允许的最大电角速度。


-----------
------------

标准工业级 FOC 无感控制中 PLL 离散化 C 语言代码包含**反电动势幅值归一化**、**防除零保护**、抗积分饱和（Anti-Windup）**以及**速度一阶低通滤波（LPF）功能。

```c
#ifndef FOC_PLL_H
#define FOC_PLL_H

#include <math.h>

#define TWO_PI 6.283185307179586f

typedef struct {
    // --- 参数配置 (Configuration) ---
    float Kp;            // PLL 比例增益
    float Ki_dt;         // PLL 离散积分系数 (Ki * Ts)
    float Ts;            // 控制周期 (秒)
    float max_speed;     // 最大允许电角速度限幅 (rad/s)
    float lpf_alpha;     // 速度低通滤波系数
    float min_e_mag;     // 幅值归一化最小下限 (防除零)

    // --- 运行状态变量 (State Variables) ---
    float integr;        // PI 积分器累加值
    float speed_raw;     // PLL 估算原始电角速度 (rad/s)
    float speed_lpf;     // 低通滤波后的电角速度 (用于速度环反馈)
    float theta;         // 估算电角度 (rad, 范围 [0, 2π))
} PLL_TypeDef;

/**
 * @brief  PLL 初始化函数
 * @param  pll: PLL 结构体指针
 * @param  fn: 设计的目标带宽 (Hz)，推荐 20~50Hz
 * @param  zeta: 阻尼比，推荐 0.707f
 * @param  Ts: 执行周期 (s)，例如 0.0001f (对应 10kHz)
 * @param  max_elec_rpm: 允许的最大电转速 (RPM)
 * @param  lpf_cutoff_hz: 速度滤波截止频率 (Hz)，推荐 2~3 倍 fn
 */
void PLL_Init(PLL_TypeDef *pll, float fn, float zeta, float Ts, float max_elec_rpm, float lpf_cutoff_hz) {
    float omega_n = TWO_PI * fn;
    
    pll->Ts = Ts;
    pll->Kp = 2.0f * zeta * omega_n;
    pll->Ki_dt = (omega_n * omega_n) * Ts;
    
    // 将电 RPM 转换为电角速度 (rad/s)
    pll->max_speed = max_elec_rpm * (TWO_PI / 60.0f); 
    pll->min_e_mag = 0.001f; // 归一化分母下限保护，视反电动势幅值标幺值而定

    // 一阶低通滤波系数 alpha = Ts / (Ts + RC)
    float rc = 1.0f / (TWO_PI * lpf_cutoff_hz);
    pll->lpf_alpha = Ts / (Ts + rc);

    // 状态复位
    pll->integr = 0.0f;
    pll->speed_raw = 0.0f;
    pll->speed_lpf = 0.0f;
    pll->theta = 0.0f;
}

/**
 * @brief  PLL 周期计算更新函数 (放在中断中运行)
 * @param  pll: PLL 结构体指针
 * @param  E_alpha: 观测器输出的 alpha 轴反电动势
 * @param  E_beta:  观测器输出的 beta 轴反电动势
 */
void PLL_Update(PLL_TypeDef *pll, float E_alpha, float E_beta) {
    // 1. 计算反电动势幅值
    float e_mag = sqrtf(E_alpha * E_alpha + E_beta * E_beta);
    float err = 0.0f;

    // 2. 鉴相器计算 + 幅值归一化 (加上防除零保护)
    if (e_mag > pll->min_e_mag) {
        // e = (-E_alpha * cos(theta) - E_beta * sin(theta)) / E_mag
        err = (-E_alpha * cosf(pll->theta) - E_beta * sinf(pll->theta)) / e_mag;
    } else {
        // 静止或低速反电动势未建立时，不进行角度纠偏
        err = 0.0f;
    }

    // 3. 环路滤波器 (PI 控制器)
    // 积分项累加
    pll->integr += pll->Ki_dt * err;

    // 积分饱和限幅 (Anti-Windup)
    if (pll->integr > pll->max_speed) {
        pll->integr = pll->max_speed;
    } else if (pll->integr < -pll->max_speed) {
        pll->integr = -pll->max_speed;
    }

    // PI 最终输出原始角速度
    pll->speed_raw = pll->Kp * err + pll->integr;

    // 原始角速度总限幅
    if (pll->speed_raw > pll->max_speed) {
        pll->speed_raw = pll->max_speed;
    } else if (pll->speed_raw < -pll->max_speed) {
        pll->speed_raw = -pll->max_speed;
    }

    // 4. 速度一阶低通滤波 (用于闭环反馈)
    pll->speed_lpf += pll->lpf_alpha * (pll->speed_raw - pll->speed_lpf);

    // 5. 积分器 (VCO) —— 更新电角度
    pll->theta += pll->speed_raw * pll->Ts;

    // 6. 角度归一化到 [0, 2π) 循环区间
    if (pll->theta >= TWO_PI) {
        pll->theta -= TWO_PI;
    } else if (pll->theta < 0.0f) {
        pll->theta += TWO_PI;
    }
}

#endif // FOC_PLL_H

```

---

**代码关键点设计说明：**

1. **防除零保护（`min_e_mag`）：** 当电机处于零速或极低速时，观测出的反电动势接近 $0$。若强行除以 $E_m$ 会引发浮点数溢出或噪声自激，此时将误差强制置零，锁定估算。
2. **离散积分处理（`Ki_dt`）：** 在 `PLL_Init` 中提前将 $K_i \times T_s$ 算好，可以在中断执行中节省一次浮点乘法运算。
3. **角度卷绕（`theta` 归一化）：** 控制角度始终在 $[0, 2\pi)$ 之间，方便直接传给后续的 `arm_cos_f32` / `arm_sin_f32` 等三角函数查表模块。
4. **两路速度输出分工：**
* `speed_raw` 用于内部更新角度积分，保留相位超前特性；
* `speed_lpf` 滤除了高频颤振，专门提供给 Outer Loop（速度环）进行 PID 反馈。

---------
-------

将滑模观测器（SMO）与 PLL 无缝结合的核心优势在于：**不再需要传统的低通滤波器（LPF）来滤除反电动势高频抖振，从而彻底避免了 LPF 带来的相位滞后补偿难题**。PLL 的环路滤波器本身就担当了最佳的自适应滤波角色。

---

### 系统数据流图

$$\begin{matrix} v_{\alpha,\beta} \\ i_{\alpha,\beta} \end{matrix}  \xrightarrow{\quad}  \begin{bmatrix} \text{滑模电流观测器} \\ \text{(SMO)} \end{bmatrix}  \xrightarrow{\quad z_{\alpha,\beta}\ (E_{\alpha,\beta}) \quad}  \begin{bmatrix} \text{幅值归一化} \\ + \\ \text{锁相环 (PLL)} \end{bmatrix}  \xrightarrow{\quad}  \begin{matrix} \hat{\theta} \ (\text{用于 Park 变换}) \\ \hat{\omega} \ (\text{用于速度闭环}) \end{matrix}$$

---

### SMO + PLL 完整 C 语言实现

```c
#ifndef FOC_SMO_PLL_H
#define FOC_SMO_PLL_H

#include <math.h>
#include "foc_pll.h"  // 引入上一节定义的 PLL_TypeDef 及函数

// --- 滑模观测器结构体定义 ---
typedef struct {
    // 物理参数 (Physical Parameters)
    float Rs;           // 定子相电阻 (Ω)
    float Ls;           // 定子相电感 (H)
    float Ts;           // 控制周期 (s)

    // 调参变量 (Tuning Parameters)
    float Ksmo;         // 滑模控制增益
    float Epsilon;      // 饱和函数边界层厚度 (用于抑制高频抖振)

    // 运行状态变量 (States)
    float i_alpha_hat;  // 估算的 alpha 轴电流 (A)
    float i_beta_hat;   // 估算的 beta 轴电流 (A)
    float z_alpha;      // 滑模输出，即提取出的 E_alpha (V)
    float z_beta;       // 滑模输出，即提取出的 E_beta (V)
} SMO_TypeDef;

/**
 * @brief 连续连续函数的连续近似 —— 饱和函数 sat(x)
 * @note  替代传统 sign() 符号函数，大幅削弱滑动模态下的高频抖振噪声
 */
static inline float sat(float x, float eps) {
    if (x > eps)  return 1.0f;
    if (x < -eps) return -1.0f;
    return x / eps;
}

/**
 * @brief SMO 初始化
 */
void SMO_Init(SMO_TypeDef *smo, float Rs, float Ls, float Ts, float Ksmo, float Epsilon) {
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->Ts = Ts;
    smo->Ksmo = Ksmo;
    smo->Epsilon = Epsilon;

    smo->i_alpha_hat = 0.0f;
    smo->i_beta_hat = 0.0f;
    smo->z_alpha = 0.0f;
    smo->z_beta = 0.0f;
}

/**
 * @brief  SMO 核心更新算法 (单步迭代)
 * @param  v_alpha, v_beta: α-β 轴端电压 (V)
 * @param  i_alpha, i_beta: α-β 轴实测电流 (A)
 */
void SMO_Update(SMO_TypeDef *smo, float v_alpha, float v_beta, float i_alpha, float i_beta) {
    // 1. 计算电流估算偏差: e_i = i_hat - i_actual
    float err_i_alpha = smo->i_alpha_hat - i_alpha;
    float err_i_beta  = smo->i_beta_hat - i_beta;

    // 2. 滑模控制律生成 (利用饱和函数提取反电动势等效信号 z)
    smo->z_alpha = smo->Ksmo * sat(err_i_alpha, smo->Epsilon);
    smo->z_beta  = smo->Ksmo * sat(err_i_beta, smo->Epsilon);

    // 3. 欧拉法离散化电流状态方程更新 i_hat
    // di_hat/dt = (-Rs/Ls) * i_hat + (1/Ls) * v - (1/Ls) * z
    float inv_Ls = 1.0f / smo->Ls;
    float di_alpha = (-smo->Rs * inv_Ls * smo->i_alpha_hat) + (inv_Ls * v_alpha) - (inv_Ls * smo->z_alpha);
    float di_beta  = (-smo->Rs * inv_Ls * smo->i_beta_hat)  + (inv_Ls * v_beta)  - (inv_Ls * smo->z_beta);

    smo->i_alpha_hat += di_alpha * smo->Ts;
    smo->i_beta_hat  += di_beta * smo->Ts;
}

#endif // FOC_SMO_PLL_H

```

---

### 实际中断服务函数 (ISR) 对接例程

在 PWM 中断服务函数中按顺序调用：

```c
// 实例化全局控制变量
SMO_TypeDef g_smo;
PLL_TypeDef g_pll;

void System_Init(void) {
    // 1. 初始化 SMO (例：相电阻 0.5Ω，相电感 1.5mH，周期 100us)
    // Ksmo 初值建议设为电机额定反电动势峰值的 1.2~2.0 倍
    SMO_Init(&g_smo, 0.5f, 0.0015f, 0.0001f, 20.0f, 0.5f);

    // 2. 初始化 PLL (带宽 30Hz, 阻尼比 0.707, 周期 100us, 最高电转速 6000RPM, 滤波 60Hz)
    PLL_Init(&g_pll, 30.0f, 0.707f, 0.0001f, 6000.0f, 60.0f);
}

// 假设为 10kHz 的 FOC 核心中断服务程序
void PWM_Interrupt_Handler(void) {
    // A. 读取相电流采样并进行 Clarke 变换，得到 i_alpha, i_beta
    float i_alpha = Get_Current_Alpha();
    float i_beta  = Get_Current_Beta();

    // B. 获取上一周期的 α-β 轴施加电压 v_alpha, v_beta (可以由 Udq 经反 Park 变换得到)
    float v_alpha = Get_Voltage_Alpha();
    float v_beta  = Get_Voltage_Beta();

    // C. 运行 SMO 观测器，更新 z_alpha 和 z_beta (即估算的反电动势)
    SMO_Update(&g_smo, v_alpha, v_beta, i_alpha, i_beta);

    // D. 将 SMO 提取到的反电动势输入 PLL 解算角度与转速
    PLL_Update(&g_pll, g_smo.z_alpha, g_smo.z_beta);

    // E. 提取最终控制所需的物理量
    float theta_est = g_pll.theta;     // 用于本周期 Park / 反 Park 变换
    float speed_est = g_pll.speed_lpf; // 用于速度环 PID 控制器的反馈输入
    
    // ... 执行后续 Current Loop (电流环 PID) 和 SVPWM 计算
}

```

---

### SMO 调参要点与细节

1. **滑模增益 $K_{smo}$：**
* **准则：** 必须满足 $K_{smo} > \max(\vert{}E_{\alpha,\beta}\vert{})$。即 $K_{smo}$ 的取值必须大于电机最高转速下的反电动势峰值。
* **现象：** 若 $K_{smo}$ 过小，观测器无法进入滑动模态，电流无法跟随，角度解算彻底崩溃；若过大，会引入剧烈的抖振，增加噪音。


2. **饱和函数边界层 $\epsilon$ (`Epsilon`)：**
* 采用 `sat()` 替代传统的 `sign()` 符号函数，可以在偏差小范围内进行线性连续过渡。
* $\epsilon$ 太小抑制抖振效果不明显；$\epsilon$ 太大会降低观测器的动态响应速度，通常取 `0.1 ~ 2.0`。


3. **低速启动问题：**
* SMO + PLL 属于**中高速无感算法**。当电机零速或极低速时，反电动势 $E \approx 0$，此时 SMO 提取不到有效信号。
* 实际工程中必须配合 **I-F 切无感**、**高频注入法 (HFI)** 或 **开环拖动 (Open-Loop Startup)** 策略将电机拖动至额定转速的 5%~10% 以上，再无缝切换至 SMO+PLL 闭环。

------
-------

无感 FOC 启动的最核心痛点在于：**低速时反电动势极弱，SMO 无法提取有效角度**。如果直接从开环强切到闭环，角度的突变会导致电流环剧烈震荡甚至过流抛错。

工程上最稳妥的解决方案是 **I-F 加速 + 加权平滑切换（Cross-Fading / Soft-Switching）** 策略。

---

### 切换控制四阶段状态机

通过状态机控制启动全过程，将开环角度与 PLL 角度进行加权过渡：

```
 [1. 定位阶段 ALIGN] 
         │ (注入 d 轴电流锁死转子)
         ▼
 [2. I-F 拖动加速阶段 IF_RAMP] ── (SMO/PLL 在后台静默运行)
         │ (当 转速 > 阈值 且 角度差 < 阈值)
         ▼
 [3. 软切换阶段 SWITCHING] ───── (角度进行 alpha 加权渐变插值)
         │ (alpha 从 0.0 线性增加到 1.0)
         ▼
 [4. 全闭环阶段 CLOSED_LOOP] ── (完全由 PLL 角度与转速接管)

```

---

### 关键数学技巧：避免角度跨越 $2\pi$ 突变

在平滑切换阶段，不能直接对角度进行简单加权（例如 $\theta = (1-\alpha)\theta_{open} + \alpha \theta_{pll}$）。因为如果 $\theta_{open} = 6.27$（接近 $2\pi$），而 $\theta_{pll} = 0.01$（刚过 $0$），直接加权会产生极大的反向角度突变。

正确做法是**对角度差值进行周期归一化**：

$$\Delta\theta = \text{WrapToPi}(\theta_{pll} - \theta_{open})$$

$$\theta_{final} = \text{WrapToTwoPi}(\theta_{open} + \alpha \cdot \Delta\theta)$$

---

### C 语言离散化实现

```c
#ifndef FOC_IF_SWITCH_H
#define FOC_IF_SWITCH_H

#include <math.h>
#include "foc_pll.h"
#include "foc_smo.h"

#define TWO_PI 6.283185307179586f
#define PI     3.141592653589793f

typedef enum {
    FOC_STATE_ALIGN = 0,     // 1. 定位阶段
    FOC_STATE_IF_RAMP,       // 2. I-F 加速阶段
    FOC_STATE_SWITCHING,     // 3. 软切换/渐变过渡阶段
    FOC_STATE_CLOSED_LOOP    // 4. 全闭环运行阶段
} FOC_State_t;

typedef struct {
    FOC_State_t state;
    
    // 参数配置
    float Ts;                // 控制周期 (s)
    float align_time;        // 定位持续时间 (s)
    float if_target_rpm;     // I-F 拖动的目标电转速 (RPM)
    float if_accel_rpm_s;    // I-F 加速度 (RPM/s)
    float switch_min_rpm;    // 允许切闭环的最低转速 (RPM)
    float max_allow_err_rad; // 允许切闭环的最大角度偏差 (rad, 通常取 0.2~0.3 rad)
    float switch_duration;   // 过渡平滑时长 (s, 通常取 0.05~0.2s)
    
    // 内部运行变量
    float align_timer;
    float switch_timer;
    float open_rpm;          // 当前开环电转速
    float open_theta;        // 开环强制角度 [0, 2π)
    float blend_alpha;       // 平滑切换加权系数 [0.0 -> 1.0]
    
    // 输出给 Park 变换的最终角度与速度
    float final_theta;
    float final_speed_rpm;
} IF_Switch_TypeDef;

// 将角度差处理到 [-π, π] 之间
static inline float wrap_to_pi(float angle) {
    while (angle > PI)  angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// 将角度限制在 [0, 2π) 之间
static inline float wrap_to_2pi(float angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f)   angle += TWO_PI;
    return angle;
}

/**
 * @brief 初始化切换逻辑结构体
 */
void IF_Switch_Init(IF_Switch_TypeDef *ctrl, float Ts) {
    ctrl->Ts = Ts;
    ctrl->state = FOC_STATE_ALIGN;
    
    ctrl->align_time = 0.5f;           // 定位 0.5 秒
    ctrl->if_target_rpm = 500.0f;      // I-F 加速到 500 RPM
    ctrl->if_accel_rpm_s = 1000.0f;    // 加速度 1000 RPM/s
    ctrl->switch_min_rpm = 300.0f;     // 最低 300 RPM 允许切闭环
    ctrl->max_allow_err_rad = 0.26f;   // 允许最大角度偏差约 15° (0.26 rad)
    ctrl->switch_duration = 0.1f;      // 切换过渡耗时 0.1 秒
    
    ctrl->align_timer = 0.0f;
    ctrl->switch_timer = 0.0f;
    ctrl->open_rpm = 0.0f;
    ctrl->open_theta = 0.0f;
    ctrl->blend_alpha = 0.0f;
    ctrl->final_theta = 0.0f;
    ctrl->final_speed_rpm = 0.0f;
}

/**
 * @brief 切换状态机主迭代函数 (放在 PWM 中断执行)
 */
void IF_Switch_Update(IF_Switch_TypeDef *ctrl, PLL_TypeDef *pll, float *I_q_ref, float *I_d_ref, float I_f_current) {
    switch (ctrl->state) {
        
        // -----------------------------------------------------------------
        // 1. 定位阶段：强行将 d 轴对齐到 0 度，拉牢转子
        // -----------------------------------------------------------------
        case FOC_STATE_ALIGN:
            *I_d_ref = I_f_current;  // 给定预定位电流
            *I_q_ref = 0.0f;
            ctrl->open_theta = 0.0f;
            ctrl->final_theta = 0.0f;
            ctrl->final_speed_rpm = 0.0f;

            ctrl->align_timer += ctrl->Ts;
            if (ctrl->align_timer >= ctrl->align_time) {
                ctrl->align_timer = 0.0f;
                ctrl->state = FOC_STATE_IF_RAMP; // 进入 I-F 加速
            }
            break;

        // -----------------------------------------------------------------
        // 2. I-F 开环拖动加速阶段
        // -----------------------------------------------------------------
        case FOC_STATE_IF_RAMP:
            *I_d_ref = 0.0f;
            *I_q_ref = I_f_current; // 给定交轴拖动电流

            // 速度斜坡递增
            if (ctrl->open_rpm < ctrl->if_target_rpm) {
                ctrl->open_rpm += ctrl->if_accel_rpm_s * ctrl->Ts;
            }

            // 开环角度积分: theta = theta + w * Ts
            float open_omega = ctrl->open_rpm * (TWO_PI / 60.0f);
            ctrl->open_theta += open_omega * ctrl->Ts;
            ctrl->open_theta = wrap_to_2pi(ctrl->open_theta);

            ctrl->final_theta = ctrl->open_theta;
            ctrl->final_speed_rpm = ctrl->open_rpm;

            // 切闭环条件判定：
            // 1) 转速达到预设门限；2) PLL 解算出来的角度与开环角度偏差足够小
            if (ctrl->open_rpm >= ctrl->switch_min_rpm) {
                float angle_err = fabsf(wrap_to_pi(pll->theta - ctrl->open_theta));
                if (angle_err < ctrl->max_allow_err_rad) {
                    ctrl->switch_timer = 0.0f;
                    ctrl->state = FOC_STATE_SWITCHING; // 满足条件，开始过渡
                }
            }
            break;

        // -----------------------------------------------------------------
        // 3. 软切换阶段 (Cross-Fading)
        // -----------------------------------------------------------------
        case FOC_STATE_SWITCHING:
            *I_d_ref = 0.0f;
            *I_q_ref = I_f_current; // 过渡期保持开环电流

            // 切换系数 alpha 从 0.0 线性增至 1.0
            ctrl->switch_timer += ctrl->Ts;
            ctrl->blend_alpha = ctrl->switch_timer / ctrl->switch_duration;

            if (ctrl->blend_alpha >= 1.0f) {
                ctrl->blend_alpha = 1.0f;
                ctrl->state = FOC_STATE_CLOSED_LOOP; // 彻底进入全闭环
            }

            // 维护开环角度基准
            float omega_step = ctrl->open_rpm * (TWO_PI / 60.0f);
            ctrl->open_theta += omega_step * ctrl->Ts;
            ctrl->open_theta = wrap_to_2pi(ctrl->open_theta);

            // 角度无缝插值计算
            float delta_theta = wrap_to_pi(pll->theta - ctrl->open_theta);
            ctrl->final_theta = wrap_to_2pi(ctrl->open_theta + ctrl->blend_alpha * delta_theta);

            // 转速反馈加权
            float pll_rpm = pll->speed_lpf * (60.0f / TWO_PI);
            ctrl->final_speed_rpm = (1.0f - ctrl->blend_alpha) * ctrl->open_rpm + ctrl->blend_alpha * pll_rpm;
            break;

        // -----------------------------------------------------------------
        // 4. 全闭环阶段
        // -----------------------------------------------------------------
        case FOC_STATE_CLOSED_LOOP:
            // 角度与转速完全由 SMO + PLL 提供
            ctrl->final_theta = pll->theta;
            ctrl->final_speed_rpm = pll->speed_lpf * (60.0f / TWO_PI);
            
            // 此阶段电流指令 I_q_ref 将由外部速度环 PID 控制器接管
            break;
    }
}

#endif // FOC_IF_SWITCH_H

```

---

### 中断中的调用顺序

在 PWM 中断中，将 `IF_Switch` 与之前的 `SMO` 和 `PLL` 串联起来：

```c
IF_Switch_TypeDef g_if_switch;
SMO_TypeDef       g_smo;
PLL_TypeDef       g_pll;

void PWM_Interrupt_Handler(void) {
    // 1. 采样与 Clarke 变换
    float i_alpha = Get_Current_Alpha();
    float i_beta  = Get_Current_Beta();
    
    // 2. 获取上一周期的 α-β 轴施加电压
    float v_alpha = Get_Voltage_Alpha();
    float v_beta  = Get_Voltage_Beta();

    // 3. 运行 SMO 观测器
    SMO_Update(&g_smo, v_alpha, v_beta, i_alpha, i_beta);

    // 4. 运行 PLL 解算角度与转速 (即使在开环阶段，PLL 也一直在后台默默跟踪)
    PLL_Update(&g_pll, g_smo.z_alpha, g_smo.z_beta);

    // 5. 切换状态机更新
    float Iq_ref, Id_ref;
    IF_Switch_Update(&g_if_switch, &g_pll, &Iq_ref, &Id_ref, 3.0f /* 开环设定 3A 电流 */);

    // 6. 如果进入全闭环，将速度环 PID 输出赋给 Iq_ref
    if (g_if_switch.state == FOC_STATE_CLOSED_LOOP) {
        float speed_error = Target_Speed_RPM - g_if_switch.final_speed_rpm;
        Iq_ref = Speed_Loop_PID_Calc(speed_error);
    }

    // 7. 使用最终决定的 final_theta 进行 Park 变换与电流环控制
    float theta_for_park = g_if_switch.final_theta;
    
    // ... 执行 Current Loop PID & Inverse Park & SVPWM ...
}

```

---

### 调试关键点与避坑指南

1. **拖动电流 $I_f$ 的设定：** 开环电流 $I_f$ 必须足够大，以克服系统静摩擦力和负载；但又不能过大，否则会导致电机发热严重且开环震荡。通常设定为电机**额定电流的 30%~50%**。
2. **切闭环阈值选择：** 切换转速 `switch_min_rpm` 必须保证 SMO 已经能够输出足够强度的反电动势（通常为额定转速的 5%~10%）。
3. **切换失败/超时保护：** 如果加速到目标转速后，`angle_err` 始终大于 `max_allow_err_rad`（例如负载突变或电机失步），状态机会卡在 `FOC_STATE_IF_RAMP`。工程上必须加入**超时报故障机制**，超时未切入闭环则直接关断 PWM 停机。