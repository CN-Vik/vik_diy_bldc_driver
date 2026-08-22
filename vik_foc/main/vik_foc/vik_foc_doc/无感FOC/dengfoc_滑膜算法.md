可以。你这份代码里的 **DengFOC 滑模观测器（SMO）**，核心目的其实就一句话：

> **不用 AS5600 的角度，利用“电压 + 电流 + 电机 Rs/Ls 模型”反推出反电动势，再由反电动势反推出电角度，最后由角度差得到速度。**

你这份代码不是一个特别标准的“教科书 SMO”，而是一个**简化版电流模型 + 饱和函数滑模 + 低通滤波 + atan 反电动势角度计算**。

我按照你代码真实执行顺序给你拆开。

---

# 一、整个 DengFOC SMO 的信号链

你的 `runFOC()` 实际上是：

```text
        电机
         ↓
     相电流 Ia Ib
         ↓
     Clarke变换
         ↓
   Iα、Iβ
         ↓
┌──────────────────────┐
│      SMO观测器        │
│                      │
│ 已知：Uα Uβ          │
│ 已知：Rs、Ls          │
│ 实测：Iα Iβ           │
│                      │
│ 估计：Eα Eβ           │
└──────────────────────┘
         ↓
   低通滤波 Eα Eβ
         ↓
      atan2
         ↓
     电角度 θe
         ↓
   角度差分 / 机械速度
         ↓
      SMO速度
```

而你现在代码里有一个很重要的特点：

```cpp
S0.Sensor_update();
```

还在运行。

所以你目前实际上是：

```text
AS5600 ─────→ 真实/参考电角度
                  │
                  └── DFOC_M0_Velocity()

电流 + 电压 ─→ SMO ─→ SMO_Est_theta ─→ SMO_Vel
```

也就是说，你现在是拿 **AS5600速度和SMO速度进行对比验证**，并不是已经完全切换成无感FOC。

---

# 二、第一步：采集电机电流

在：

```cpp
CS_M0.getPhaseCurrents();
```

里面得到：

```cpp
CS_M0.current_a
CS_M0.current_b
```

假设：

```text
Ia = U相电流
Ib = V相电流
```

然后：

```cpp
float Ialpha = CS_M0.current_a;

float Ibeta =
    _1_SQRT3 * CS_M0.current_a
    + _2_SQRT3 * CS_M0.current_b;
```

也就是 Clarke 变换：

$$
I_\alpha=I_a
$$

$$
I_\beta=
\frac{1}{\sqrt3}I_a+
\frac{2}{\sqrt3}I_b
$$

也就是：

$$
I_\beta=
\frac{I_a+2I_b}{\sqrt3}
$$

这里是标准的两电阻采样 Clarke 形式之一。

---

# 三、第二步：建立电机电流数学模型

这是你这个 SMO 最核心的地方。

你定义：

```cpp
float Rs = 8.25;
float Ls = 0.004256;
```

也就是：

$$
R_s=8.25\Omega
$$

$$
L_s=4.256mH
$$

电机 αβ 模型：

$$
L_s\frac{dI_\alpha}{dt}
=

U_\alpha-R_sI_\alpha-E_\alpha
$$

整理：

$$
\frac{dI_\alpha}{dt}
=

-\frac{R_s}{L_s}I_\alpha
+
\frac{1}{L_s}(U_\alpha-E_\alpha)
$$

β轴同理：

$$
\frac{dI_\beta}{dt}
=

-\frac{R_s}{L_s}I_\beta
+
\frac{1}{L_s}(U_\beta-E_\beta)
$$

你代码：

```cpp
Est_Ialpha += Ts *
(
    -Rs / Ls * Est_Ialpha
    + 1 / Ls * (SMO_Ualpha - Ealpha)
);
```

对应的就是：

$$
\hat I_\alpha(k+1)
=
\hat I_\alpha(k)
+
T_s
\left[
-\frac{R_s}{L_s}\hat I_\alpha
+
\frac{1}{L_s}(U_\alpha-E_\alpha)
\right]
$$

β轴完全一样。

---

# 四、这里一定要理解：Est_Ialpha 是什么？

这个非常关键。

```cpp
Est_Ialpha
Est_Ibeta
```

不是实际电流。

实际电流：

```cpp
Ialpha
Ibeta
```

来自：

```cpp
CS_M0.current_a
CS_M0.current_b
```

而：

```cpp
Est_Ialpha
Est_Ibeta
```

是：

> **SMO根据电机数学模型“算出来的电流”。**

所以现在有两套电流：

```text
实际电流
   │
   ├── Ialpha
   └── Ibeta

模型计算电流
   │
   ├── Est_Ialpha
   └── Est_Ibeta
```

然后比较：

```cpp
Ialpha_Err = Est_Ialpha - Ialpha;
Ibeta_Err  = Est_Ibeta - Ibeta;
```

也就是：

$$
e_\alpha=\hat I_\alpha-I_\alpha
$$

$$
e_\beta=\hat I_\beta-I_\beta
$$

---

# 五、第三步：滑模控制器开始工作

这两句就是你代码里面真正的“滑模”：

```cpp
Ealpha = h * sat(Ialpha_Err, 0.5f);
Ebeta  = h * sat(Ibeta_Err, 0.5f);
```

先看：

```cpp
sat(err, 0.5f)
```

你的：

```cpp
float sat(float err, float limits) {
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else
    return err / limits;
}
```

其实就是一个**饱和函数**：


$$
sat(e)=
\begin{cases}
1 & e>0.5 \\
e/0.5 & |e|\le 0.5 \\
-1 & e<-0.5
\end{cases}
$$


所以：

```cpp
Ealpha = h * sat(...)
```

实际上相当于：

$$
\hat E_\alpha
=

h\cdot sat(e_\alpha)
$$

β轴：

$$
\hat E_\beta
=

h\cdot sat(e_\beta)
$$

你设置：

```cpp
h = 0.3;
```

所以最大：

$$
|\hat E_\alpha|\le0.3
$$

$$
|\hat E_\beta|\le0.3
$$

---

# 六、为什么电流误差可以拿来估计反电动势？

这是 SMO 最核心的思想。

电机模型：

$$
U=RI+L\frac{dI}{dt}+E
$$

如果你知道：

```text
U
R
L
I
```

理论上就可以求：

$$
E=U-RI-L\frac{dI}{dt}
$$

但实际直接求：

$$
\frac{dI}{dt}
$$

会非常容易受到 ADC 噪声影响。

所以 SMO 不直接微分电流。

它干了一件事：

```text
根据电机模型
       ↓
预测电流
       ↓
和实际电流比较
       ↓
产生电流误差
       ↓
滑模控制器
       ↓
逼近反电动势
```

所以你可以把它理解成：

> **SMO不是直接“测”反电动势，而是通过让模型电流追踪实际电流，反推出能够解释这个电流误差的反电动势。**

---

# 七、第四步：为什么后面又搞了低通滤波？

你这里：

```cpp
Ealpha_flt = 0.1 * Ealpha_flt + 0.9 * Ealpha;
Ebeta_flt  = 0.1 * Ebeta_flt + 0.9 * Ebeta;
```

这是一个一阶 IIR 低通。

数学上：

$$
E_{\alpha,f}(k)
=
0.1E_{\alpha,f}(k-1)
+
0.9E_\alpha(k)
$$

β同理。

但是这里有一个值得你注意的地方：

**这个滤波非常强地偏向当前值。**

因为：

```text
当前值权重 = 0.9
历史值权重 = 0.1
```

所以它更像：

```text
Eraw ──→ 很快跟随 ──→ Eflt
```

而不是很重的低通。

---

# 八、第五步：由反电动势求电角度

你的代码：

```cpp
SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));
SMO_Est_theta = _normalizeAngle(SMO_Est_theta);
```

数学上：

$$
\hat\theta_e
=

-\arctan
\left(
\frac{\hat E_\alpha}{\hat E_\beta}
\right)
$$

这就是你之前问我的：

> 为什么滑膜计算角度使用 atan2，而不是 atan？

这里正好能看到问题。

---

# 九、你这份代码这里其实存在一个明显问题

你现在：

```cpp
atan(Ealpha_flt / Ebeta_flt)
```

不是完整的四象限角度计算。

因为 `atan()` 只能知道：

$$
\frac{E_\alpha}{E_\beta}
$$

这个比值。

它不知道：

```text
Ealpha > 0 / < 0
Ebeta  > 0 / < 0
```

两个轴各自的符号。

比如：

```text
Eα = +1
Eβ = +1
```

和：

```text
Eα = -1
Eβ = -1
```

都有：

$$
E_\alpha/E_\beta=1
$$

所以：

```cpp
atan(1)
```

结果完全一样。

但这两个反电动势向量实际上相差：

$$
180^\circ
$$

这就是为什么无感 FOC 里面通常使用：

```cpp
atan2(Ebeta, Ealpha)
```

而不是：

```cpp
atan(Ealpha / Ebeta)
```

---

# 十、你的 atan 公式为什么还能“跑”？

因为你的代码：

```cpp
SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));
```

实际上隐含了一个角度定义。

假设反电动势：

$$
E_\alpha=-E_m\sin\theta_e
$$

$$
E_\beta=E_m\cos\theta_e
$$

那么：

$$
-\frac{E_\alpha}{E_\beta}
=
\tan\theta_e
$$

所以：

$$
\theta_e
=
-\arctan
\left(
\frac{E_\alpha}{E_\beta}
\right)
$$

**在某些象限里面是成立的。**

但是一旦跨越：

```text
0°
180°
```

就会产生象限问题。

所以你现在：

```cpp
atan()
```

更像是一个**简化版 DengFOC 实现**。

---

# 十一、正确的 atan2 应该怎么理解？

如果你的定义是：

$$
E_\alpha=-E_m\sin\theta
$$

$$
E_\beta=E_m\cos\theta
$$

那么：

```cpp
theta = atan2(-Ealpha, Ebeta);
```

更加合理。

也就是：

```cpp
SMO_Est_theta =
    atan2f(-Ealpha_flt, Ebeta_flt);

SMO_Est_theta =
    _normalizeAngle(SMO_Est_theta);
```

注意这里不是随便改成：

```cpp
atan2(Ealpha, Ebeta)
```

因为你的 αβ → 电角度的定义本身带了负号。

---

# 十二、第六步：SMO角度进入速度计算

你后面：

```cpp
SMOThetaUpdate();
```

里面：

```cpp
float val = SMO_Est_theta;

SMO_angle_prev_ts = micros();

float d_angle = val - SMO_angle_prev;
```

这就是：

$$
\Delta\theta
=

\theta(k)-\theta(k-1)
$$

问题是：

```cpp
theta
```

是：

$$
0\sim2\pi
$$

所以从：

```text
359°
→
1°
```

直接相减：

$$
1-359=-358^\circ
$$

显然不对。

---

# 十三、所以它专门处理 0/2π 跨越

你的代码：

```cpp
if(abs(d_angle) > (0.8f*_2PI))
    SMO_full_rotations +=
        (d_angle > 0) ? -1 : 1;
```

例如：

### 正转：

```text
359°
↓
1°
```

：

$$
\Delta\theta=1-359=-358^\circ
$$

它发现：

```text
abs(d_angle) > 0.8*2π
```

于是：

```cpp
SMO_full_rotations += 1;
```

也就是说：

```text
实际：
359° → 360° → 361°

程序：
359° → 1°
       ↓
   +1圈修正
```

---

# 十四、于是形成“展开角度”

你现在实际上维护：

```cpp
SMO_angle_prev
SMO_full_rotations
```

两者合起来：

$$
\Theta_{unwrap}
=

N\cdot2\pi+\theta
$$

其中：

```text
N = SMO_full_rotations
θ = SMO_angle_prev
```

这就是**展开角度**。

例如：

```text
第一次：
θ = 6.1rad
N = 0

第二次跨过0：
θ = 0.1rad
N = 1
```

于是：

$$
\Theta=1\times2\pi+0.1
$$

所以角度连续了。

---

# 十五、第七步：用展开角度计算速度

你的：

```cpp
getSMOVel()
```

核心：

```cpp
float vel =
(
    (SMO_full_rotations - SMO_vel_full_rotations) * _2PI
    +
    (SMO_angle_prev - SMO_vel_angle_prev)
) / Ts;
```

本质就是：

$$
\omega_e
=

\frac{\Delta\Theta_e}{\Delta t}
$$

得到的是：

$$
rad/s
$$

但是你又：

```cpp
vel = vel/14;
```

这里非常关键。

你的电机是：

```text
12槽14极
```

所以：

$$
p=7
$$

电角速度和机械角速度：

$$
\omega_e=p\omega_m
$$

所以：

$$
\omega_m=\frac{\omega_e}{7}
$$

**但是你的代码除的是 14：**

```cpp
vel = vel/14;
```

这意味着：

> 如果 `SMO_Est_theta` 真的是电角度，那么这里理论上应该除以 **7**，不是 14。

除以 14 等价于把机械速度又缩小了一半。

---

# 十六、这里你一定要重点检查

你前面：

```cpp
SMO_Est_theta
```

是通过：

```cpp
Ealpha
Ebeta
```

计算出来的。

这个角度应该是：

```text
电角度 θe
```

所以：

```cpp
ωe = dθe/dt
```

而：

```text
p = 7
```

因此：

```cpp
ωm = ωe / 7
```

不是：

```cpp
ωe / 14
```

所以如果你的：

```cpp
DFOC_M0_Velocity()
```

单位是机械 rad/s，

那么两者比较：

```cpp
SMO_Vel
DFOC_M0_Velocity()
```

的时候，你现在的 `SMO_Vel` 很可能只有真实机械速度的一半。

---

# 十七、第八步：SMO速度又拿去积分

你的：

```cpp
getSMOTheta();
```

里面：

```cpp
SMO_Theta += ts * SMO_Vel;
```

数学：

$$
\theta_m(k+1)
=

\theta_m(k)+\omega_mT_s
$$

所以：

```text
SMO速度
   ↓
积分
   ↓
SMO_Theta
```

但是这里还有一个重要问题。

你的：

```cpp
SMO_Theta
```

如果使用的是：

```cpp
SMO_Vel
```

而 `SMO_Vel` 已经：

```cpp
vel = vel / 14;
```

那么你积分出来的也是一个**机械角度**，而不是电角度。

所以：

```cpp
SMO_Theta
```

应该是：

```text
机械角度
```

前提是你除以的极对数正确。

---

# 十八、整个 `runFOC()` 的真实执行顺序

你这段代码每次调用：

```cpp
runFOC();
```

实际上是：

### ① 读取 AS5600

```cpp
S0.Sensor_update();
```

得到：

$$
\theta_m
$$

---

### ② 读取电流

```cpp
CS_M0.getPhaseCurrents();
```

得到：

```text
Ia
Ib
```

---

### ③ Clarke

```cpp
Ialpha = Ia;

Ibeta =
Ia / sqrt(3)
+
2Ib / sqrt(3);
```

得到：

$$
I_\alpha,I_\beta
$$

---

### ④ SMO电流模型

```cpp
Est_Ialpha
Est_Ibeta
```

通过：

$$
\frac{d\hat I}{dt}
=
-\frac{R}{L}\hat I+
\frac{1}{L}(U-\hat E)
$$

预测电流。

---

### ⑤ 比较模型电流和实际电流

```cpp
Ialpha_Err = Est_Ialpha - Ialpha;
Ibeta_Err  = Est_Ibeta - Ibeta;
```

得到：

$$
e_\alpha,e_\beta
$$

---

### ⑥ 滑模函数

```cpp
Ealpha = h * sat(Ialpha_Err, 0.5f);
Ebeta  = h * sat(Ibeta_Err, 0.5f);
```

估计：

$$
\hat E_\alpha,\hat E_\beta
$$

---

### ⑦ 低通

```cpp
Ealpha_flt
Ebeta_flt
```

减少滑模带来的抖振。

---

### ⑧ 反正切得到电角度

现在：

```cpp
atan(Ealpha/Ebeta)
```

得到：

$$
\hat\theta_e
$$

---

### ⑨ 处理 0～2π 跳变

```cpp
SMOThetaUpdate();
```

维护：

```cpp
SMO_full_rotations
```

得到连续的展开角度。

---

### ⑩ 角度差分

```cpp
getSMOVel();
```

：

$$
\omega_e=
\frac{\Delta\theta_e}{\Delta t}
$$

---

### ⑪ 电角速度 → 机械角速度

理论：

$$
\omega_m=\frac{\omega_e}{7}
$$

但你代码目前：

```cpp
/14
```

这个值得修改检查。

---

### ⑫ 速度滤波

```cpp
SMO_Vel_Flter(getSMOVel());
```

得到：

```cpp
SMO_Vel
```

---

### ⑬ 速度积分

```cpp
SMO_Theta += ts * SMO_Vel;
```

得到：

```cpp
SMO_Theta
```

---

# 十九、把整个 DengFOC SMO 画成一张图

你这份代码可以理解成：

```text
                 ┌───────────────┐
                 │   PWM控制器    │
                 └───────┬───────┘
                         │
                    Uα / Uβ
                         │
                         ▼
                ┌────────────────┐
                │  电机数学模型   │
                │                │
                │ Rs / Ls        │
                └───────┬────────┘
                        │
                  Est_Iα Est_Iβ
                        │
                        ▼
实际电流 ─→ Clarke ─→ Iα Iβ
                        │
                        ▼
              ┌─────────────────┐
              │   电流误差       │
              │                 │
              │ Est_I - I       │
              └────────┬────────┘
                       │
                       ▼
                ┌─────────────┐
                │ sat()       │
                │ 滑模控制器   │
                └──────┬──────┘
                       │
                       ▼
                  Eα / Eβ
                       │
                       ▼
                  低通滤波
                       │
                       ▼
                atan2(-Eα,Eβ)
                       │
                       ▼
                    θe
                       │
             ┌─────────┴─────────┐
             │                   │
             ▼                   ▼
        角度展开              角度差分
             │                   │
             │                   ▼
             │                 ωe
             │                   │
             │                 / p
             │                   │
             │                   ▼
             │                 ωm
             │                   │
             │                   ▼
             │                 积分
             │                   │
             └──────────────→ θm
```

---

# 二十、但是你这份代码有 5 个地方，我建议你重点改

### ① `atan()` 改 `atan2f()`

现在：

```cpp
SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));
```

建议：

```cpp
SMO_Est_theta =
    atan2f(-Ealpha_flt, Ebeta_flt);

SMO_Est_theta =
    _normalizeAngle(SMO_Est_theta);
```

这样才能正确处理四个象限。

---

### ② `14` 很可能应该是 `7`

你：

```cpp
vel = vel/14;
```

你的电机：

```text
14极
7极对
```

所以：

$$
\omega_m=\frac{\omega_e}{7}
$$

建议至少改成：

```cpp
vel /= M0_PP;
```

而不是写死：

```cpp
vel /= 14;
```

---

### ③ `SMO_Ualpha/Ubeta` 不是实时测量电压

这个特别重要。

你这里：

```cpp
SMO_Ualpha = Ualpha;
SMO_Ubeta = Ubeta;
```

来自：

```cpp
M0_setTorque()
```

所以 SMO 使用的是：

> **你命令给逆变器的电压**

而不是：

> **电机绕组实际得到的电压**

实际电机电压会受到：

```text
MOSFET压降
死区
母线电压变化
PWM非线性
采样延迟
SVPWM误差
```

影响。

低速的时候，这个误差尤其明显。

---

### ④ 你的 SMO 是“电流模型型”的简化实现

它并不是特别完整的工程级 SMO。

你现在：

```cpp
Est_I
    ↓
Ierr
    ↓
sat()
    ↓
E
```

属于比较直接的结构。

真正工程里通常会进一步考虑：

```text
sign/sat函数
↓
滑模增益
↓
反电动势滤波
↓
相位补偿
↓
PLL
↓
电角速度
```

尤其是：

> **SMO + PLL**

通常比：

```cpp
atan2(Eβ,Eα)
```

之后直接差分得到速度更加稳定。

---

### ⑤ 你的 `SMO_Theta` 和 `SMO_Est_theta` 其实是两个不同东西

这个非常容易搞混。

```cpp
SMO_Est_theta
```

是：

> **SMO瞬时估计出来的电角度**

而：

```cpp
SMO_Theta
```

是：

> **把 SMO_Vel 积分出来的角度**

所以：

```text
反电动势
   ↓
SMO_Est_theta
   ↓
差分
   ↓
SMO_Vel
   ↓
积分
   ↓
SMO_Theta
```

这两个不要认为是同一个角度。

---

# 二十一、最后你可以把 DengFOC 这个 SMO 记成 4 个核心公式

实际上整套东西最重要的就是这四组公式。

### ① 电机模型

$$
\boxed{
\frac{d\hat I}{dt}
=
-\frac{R_s}{L_s}\hat I
+
\frac{1}{L_s}(U-\hat E)
}
$$

---

### ② 电流误差

$$
\boxed{
e=\hat I-I
}
$$

---

### ③ 滑模估计反电动势

$$
\boxed{
\hat E=h\cdot sat(e)
}
$$

---

### ④ 反电动势 → 电角度

按照你当前坐标定义：

$$
\boxed{
\hat\theta_e
=

atan2(-\hat E_\alpha,\hat E_\beta)
}
$$

然后：

$$
\boxed{
\omega_e=\frac{d\theta_e}{dt}
}
$$

再：

$$
\boxed{
\omega_m=\frac{\omega_e}{p}
}
$$

你的 2208 是 **14 极、7 极对**，所以这里的 (p=7)。

---

**最关键的一点：**你现在这份 DengFOC 代码，真正的 SMO 核心并不复杂，实际上就是：

> **“我根据 U、Rs、Ls 算一个电流 → 和真实电流比较 → 用滑模函数逼近这个误差对应的反电动势 → 用反电动势方向求电角度。”**

而你现在调试无感 FOC 时，**最值得先改的是 `atan → atan2`、`/14 → /7`，以及确认 `SMO_Ualpha/Ubeta` 的符号和坐标系与你的 FOC 完全一致。**



---------------
----------------


这段代码中的“滑膜控制”本质上是 **滑模观测器（Sliding Mode Observer, SMO）**，用于 FOC 无位置传感器运行时的转子角度/速度估计。下面按代码流程逐步解析。

---

## 一、滑模观测器相关变量

```cpp
float Ealpha, Ebeta;               // α/β 轴反电动势估计
float Est_Ialpha, Est_Ibeta;       // α/β 轴估计电流
float Ealpha_flt, Ebeta_flt;       // 低通滤波后的反电动势
float SMO_Est_theta;               // 估计出的电角度

float Rs = 8.25;                  // 定子电阻
float Ls = 0.004256;              // 定子电感
float h = 0.3;                    // 滑模增益
float SMO_Ualpha, SMO_Ubeta;      // 当前 α/β 轴电压矢量
```

还有角度/速度处理变量：

```cpp
long SMO_angle_prev_ts;            // 上次角度更新时间
float SMO_angle_prev;              // 上次估计电角度
int32_t SMO_full_rotations;        // 累计整圈数（电角度跨圈）
int32_t SMO_vel_full_rotations;    // 速度计算时的上一整圈数
long SMO_vel_angle_prev_ts;        // 速度计算时的上一时间
float SMO_vel_angle_prev;          // 速度计算时的上一角度
float SMO_Vel;                     // 估计速度（机械速度）
float SMO_Theta;                   // 积分得到的连续角度
long SMO_theta_prev;               // 角度积分时间戳
LowPassFilter SMO_Vel_Flter = LowPassFilter(0.05); // 速度低通滤波器
```

---

## 二、电压矢量来源

在 `M0_setTorque()` 中，FOC 计算出 α/β 轴电压后，会保存给滑模观测器使用：

```cpp
float Ualpha = -Uq * sin(angle_el);
float Ubeta =  Uq * cos(angle_el);
SMO_Ualpha = Ualpha;
SMO_Ubeta = Ubeta;
```

所以 `SMO_Ualpha/SMO_Ubeta` 是上一周期电流环输出的电压矢量，作为滑模观测器的输入。

---

## 三、核心流程：`SMO_position_estimate()`

该函数是滑模观测器的主体，每个控制周期调用一次。

### 1. 计算控制周期 Ts

```cpp
uint32_t now_time = micros();
Ts = (now_time - SMO_last_time) * 1e-6f;
SMO_last_time = now_time;
if (Ts < 0 || Ts > 5e-3f) Ts = 1e-3f;
```

根据两次调用之间的微秒时间差得到采样周期 `Ts`，限制最大值防止异常。

### 2. 读取电流并做 Clarke 变换

```cpp
float Ialpha = CS_M0.current_a;
float Ibeta = _1_SQRT3 * CS_M0.current_a + _2_SQRT3 * CS_M0.current_b;
```

将相电流 `Ia、Ib` 变换到 α/β 坐标系：

- `Iα = Ia`
- `Iβ = (1/√3)*Ia + (2/√3)*Ib`

### 3. 滑模电流观测器离散更新

α/β 坐标下的 PMSM 电压方程：

```
dIα/dt = -Rs/Ls * Iα + 1/Ls * (Uα - Eα)
dIβ/dt = -Rs/Ls * Iβ + 1/Ls * (Uβ - Eβ)
```

代码用欧拉法离散：

```cpp
Est_Ialpha += Ts * (-Rs / Ls * Est_Ialpha + 1 / Ls * (SMO_Ualpha - Ealpha));
Est_Ibeta  += Ts * (-Rs / Ls * Est_Ibeta  + 1 / Ls * (SMO_Ubeta  - Ebeta));
```

其中 `Ealpha/Ebeta` 是上一周期计算出的反电动势估计值。

### 4. 计算电流估计误差

```cpp
float Ialpha_Err = Est_Ialpha - Ialpha;
float Ibeta_Err  = Est_Ibeta  - Ibeta;
```

用估计电流减去实际电流，得到误差。

### 5. 滑模控制律生成反电动势

```cpp
Ealpha = h * sat(Ialpha_Err, 0.5f);
Ebeta  = h * sat(Ibeta_Err,  0.5f);
```

`sat()` 函数实现带限幅的符号函数：

```cpp
float sat(float err, float limits) {
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else return err / limits;
}
```

所以：

- 当误差绝对值大于 0.5 时，输出 ±1；
- 否则输出误差/0.5，相当于比例段。

滑模增益 `h = 0.3`，所以反电动势幅值被限制在 ±0.3。

这一步是滑模观测器的核心：利用电流误差的开关特性，使估计电流迅速跟随实际电流，此时控制量 `Eα/Eβ` 就近似等于反电动势。

### 6. 低通滤波反电动势

由于滑模输出含有高频抖振，代码使用一阶低通滤波器：

```cpp
Ealpha_flt = 0.1 * Ealpha_flt + 0.9 * Ealpha;
Ebeta_flt  = 0.1 * Ebeta_flt  + 0.9 * Ebeta;
```

当前值权重 0.9，历史值权重 0.1，相当于对反电动势进行平滑。

### 7. 估计电角度

```cpp
SMO_Est_theta = -atan(Ealpha_flt / Ebeta_flt);
SMO_Est_theta = _normalizeAngle(SMO_Est_theta);
```

利用反电动势矢量在 α/β 轴的分量求反正切，取负号后得到估计电角度，并归一化到 `[0, 2π]`。

> 注意：这里使用的是 `atan`，只能返回 `[-π/2, π/2]`，而不是全象限角度。更好的做法是使用 `atan2(Ealpha_flt, Ebeta_flt)`。

---

## 四、跨圈计数：`SMOThetaUpdate()`

由于电角度被归一化到 `[0, 2π]`，当电机连续旋转时，角度会在 0 和 2π 之间跳变。`SMOThetaUpdate()` 负责累计整圈数。

```cpp
void SMOThetaUpdate() {
  float val = SMO_Est_theta;
  SMO_angle_prev_ts = micros();
  float d_angle = val - SMO_angle_prev;
  if (abs(d_angle) > (0.8f * _2PI))
    SMO_full_rotations += (d_angle > 0) ? -1 : 1;
  SMO_angle_prev = val;
}
```

逻辑：

- 当前角度 `val` 与上次角度 `SMO_angle_prev` 做差；
- 如果差值绝对值超过 `0.8 * 2π`，说明发生了 0 到 2π 或 2π 到 0 的跳变；
- 根据跳变方向增加或减少整圈计数；
- 保存当前角度和时间。

---

## 五、速度估计：`getSMOVel()`

速度由角度差分得到：

```cpp
float getSMOVel() {
  float Ts = (SMO_angle_prev_ts - SMO_vel_angle_prev_ts) * 1e-6;
  if (Ts <= 0) Ts = 1e-3f;

  float vel = ((float)(SMO_full_rotations - SMO_vel_full_rotations) * _2PI
              + (SMO_angle_prev - SMO_vel_angle_prev)) / Ts;
  vel = vel / 14;
  SMO_vel_angle_prev = SMO_angle_prev;
  SMO_vel_full_rotations = SMO_full_rotations;
  SMO_vel_angle_prev_ts = SMO_angle_prev_ts;
  return vel;
}
```

计算步骤：

1. 计算距离上次速度更新的时间差 `Ts`；
2. 角度总变化 = 整圈数变化 × 2π + 当前角度差；
3. 除以时间得到电角速度；
4. 除以 14，将电角速度转换为机械角速度（硬编码 14，实际应为极对数或比例系数，需要根据电机调整）；
5. 更新速度计算基准。

在 `runFOC()` 中，估计速度还会经过低通滤波器：

```cpp
SMO_Vel = SMO_Vel_Flter(getSMOVel());
```

---

## 六、连续角度积分：`getSMOTheta()`

由于直接使用 `SMO_Est_theta` 只有 `[0, 2π]` 范围，且跨圈处理复杂，代码通过积分速度来获得连续角度：

```cpp
void getSMOTheta() {
  long currTime = micros();
  float ts = (currTime - SMO_theta_prev) * 1e-6;
  if (ts <= 0) ts = 1e-3f;
  SMO_Theta += ts * SMO_Vel;
  SMO_Theta = _normalizeAngle(SMO_Theta);
  SMO_theta_prev = currTime;
}
```

也就是：

- 用采样时间乘以估计速度，累加得到角度；
- 再归一化到 `[0, 2π]`。

这样 `SMO_Theta` 就是可用于 FOC 的连续电/机械角度（取决于速度是否已折算）。

---

## 七、`runFOC()` 中的调用顺序

```cpp
void runFOC() {
  S0.Sensor_update();               // 更新 AS5600（用于对比）
  CS_M0.getPhaseCurrents();         // 读取 M0 相电流
  CS_M1.getPhaseCurrents();         // 读取 M1 相电流

  SMO_position_estimate();          // 1. 估计反电动势和电角度

  SMOThetaUpdate();                 // 2. 处理电角度跨圈，累计整圈数
  SMO_Vel = SMO_Vel_Flter(getSMOVel()); // 3. 计算并滤波估计速度
  getSMOTheta();                    // 4. 积分速度得到连续角度

  // 周期性打印对比
  if (cntt++ > 20) {
    cntt = 0;
    Serial.printf("%f,%f\n", SMO_Vel, DFOC_M0_Velocity());
  }
}
```

所以每个控制周期：

```text
传感器/电流更新
   ↓
滑模观测器估计反电动势 + 电角度
   ↓
跨圈计数
   ↓
差分计算速度 + 低通滤波
   ↓
积分得到连续角度
```

---

## 八、注意事项与改进建议

1. **反电动势角度计算**  
   代码使用 `atan(Ealpha_flt / Ebeta_flt)`，只能得到 `[-π/2, π/2]`，建议改为 `atan2(Ealpha_flt, Ebeta_flt)` 或带符号补偿，以获得全象限角度。

2. **低通滤波延迟**  
   反电动势滤波后会产生相位延迟，尤其高速时更明显。高速运行时需要加入角度补偿。

3. **滑模增益与限幅**  
   `h=0.3`、限幅 `0.5` 是经验参数，需要根据电机和电压等级调整。过大会抖振严重，过小则观测器收敛慢。

4. **速度计算中的 14**  
   代码中 `vel = vel / 14`，这个 14 很可能是极对数或减速比，但当前电机参数 `M0_PP=1`，因此实际使用时应改为对应极对数。

5. **电压输入时序**  
   `SMO_Ualpha/Ubeta` 来自上一周期的 FOC 输出。初始时为零，因此滑模观测器需要一定时间收敛，启动阶段可能不可靠。

6. **观测器只针对 M0**  
   目前滑模观测器只使用了 `CS_M0` 的电流和 `M0_setTorque` 的电压，M1 仍使用编码器。

---

## 总结

DengFOC 的滑模观测器流程可以概括为：

> 采样电流 → 克拉克变换 → 电流估计误差 → 滑模控制律生成反电动势 → 低通滤波反电动势 → 反正切求电角度 → 跨圈计数 → 差分求速度 → 低通滤波速度 → 积分求连续角度。

它利用滑模观测器从电流和电压中提取反电动势，从而实现无传感器角度/速度估计。



--------------
---------------
这段代码实现了一个基于 **ESP32** 的双电机 FOC（Field-Oriented Control，磁场定向控制）驱动库，名为 **DengFOC**。它使用 **AS5600** 磁编码器获取转子角度，并集成了 **滑模观测器（SMO，Sliding Mode Observer）** 用于无传感器位置/速度估计。

以下是对代码中 **SMO（滑模控制/观测器）** 流程及其在整个 FOC 框架中的角色的详细解析。

---

### 1. 全局架构与 FOC 基础流程

在深入 SMO 之前，先理解代码的整体框架：

1. **硬件初始化 (`DFOC_Vbus`)**：配置 PWM、I2C 编码器、电流传感器。
2. **校准 (`DFOC_M0_alignSensor`)**：通过注入特定角度的电流，使转子对齐到已知电角度，从而确定编码器零位与电角度零位的偏移。
3. **FOC 核心闭环**：
   - **最内环：电流环** (`DFOC_M0_setTorque`)：控制 Iq（力矩电流）和 Id（励磁电流，代码中设为 0）。
   - **中间环：速度环** (`DFOC_M0_setVelocity`)。
   - **最外环：位置环** (`DFOC_M0_set_Velocity_Angle` 或 `DFOC_M0_set_Force_Angle`)。
4. **FOC 执行函数 (`runFOC`)**：周期性调用，读取传感器、更新 SMO、执行控制算法。

SMO 的作用是：**在不依赖 AS5600 编码器的情况下，通过电机电压和电流模型估计反电动势（BEMF），进而求得转子角度和速度**。这在编码器故障或低成本应用中很有用。代码中同时保留了编码器方案（`DFOC_M0_Velocity` 等）和 SMO 方案，并在 `runFOC` 中打印两者进行对比。

---

### 2. SMO 滑模观测器的数学模型与代码实现

SMO 的核心思想基于永磁同步电机（PMSM）在 α-β 静止坐标系下的电压方程：

$$
\frac{d}{dt}
\begin{bmatrix}
I_\alpha \\
I_\beta
\end{bmatrix}
=
-\frac{R_s}{L_s}
\begin{bmatrix}
I_\alpha \\
I_\beta
\end{bmatrix}
+
\frac{1}{L_s}
\begin{bmatrix}
U_\alpha - E_\alpha \\
U_\beta - E_\beta
\end{bmatrix}
$$

其中：
- \(I_\alpha, I_\beta\)：α-β 轴电流
- \(U_\alpha, U_\beta\)：α-β 轴电压
- \(E_\alpha, E_\beta\)：反电动势（BEMF）
- \(R_s, L_s\)：定子电阻和电感

SMO 通过构建一个观测器模型来估计电流，并通过滑模控制律使估计电流收敛到实际电流，从而间接提取 BEMF。

#### 2.1 变量定义与初始化
```cpp
uint32_t SMO_last_time;
float Ealpha, Ebeta;               // 估计的反电动势
float Est_Ialpha, Est_Ibeta;       // 估计的电流
float Ealpha_flt, Ebeta_flt;       // 低通滤波后的反电动势
float SMO_Est_theta;               // 估计的电角度
float Rs = 8.25;                   // 定子电阻
float Ls = 0.004256;               // 定子电感
float h = 0.3;                     // 滑模增益
float SMO_Ualpha, SMO_Ubeta;       // 输入的α-β轴电压
```

#### 2.2 核心函数 `SMO_position_estimate()`

该函数在 `runFOC()` 中被周期调用（每次 FOC 循环执行一次）。

**步骤 1：计算时间步长 Ts**
```cpp
uint32_t now_time = micros();
Ts = (now_time - SMO_last_time) * 1e-6f;
SMO_last_time = now_time;
if (Ts < 0 || Ts > 5e-3f) Ts = 1e-3f;  // 异常处理，防止过大/过小
```
`Ts` 用于离散积分，表示自上次调用以来的时间间隔（秒）。

**步骤 2：获取 α-β 轴电流**
```cpp
float Ialpha = CS_M0.current_a;
float Ibeta = _1_SQRT3 * CS_M0.current_a + _2_SQRT3 * CS_M0.current_b;
```
这里使用了 Clarke 变换（从三相电流到两相静止坐标系）。假设 `Ia + Ib + Ic = 0`，则：
- \(I_\alpha = I_a\)
- \(I_\beta = \frac{1}{\sqrt{3}}I_a + \frac{2}{\sqrt{3}}I_b\)

**步骤 3：电流观测器更新（滑模观测器核心）**
```cpp
Est_Ialpha += Ts * (-Rs / Ls * Est_Ialpha + 1 / Ls * (SMO_Ualpha - Ealpha));
Est_Ibeta  += Ts * (-Rs / Ls * Est_Ibeta  + 1 / Ls * (SMO_Ubeta  - Ebeta));
```
这是电机电气方程的离散化形式（欧拉积分）。观测器以 `SMO_Ualpha` 和 `SMO_Ubeta`（来自 `M0_setTorque` 中计算并存储的电压）以及估计的 BEMF（`Ealpha`， `Ebeta`）作为输入，更新估计电流。

**步骤 4：计算电流误差**
```cpp
float Ialpha_Err = Est_Ialpha - Ialpha;
float Ibeta_Err  = Est_Ibeta  - Ibeta;
```
这是观测器估计电流与实际测量电流之间的误差。

**步骤 5：滑模控制律（BEMF 估计）**
```cpp
Ealpha = h * sat(Ialpha_Err, 0.5f);
Ebeta  = h * sat(Ibeta_Err, 0.5f);
```
这里使用了饱和函数 `sat` 作为滑模切换函数，目的是减少抖振。`h` 是滑模增益。`sat` 函数定义如下：
```cpp
float sat(float err, float limits) {
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else
    return err / limits;
}
```
当误差绝对值大于 `limits`（0.5）时，输出 ±1（相当于符号函数）；在边界层内（±0.5）则线性过渡。因此，`Ealpha` 和 `Ebeta` 实际上是通过滑模控制律从电流误差中提取出的反电动势估计值。

**步骤 6：低通滤波 BEMF**
```cpp
Ealpha_flt = 0.1 * Ealpha_flt + 0.9 * Ealpha;
Ebeta_flt  = 0.1 * Ebeta_flt  + 0.9 * Ebeta;
```
滑模输出包含高频开关噪声，需要低通滤波。这里采用了一阶低通滤波器（IIR 形式），滤波系数为 0.1（新数据权重 0.9，旧数据权重 0.1）。注意：滤波会引入相位延迟，实际应用中通常需要相位补偿。

**步骤 7：角度估计**
```cpp
SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));
SMO_Est_theta = _normalizeAngle(SMO_Est_theta);
```
利用反电动势的 α-β 分量，通过反正切计算转子电角度。注意这里使用了 `atan(Ealpha / Ebeta)` 并取负，这与坐标系定义和反电动势相位有关。最后归一化到 [0， 2π]。

---

### 3. SMO 速度与位置估计的辅助功能

#### 3.1 `SMOThetaUpdate()` — 处理角度跳变
```cpp
void SMOThetaUpdate(){
  float val = SMO_Est_theta;
  SMO_angle_prev_ts = micros();
  float d_angle = val - SMO_angle_prev;
  if(abs(d_angle) > (0.8f*_2PI) ) SMO_full_rotations += ( d_angle > 0 ) ? -1 : 1; 
  SMO_angle_prev = val;
}
```
- 每次 SMO 更新后调用。
- 检测角度是否发生从 2π 到 0（或反向）的跳变（阈值设为 0.8 * 2π）。
- 如果跳变，则更新累计圈数 `SMO_full_rotations`（正向跳变加 1，反向跳变减 1），以跟踪总机械角度（可超过 2π）。

#### 3.2 `getSMOVel()` — 计算速度
```cpp
float getSMOVel(){
  float Ts = (SMO_angle_prev_ts - SMO_vel_angle_prev_ts)*1e-6;
  if(Ts <= 0) Ts = 1e-3f;
  float vel = ((float)(SMO_full_rotations - SMO_vel_full_rotations)*_2PI + (SMO_angle_prev - SMO_vel_angle_prev)) / Ts; 
  vel = vel / 14;
  SMO_vel_angle_prev = SMO_angle_prev;
  SMO_vel_full_rotations = SMO_full_rotations;
  SMO_vel_angle_prev_ts = SMO_angle_prev_ts;
  return vel;
}
```
- 计算角度差（包含圈数）除以时间差得到角速度（rad/s）。
- **`vel = vel / 14;`**：这里除以 14 是因为电角度与机械角度的关系：`电角度 = 极对数 * 机械角度`。如果电机极对数为 14，则电角速度是机械角速度的 14 倍。该函数返回的是机械角速度（rad/s）。

#### 3.3 `getSMOTheta()` — 连续角度积分
```cpp
void getSMOTheta(){
  long currTime = micros();
  float ts = (currTime - SMO_theta_prev) *1e-6;
  if(ts <= 0) ts = 1e-3f;
  SMO_Theta += ts * SMO_Vel;
  SMO_Theta = _normalizeAngle(SMO_Theta);
  SMO_theta_prev = currTime;
}
```
- 对速度积分得到连续的角度值。
- `SMO_Vel` 在 `runFOC` 中由 `SMO_Vel_Flter(getSMOVel())` 获得（低通滤波后）。

---

### 4. SMO 在 `runFOC()` 中的调用流程

```cpp
void runFOC() {
  S0.Sensor_update();              // 更新 AS5600 编码器角度
  CS_M0.getPhaseCurrents();        // 读取电流传感器
  CS_M1.getPhaseCurrents();

  SMO_position_estimate();         // 核心 SMO 角度估计

  SMOThetaUpdate();                // 更新圈数和原始角度
  SMO_Vel = SMO_Vel_Flter(getSMOVel());  // 计算并滤波速度
  getSMOTheta();                   // 积分得到连续角度

  // 打印对比
  if (cntt++ > 20) {
    cntt = 0;
    Serial.printf("%f,%f\n", SMO_Vel, DFOC_M0_Velocity());
  }
}
```
每次 FOC 循环都会执行这些步骤。SMO 依赖于 `SMO_Ualpha` 和 `SMO_Ubeta`，这两个值在 `M0_setTorque` 中会被更新（见下文），因此 SMO 需要与 FOC 控制同步运行。

---

### 5. SMO 与 FOC 电压输入的关联

SMO 需要知道施加到电机上的电压。在 `M0_setTorque` 中：
```cpp
void M0_setTorque(float Uq, float angle_el) {
  Uq = _constrain(Uq, -(voltage_power_supply)/2, (voltage_power_supply)/2);
  float Ud = 0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  float Ualpha = -Uq * sin(angle_el);
  float Ubeta  =  Uq * cos(angle_el);
  SMO_Ualpha = Ualpha;  // <-- 存储电压供 SMO 使用
  SMO_Ubeta  = Ubeta;
  // 克拉克逆变换 -> PWM
  ...
}
```
这样，SMO 使用的电压与 FOC 实际输出的电压是一致的。

---

### 6. 关键问题与注意事项

#### 6.1 `voltage_power_supply` 初始化顺序问题
代码开头定义了：
```cpp
PIDController vel_loop_M0 = PIDController{ .P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = voltage_power_supply / 2 };
```
但 `voltage_power_supply` 是在 `DFOC_Vbus()` 中才赋值的。C++ 全局变量初始化时 `voltage_power_supply` 默认为 0，因此所有 PID 的 `limit` 都被初始化为 0。必须在 `setup()` 中重新设置这些限制，否则 PID 输出会一直被限制在 0。

#### 6.2 SMO 增益与滤波
- `h = 0.3` 可能需要根据实际电机调整。过大可能振荡，过小收敛慢。
- 低通滤波系数 0.1 是固定值，可能引入相位延迟，导致角度估计滞后。实际无传感器 FOC 中通常需要根据速度进行相位补偿。

#### 6.3 坐标系与角度定义
- `SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));` 这种用法依赖反正切的象限，但 `atan` 返回范围是 [-π/2， π/2]，无法区分四个象限。通常应使用 `atan2(Ebeta_flt, Ealpha_flt)` 或 `atan2(Ealpha_flt, Ebeta_flt)` 结合负号。这里的写法在符号上可能与 FOC 的坐标系定义不完全匹配，需要实际调试验证。

#### 6.4 速度计算的除法
`vel = vel / 14;` 表明电机极对数被硬编码为 14。如果电机极对数不同，此值需要修改。

#### 6.5 电流传感器与 SMO 的耦合
SMO 使用 `CS_M0.current_a` 和 `CS_M0.current_b`。`CS_M0` 是 `CurrSense` 类型（可能基于 ADC），其精度和噪声会直接影响 SMO 的估计效果。

---

### 7. 总结：SMO 流程图谱

1. **输入**：
   - 电压：\(U_\alpha\)， \(U_\beta\)（来自 FOC 输出）
   - 电流：\(I_\alpha\)， \(I_\beta\)（来自传感器）
   - 参数：\(R_s\)， \(L_s\)， 增益 \(h\)
2. **观测器**：
   - 估计电流 \(\hat{I}_{\alpha,\beta}\) 通过电机模型更新。
   - 误差 \(e = \hat{I} - I\) 驱动滑模函数 \(sat(e)\) 生成 BEMF \(E_{\alpha,\beta}\)。
3. **滤波**：
   - BEMF 经低通滤波得到平滑的 \(E_{\alpha,\beta\_flt}\)。
4. **角度提取**：
   - \(\theta_{est} = -\arctan(E_{\alpha\_flt} / E_{\beta\_flt})\) 归一化到 [0， 2π]。
5. **速度与圈数**：
   - `SMOThetaUpdate` 跟踪圈数，处理角度跳变。
   - `getSMOVel` 计算速度（除以极对数得到机械速度）。
   - `getSMOTheta` 积分得到连续角度（可用于位置环）。

这个 SMO 实现是一个基础的滑模反电动势观测器，用于无传感器 FOC。代码中同时保留了编码器测速（`DFOC_M0_Velocity`）用于对比验证 SMO 的精度。


-------------
-----------
------------

### 一、整体概述
这段代码是 DengFOC 中基于**αβ静止坐标系的滑模观测器（SMO, Sliding Mode Observer）**工程实现，用于永磁同步电机（PMSM）的无感FOC控制：无需依赖编码器，仅通过采样定子电流、给定输出电压，即可实时估计转子的电角度与转速。

代码针对 M0 电机实现完整观测链路，采用「电流观测迭代→滑模反电动势估计→反正切解算角度→差分测速→速度积分平滑角度」的经典 SMO 架构，同时保留 AS5600 编码器接口用于观测效果的对比验证。

---

### 二、核心全局变量说明
| 分类 | 变量 | 作用说明 |
|------|------|----------|
| 电机本体参数 | `Rs = 8.25` | 定子相电阻，单位Ω |
| | `Ls = 0.004256` | 定子相电感，单位H |
| | `h = 0.3` | 滑模增益，决定收敛速度与抖振强度 |
| 观测器状态 | `Est_Ialpha / Est_Ibeta` | αβ轴估计电流 |
| | `Ealpha / Ebeta` | 滑模反馈项，相位等价于电机反电动势 |
| | `Ealpha_flt / Ebeta_flt` | 低通滤波后的反电动势信号 |
| | `SMO_Est_theta` | 直接解算的估计电角度（0~2π） |
| | `SMO_Ualpha / SMO_Ubeta` | FOC输出的αβ轴电压（观测器输入量） |
| 角度速度处理 | `SMO_full_rotations` 等 | 圈数计数，处理0~2π角度跳变 |
| | `SMO_Vel` | 最终估计的机械角速度 |
| | `SMO_Theta` | 速度积分得到的平滑电角度 |
| | `SMO_Vel_Flter` | 速度低通滤波器，时间常数0.05s |

---

### 三、完整执行流程（主循环调用顺序）
SMO 的计算在 `runFOC()` 中按固定顺序执行，每个控制周期运行一次：
1. **采样更新**：更新编码器角度、采样电机三相电流
2. **核心滑模估计**：调用 `SMO_position_estimate()`，基于电压电流计算反电动势与电角度
3. **角度连续化**：调用 `SMOThetaUpdate()`，处理 0~2π 的角度跳变，累计圈数
4. **速度计算滤波**：调用 `getSMOVel()` 计算角速度，经低通滤波得到平滑速度
5. **角度平滑积分**：调用 `getSMOTheta()`，用速度积分得到更稳定的角度值
6. **调试输出**：每 20 个控制周期打印一次，对比 SMO 估计速度与编码器真实速度

---

### 四、核心函数逐步骤解析
#### 1. sat 饱和函数
```c
float sat(float err, float limits) {
  if (err > limits) return 1;
  else if (err < -limits) return -1;
  else return err / limits;
}
```
- 作用：**替代传统滑模的符号函数 `sign()`**，在误差较小时切换为线性反馈，大幅削弱滑模固有的高频抖振，是工程化 SMO 的标准优化手段。
- 逻辑：误差超出边界层 `±limits` 时输出 ±1（纯开关模式）；误差在边界层内时线性输出，形成「软开关」。
- 代码中边界层设为 0.5，为调试后的经验值。

#### 2. SMO_position_estimate() 滑模核心估计
这是整个观测器的核心，基于 PMSM αβ 轴电压方程做离散化迭代，共 6 个步骤：

##### 步骤1：计算控制周期 Ts
```c
uint32_t now_time = micros();
Ts = (now_time - SMO_last_time) * 1e-6f;
SMO_last_time = now_time;
if (Ts < 0 || Ts > 5e-3f) Ts = 1e-3f;
```
- 用微秒级时间戳计算本次迭代的时间间隔，转换为秒；
- 异常保护：时间异常时强制设为 1ms，避免迭代发散。

##### 步骤2：采样电流做 Clark 变换
```c
float Ialpha = CS_M0.current_a;
float Ibeta = _1_SQRT3 * CS_M0.current_a + _2_SQRT3 * CS_M0.current_b;
```
- 从三相电流采样中，通过**等幅值 Clark 变换**，将 abc 三相电流转换为 αβ 静止坐标系的两相电流；
- 三线制电机满足 `Ia+Ib+Ic=0`，因此只需采样 A、B 两相即可计算完整的 αβ 电流。

##### 步骤3：电流观测器迭代（前向欧拉离散）
```c
Est_Ialpha += Ts * (-Rs / Ls * Est_Ialpha + 1 / Ls * (SMO_Ualpha - Ealpha));
Est_Ibeta += Ts * (-Rs / Ls * Est_Ibeta + 1 / Ls * (SMO_Ubeta - Ebeta));
```
- 数学来源：PMSM αβ 轴定子电压方程
  $$ u_\alpha = R_s i_\alpha + L_s \frac{di_\alpha}{dt} + e_\alpha $$
  $$ u_\beta = R_s i_\beta + L_s \frac{di_\beta}{dt} + e_\beta $$
- 整理得电流变化率：$\frac{di}{dt} = \frac{u - R_s i - e}{L_s}$
- 代码用**前向欧拉法**离散化：$i_{est}(k+1) = i_{est}(k) + T_s \cdot \frac{u - R_s i_{est}(k) - e(k)}{L_s}$
- 其中 `Ealpha/Ebeta` 是滑模反馈项，用来修正估计电流，迫使估计值跟踪真实电流。

##### 步骤4：计算电流误差，更新滑模反馈
```c
float Ialpha_Err = Est_Ialpha - Ialpha;
float Ibeta_Err = Est_Ibeta - Ibeta;
Ealpha = h * sat(Ialpha_Err, 0.5f);
Ebeta = h * sat(Ibeta_Err, 0.5f);
```
- 计算估计电流与实际采样电流的误差；
- 滑模反馈项 = 滑模增益 h × 饱和函数(电流误差)；
- **物理意义**：当系统进入「滑模面」（电流误差趋近于 0）时，滑模反馈项 `Ealpha/Ebeta` 的相位与电机真实反电动势完全一致，幅值与反电动势成正比。

##### 步骤5：反电动势低通滤波
```c
Ealpha_flt = 0.1 * Ealpha_flt + 0.9 * Ealpha;
Ebeta_flt = 0.1 * Ebeta_flt + 0.9 * Ebeta;
```
- 一阶低通滤波器，滤除滑模抖振带来的高频噪声；
- 属于弱滤波（滤波系数 α=0.1），在保留相位信息的同时削弱尖峰。

##### 步骤6：反正切解算电角度
```c
SMO_Est_theta = (-atan(Ealpha_flt / Ebeta_flt));
SMO_Est_theta = _normalizeAngle(SMO_Est_theta);
```
- PMSM 反电动势与电角度 $\theta_e$ 满足：
  $$ e_\alpha = -K_e \omega_e \sin\theta_e $$
  $$ e_\beta = K_e \omega_e \cos\theta_e $$
- 推导得：$\theta_e = \arctan\left( -\frac{e_\alpha}{e_\beta} \right)$，与代码逻辑完全对应；
- 最后将角度归一化到 `[0, 2π]` 区间。

#### 3. SMOThetaUpdate() 角度连续化处理
反正切得到的角度范围是 0~2π，经过 0 点 / 2π 点时会发生跳变，无法直接计算速度，因此需要做圈数累计：
```c
float d_angle = val - SMO_angle_prev;
if(abs(d_angle) > (0.8f*_2PI) ) SMO_full_rotations += ( d_angle > 0 ) ? -1 : 1; 
SMO_angle_prev = val;
```
- 计算本次角度与上次角度的差值；
- 若差值绝对值超过 0.8×2π，判定为发生角度跳变（如从接近 2π 跳到 0，或反向跳变）；
- 根据跳变方向增减圈数，最终得到**连续无跳变的电角度**（圈数×2π + 当前角度），用于后续速度计算。

#### 4. getSMOVel() 速度计算
基于连续角度的差分法计算角速度：
```c
float vel = ( (float)(SMO_full_rotations - SMO_vel_full_rotations)*_2PI + (SMO_angle_prev - SMO_vel_angle_prev) ) / Ts; 
vel = vel/14;
```
1. 用「圈数差×2π + 角度差」得到总角度变化量，除以时间差得到**电角速度**；
2. 除以 14 转换为**机械角速度**（对应 7 对极电机，电角速度 = 极对数 × 机械角速度）；
3. 更新历史缓存值，为下一次计算做准备。

> 注：此处 `/14` 是硬编码值，与电机极对数强相关，通用性较差。

#### 5. getSMOTheta() 角度平滑积分
```c
SMO_Theta += ts * SMO_Vel;
SMO_Theta = _normalizeAngle(SMO_Theta);
```
- 用滤波后的速度做积分，得到更平滑的电角度；
- 本质是对估计角度做相位跟踪与平滑，削弱反电动势噪声带来的角度抖动，提升控制稳定性。

---

### 五、设计特点与工程局限
#### 关键设计特点
1. **饱和函数优化**：用软开关替代硬开关，有效降低滑模抖振，是工业级 SMO 的通用做法。
2. **极简离散方案**：前向欧拉法计算量极小，适配 ESP32 这类资源有限的单片机。
3. **分层滤波**：反电动势先滤波再解角，速度计算后再加一级低通，层层抑制噪声。
4. **结构清晰**：纯开环反正切解角，没有引入 PLL 锁相环，适合原理学习。

#### 工程局限与注意点
1. **参数依赖性强**：`Rs`、`Ls` 必须与电机实际参数匹配，电阻温漂、电感饱和都会导致观测误差。
2. **低速性能弱**：低速下反电动势幅值极低、信噪比差，反正切解算误差大，是传统 SMO 的普遍短板。
3. **无电压补偿**：直接使用指令电压作为观测器输入，未考虑功率管死区、导通压降等非线性因素。
4. **硬编码参数多**：极对数、滑模增益、边界层厚度均为固定值，适配不同电机需要手动修改调试。

需要我帮你梳理这段 SMO 代码的参数调优步骤吗？


--------------
--------------


