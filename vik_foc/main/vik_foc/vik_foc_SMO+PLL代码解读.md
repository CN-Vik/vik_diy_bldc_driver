有。**你现在这版 SMO + PLL 已经基本成型，但我会明确指出 4 个问题：其中 2 个是必须改的，另外 2 个属于工程稳定性问题。**

先不看注释，只看实际代码逻辑。

---

## 1. SMO 本身：现在的主结构可以

你的实际计算是：

$$
i_{err}=\hat i-i
$$

然后：

$$
z=K_{smo}\operatorname{sat}(i_{err})
$$

再：

$$
\hat i(k+1)
=
\hat i(k)+T_s
\left[
-\frac{R_s}{L_s}\hat i
+\frac{u-z}{L_s}
\right]
$$

最后：

$$
E_\alpha=LPF(z_\alpha)
$$

$$
E_\beta=LPF(z_\beta)
$$

这套结构从数学上是自洽的。

所以**SMO这一部分目前不用大改。**

---

# 2. 第一个必须确认的问题：你的 `z` 到底是不是 `E`

这个非常重要。

你现在：

```c
float z_alpha =
    smo->k_smo *
    SMO_Sat(i_alpha_err, 0.5f);
```

然后：

```c
(u_alpha - z_alpha) / smo->Ls
```

所以你的观测器模型实际上是：

$$
\dot{\hat i}
=
-\frac{R}{L}\hat i+
\frac{u-z}{L}
$$

真实电机：

$$
\dot i
=
-\frac{R}{L}i+
\frac{u-E}{L}
$$

因此理论上：

$$
\boxed{z\approx E}
$$

这个方向是成立的。

所以：

```c
smo->ebmf_alpha = LPF(z_alpha)
smo->ebmf_beta  = LPF(z_beta)
```

**暂时不用取负号。**

但是最终必须用实验验证方向。

---

# 3. 第二个必须注意的问题：你的 SMO `k_smo = 15.5`

你现在：

```c
smo->k_smo = 15.5f;
```

而你的电机之前给出的相电阻大约：

$$
R_s=8.25\Omega
$$

电感：

$$
L_s\approx4.25mH
$$

这里我不会直接告诉你“15.5一定对”或者“一定错”。

因为 `k_smo` 的合理范围和：

* 电流误差单位
* 电压单位
* `Ts`
* 饱和边界 `0.5A`
* 实际电压模型
* PWM电压误差

都有关系。

但你现在：

```c
SMO_Sat(i_err, 0.5f)
```

意味着：

$$
|i_{err}|>0.5A
$$

以后：

$$
z=\pm15.5V
$$

所以你的 SMO 注入量最大就是：

$$
\boxed{|z|=15.5V}
$$

而你的母线是 12V。

这**不是说一定错**，滑模注入量可以作为模型补偿量，并不要求严格小于 VBUS；但是如果长期大量打到 ±15.5V，说明 `k_smo` 偏激进或者模型误差比较大。

所以你调试的时候一定看：

```c
z_alpha
z_beta
```

如果：

```text
+15.5
-15.5
+15.5
-15.5
...
```

疯狂跳，那 SMO 就太激进了。

---

# 4. 你的 PLL：现在数学上是成立的

你现在：

```c
float Ed =
    Ealpha * cos_theta +
    Ebeta  * sin_theta;
```

对应：

$$
E_d=E_\alpha\cos\hat\theta+
E_\beta\sin\hat\theta
$$

然后：

```c
error = -Ed / E_mag;
```

所以：

$$
error=-\frac{E_d}{|E|}
$$

这就是一个**归一化鉴相器**。

这个没问题。

---

# 5. 为什么有的资料用 `Ed`，有的用 `Eq`，有的用叉乘？

你现在先不要被这些名字搞懵。

实际上它们很多时候只是**同一个相位误差的不同表达形式**。

对于你的定义：

$$
E_\alpha=-E\sin\theta
$$

$$
E_\beta=E\cos\theta
$$

你的 PLL：

$$
E_d=
E_\alpha\cos\hat\theta+
E_\beta\sin\hat\theta
$$

推出来：

$$
\boxed{
E_d=E\sin(\hat\theta-\theta)
}
$$

所以：

$$
-\frac{E_d}{E}
=
\sin(\theta-\hat\theta)
$$

而叉乘形式：

$$
E_\alpha\sin\hat\theta
-
E_\beta\cos\hat\theta
$$

本质上也是：

$$
E\sin(\theta-\hat\theta)
$$

所以你看到：

```text
Ed鉴相
叉乘鉴相
Eq鉴相
```

不要先认为是三个完全不同的 PLL。

**关键是看作者的 Eα/Eβ 定义、Park变换定义和正负号。**

---

# 6. 你现在 PLL 真正有一个小问题：`Eq` 没参与控制

你现在：

```c
float Eq =
    -Ealpha * sin_theta +
     Ebeta * cos_theta;
```

然后：

```c
float E_mag = sqrtf(...);
```

实际上：

```text
Ed → 鉴相
Eq → 仅调试
E_mag → 归一化
```

这是完全可以的。

所以：

```c
pll->Eq = Eq;
```

保留即可。

你甚至可以用它判断 PLL 是否锁住：

$$
\boxed{Ed\rightarrow0}
$$

同时：

$$
\boxed{Eq\rightarrow E}
$$

这是非常好的调试指标。

---

# 7. 你的 PLL 最大的问题反而是积分器

现在：

```c
pll->ki_integral +=
    pll->ki * error * pll->Ts;

pll->we =
    pll->kp * error +
    pll->ki_integral;
```

如果你明确决定：

> **不要 `OMEGA_MAX` 速度限幅**

那没问题，完全可以去掉。

但是我建议至少给：

```c
ki_integral
```

做一个**合理的内部保护**。

因为：

```text
SMO异常
 ↓
E方向错误
 ↓
PLL error长期不为0
 ↓
Ki不断积分
 ↓
we越来越大
 ↓
theta疯狂跑
```

如果不做任何保护，调试阶段很容易出现这种情况。

可以先不限制 `we`，只保护积分器，例如：

```c
if (pll->ki_integral > 5000.0f)
    pll->ki_integral = 5000.0f;

if (pll->ki_integral < -5000.0f)
    pll->ki_integral = -5000.0f;
```

这不是速度限制。

它只是防止：

$$
\boxed{积分器无限发散}
$$

---

# 8. 还有一个非常重要的问题：低速时你把 `we` 留在了旧值

你的代码：

```c
if(E_mag > 0.1f)
{
    ...
    pll->we = ...
    pll->theta_e += ...
}
else
{
    error = 0.0f;
}
```

假设：

```text
电机高速
we = 500rad/s
```

突然：

```text
E_mag < 0.1
```

那么你的：

```c
we
```

仍然是：

```text
500rad/s
```

只是停止积分。

这本身不一定出事故，因为角度也没有继续积分。

但调试时容易误解。

我建议改成：

```c
else
{
    error = 0.0f;
    pll->we = 0.0f;
}
```

这样状态更干净。

---

# 9. 还有一个地方建议你改：`E_mag > 0.1`

你的：

```c
if(E_mag > 0.1f)
```

不是错误。

但它是一个**固定电压阈值**。

你这个电机反电势比较小，尤其低速时：

```text
E 很小
```

那么：

```text
E < 0.1V
```

PLL直接停止。

所以这个值以后需要结合实际 SMO 输出观察。

建议先保留：

```c
#define PLL_E_MIN 0.05f
```

或者：

```c
#define PLL_E_MIN 0.1f
```

方便调。

---

# 10. 你的 PLL 初始化还应该明确

你的 `PLL_Init()` 目前没有贴出来。

应该至少：

```c
void PLL_Init(pll_t *pll)
{
    pll->Kp = ...;
    pll->Ki = ...;

    pll->Ed = 0.0f;
    pll->Eq = 0.0f;
    pll->error = 0.0f;

    pll->theta_e = 0.0f;
    pll->we = 0.0f;

    pll->ki_integral = 0.0f;

    pll->Ts = 1.0f / 20000.0f;
}
```

---

# 11. 我建议你最终把 PLL 写成这个版本

既然你明确说：

> **不要速度限幅**

那就干脆把 `OMEGA_MAX` 全部去掉：

```c
float PLL_Update(pll_t *pll, float Ealpha, float Ebeta)
{
    if (pll == NULL)
    {
        return 0.0f;
    }

    /* 当前PLL角度 */
    float theta = pll->theta_e;

    /* sin / cos */
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    /* αβ -> dq */
    float Ed =
        Ealpha * cos_theta +
        Ebeta  * sin_theta;

    float Eq =
       -Ealpha * sin_theta +
        Ebeta  * cos_theta;

    /* 反电动势幅值 */
    float E_mag =
        sqrtf(Ealpha * Ealpha +
              Ebeta  * Ebeta);

    float error = 0.0f;

    if (E_mag > 0.1f)
    {
        /*
         * 鉴相器
         */
        error = -Ed / E_mag;

        /*
         * PI积分
         */
        pll->ki_integral +=
            pll->Ki *
            error *
            pll->Ts;

        /*
         * 防止积分器无限发散
         * 这里只保护积分器，不限制电机速度
         */
        if (pll->ki_integral > 5000.0f)
            pll->ki_integral = 5000.0f;

        if (pll->ki_integral < -5000.0f)
            pll->ki_integral = -5000.0f;

        /*
         * 电角速度
         */
        pll->we =
            pll->Kp * error +
            pll->ki_integral;

        /*
         * 积分得到电角度
         */
        pll->theta_e +=
            pll->we * pll->Ts;
    }
    else
    {
        /*
         * 反电动势太小，
         * 暂停PLL积分
         */
        error = 0.0f;
        pll->we = 0.0f;
    }

    /* Angle Wrap */
    while (pll->theta_e >= FOC_2PI)
        pll->theta_e -= FOC_2PI;

    while (pll->theta_e < 0.0f)
        pll->theta_e += FOC_2PI;

    /* 调试变量 */
    pll->Ed = Ed;
    pll->Eq = Eq;
    pll->error = error;

    return pll->theta_e;
}
```

---

# 12. 最后，你这套算法现在的“正确版本”就是

你不要再纠结各种资料了，你现在可以把自己的算法固定成：

```text
              电机
               ↓
          Iα / Iβ
               ↓
             SMO
               ↓
        ┌──────┴──────┐
        ↓             ↓
       Zα             Zβ
        ↓             ↓
       LPF            LPF
        ↓             ↓
       Eα             Eβ
        └──────┬──────┘
               ↓
             PLL
               ↓
        ┌──────┴──────┐
        ↓             ↓
       Ed             Eq
        ↓
   -Ed / |E|
        ↓
       PI
        ↓
       ωe
        ↓
      积分
        ↓
      θpll
        ↓
    FOC Park
```

其中最核心的数学关系：

$$
\boxed{
E_d
=
E_\alpha\cos\theta_{pll}
+
E_\beta\sin\theta_{pll}
}
$$

$$
\boxed{
error=-\frac{E_d}{|E|}
}
$$

$$
\boxed{
\omega_e=K_p error+\int K_i error\,dt
}
$$

$$
\boxed{
\theta_e=\int\omega_e\,dt
}
$$

**这套完全可以作为你现在 ESP32 20 kHz 无感 FOC 的第一版 SMO+PLL。**

不过有一个地方我特别提醒你：**现在不要急着调 PLL 的 Kp/Ki。先把 `Ealpha/Ebeta`、`Ed/Eq/error` 打出来。** 如果 `Eα/Eβ` 本身方向或符号错了，PLL 再怎么调都没用。

最理想的测试结果应该是：

```text
Ealpha  ─── 正弦
Ebeta   ─── 余弦（相差90°）

Ed      ─── → 0
Eq      ─── 稳定非零
error   ─── → 0
we      ─── 稳定
theta   ─── 连续旋转
```

如果你把这几个波形跑出来，我就可以直接帮你判断**到底是 SMO 有问题、Eα/Eβ 符号反了，还是 PLL 的鉴相方向反了**。




---------------
-----------------
---------------------

可以。你现在这套 **SMO + PLL**，调参不要把所有参数一起乱调。最重要的是分成两层：

```text
第一层：SMO
Rs、Ls、Ts
k_smo
Sat 的 delta
LPF

第二层：PLL
Kp
Ki
E_mag阈值
```

而且**一定是先把 SMO 调好，再调 PLL**。因为 PLL 只是跟踪 SMO 给它的 `Eα/Eβ`；SMO 输出本身乱，PLL 调得再漂亮也没用。实际工程资料也是先校准/调好 observer，再确认位置跟踪，然后再调 PLL/速度估计器。([MathWorks][1])

---

# 一、先把你现在这些参数分别搞明白

你现在核心参数：

```c
smo->Rs = MOTOR_RS;
smo->Ls = MOTOR_LS;
smo->Ts = SMO_TS;

smo->k_smo = 15.5f;

SMO_Sat(i_err, 0.5f);

low_pas_filter(0.45f, ...);
```

PLL：

```c
pll->Kp
pll->Ki
pll->Ts
```

它们分别控制：

| 参数      | 控制什么      | 太小      | 太大       |
| ------- | --------- | ------- | -------- |
| `Rs`    | 电机模型      | E估计偏差   | E估计偏差    |
| `Ls`    | 电机模型      | E估计偏差   | E估计偏差    |
| `k_smo` | SMO收敛力度   | 跟不上     | 抖、噪声大    |
| `delta` | SMO边界层    | 抖动      | 估计变软、误差大 |
| LPF     | E滤波       | 噪声大     | 延迟大      |
| `Kp`    | PLL追踪速度   | 跟踪慢     | 抖/震荡     |
| `Ki`    | PLL长期锁定能力 | 锁定慢     | 低频摆动     |
| `E_min` | PLL启动门槛   | 噪声进入PLL | 低速不工作    |

SMO 的核心确实存在一个经典权衡：**增益越大，收敛/鲁棒性越强，但高频噪声和抖振也更严重**。([OPUS][2])

---

# 二、第一步：Rs、Ls 不要调

你的电机：

```text
Rs ≈ 8.25 Ω
Ls ≈ 4.25 mH
```

先直接使用实际测量/识别值。

也就是说：

```c
smo->Rs = MOTOR_RS;
smo->Ls = MOTOR_LS;
```

先不要拿它们当调参旋钮。

因为：

$$
\boxed{Rs/Ls}
$$

直接决定你的电流观测器模型。

如果 Rs/Ls 本身错很多，你会发现：

> `k_smo` 怎么调都感觉不对。

---

# 三、第二步：先调 `k_smo`

这是你 SMO 最重要的参数。

你的代码：

```c
z_alpha = k_smo * sat(i_alpha_err, delta);
z_beta  = k_smo * sat(i_beta_err, delta);
```

所以：

$$
\boxed{k_{smo}}
$$

决定：

> **SMO 有多“狠”地把电流估计误差压回去。**

---

## 怎么调？

### 先把 PLL 完全关闭

不要：

```text
SMO → PLL → FOC
```

先只看：

```text
SMO → Eα/Eβ
```

电机用**有感角度**或者开环方式稳定运行。

然后逐渐：

```text
k_smo = 5
       ↓
8
       ↓
10
       ↓
12
       ↓
15
       ↓
18
       ↓
20
```

不要一次改一大堆。

---

# 四、`k_smo` 调到什么现象算合适？

你主要观察：

```text
Ealpha
Ebeta
```

以及：

```text
i_alpha_est - i_alpha
i_beta_est - i_beta
```

### `k_smo` 太小

你会看到：

```text
i_err：

~~~~~~~
   /\
  /  \
 /    \
```

电流估计明显跟不上。

`Eα/Eβ`：

```text
畸变严重
```

---

### `k_smo` 合适

```text
i_est ≈ i_real
```

然后：

```text
Ealpha ≈ 正弦
Ebeta  ≈ 余弦
```

而且两者：

$$
\boxed{\text{相差约90°}}
$$

这才是你真正想要的。

---

### `k_smo` 太大

例如：

```text
k_smo = 30
50
100
```

可能出现：

```text
Eα  ~~~~~~~/\/\/\/\/
Eβ  ~~~~~~~/\/\/\/\/
```

然后电机：

```text
啸叫
抖动
电流毛刺
```

这就是典型的 SMO 收敛速度和 chattering 之间的权衡。([Itegam Jetia][3])

---

# 五、你现在 `k_smo=15.5` 怎么办？

**先别动。**

你的第一组实验：

```c
k_smo = 15.5f;
```

完全可以作为起点。

然后跑电机，比如：

```text
300 RPM
500 RPM
800 RPM
1000 RPM
```

观察：

```text
Ealpha
Ebeta
i_alpha_err
i_beta_err
```

如果已经很干净，就不用为了“理论参数”硬改。

---

# 六、第三步调 `delta`

你现在：

```c
SMO_Sat(i_alpha_err, 0.5f)
```

也就是：

$$
\boxed{\delta=0.5A}
$$

这个参数非常关键。

你的饱和函数：

$$
sat(x)=
\begin{cases}
1&x>\delta\\
x/\delta&|x|\leq\delta\\
-1&x<-\delta
\end{cases}
$$

所以：

### delta 小

例如：

```c
delta = 0.05f;
```

稍微一点电流误差就接近：

```text
+1 / -1
```

SMO 更接近传统 `sign()`。

优点：

> 收敛狠。

缺点：

> 抖振大。

---

### delta 大

例如：

```c
delta = 1.0f;
```

滑模控制变柔和。

优点：

> 平滑。

缺点：

> 估计误差可能变大。

---

# 七、你的 `0.5A` 我建议先别急着改

先：

```c
delta = 0.5f;
```

跑起来观察：

```text
i_alpha_err
i_beta_err
```

如果你发现绝大多数时间：

```text
i_err = ±0.02A
±0.05A
±0.1A
```

那么你的 `delta=0.5A` 就比较宽。

如果：

```text
i_err = ±0.5A
±1A
±2A
```

那就说明 SMO 本身可能没调好，**不是简单改 delta 就能解决。**

所以：

> **不要拿 delta 去掩盖 k_smo 或模型参数的问题。**

---

# 八、第四步调 SMO 的 LPF

你现在：

```c
low_pas_filter(0.45f, old, z);
```

这里我不知道你的 `low_pas_filter()` 具体实现。

所以：

$$
\boxed{0.45}
$$

到底代表：

* α = 0.45
* cutoff
* 还是某种内部系数

目前不能仅凭这一行确定。

**这个很重要。**

如果你的函数是：

```c
y = y + alpha * (x-y);
```

那么：

```c
0.45
```

已经属于比较“快”的滤波。

如果它是：

```c
LPF(fc)
```

那意义完全不同。

所以你把 `low_pas_filter()` 函数贴出来，我可以直接告诉你这个 `0.45` 对应多少 Hz。

---

# 九、但是 LPF 有一个核心原则

SMO：

```text
k_smo ↑
 ↓
E噪声 ↑
```

于是你想：

```text
LPF ↑
```

把噪声滤掉。

但是：

```text
LPF太强
 ↓
Eα/Eβ相位延迟
 ↓
PLL角度偏
```

所以：

$$
\boxed{
LPF不是越强越好
}
$$

SMO 文献里也明确把 chattering 与滤波/边界层之间的权衡作为重要设计问题。([Itegam Jetia][3])

---

# 十、SMO 调好以后，再调 PLL

这时候你固定：

```text
Rs
Ls
Ts
k_smo
delta
LPF
```

只调：

```text
Kp
Ki
```

这是最重要的调参原则：

$$
\boxed{
先SMO，后PLL
}
$$

---

# 十一、PLL 的 Kp 怎么调？

你的 PLL：

$$
error=-\frac{E_d}{|E|}
$$

然后：

$$
\omega_e=K_p error+I
$$

所以：

$$
\boxed{Kp}
$$

决定：

> **PLL追踪角度的“反应速度”。**

---

## 调法

先：

```c
Ki = 0;
```

然后：

```text
Kp = 10
20
50
100
200
300
500
```

逐渐增加。

---

### Kp 太小

你会看到：

```text
真实角度：

/ / / / /

PLL：

  / / / / /
```

明显滞后。

`Ed` 长时间不容易回到 0。

---

### Kp 合适

```text
Ed → 0

error → 0
```

而：

```text
theta_PLL
```

能够平滑跟踪。

---

### Kp 太大

出现：

```text
theta：

~~~~/\/\/\/~~~~
```

或者：

```text
we：

/\/\/\/\/\
```

然后电机：

```text
声音变尖
电流抖动
```

这就是 PLL 带宽过高，把 SMO 的噪声也跟进来了。

---

# 十二、然后再调 Ki

Kp 找到合适之后：

```c
Ki = 0;
```

然后逐渐增加：

```text
Ki：

1000
2000
5000
10000
20000
50000
```

观察：

```text
error
we
theta
Ed
```

---

### Ki 太小

PLL：

```text
锁得慢
```

速度变化以后：

```text
theta_PLL
```

恢复比较慢。

---

### Ki 合适

```text
Ed → 0
error → 0
we → 稳定
```

---

### Ki 太大

你会看到：

```text
we：

~~~~/\/\/\/~~~~
```

甚至：

```text
theta：

左右摆动
```

电机可能：

```text
啸叫
电流震荡
```

---

# 十三、你这个 PLL 最简单的调参口诀

直接记：

```text
Ki = 0
   ↓
调 Kp
   ↓
找到“不抖但跟得上”的最大附近
   ↓
固定 Kp
   ↓
慢慢加 Ki
   ↓
找到“不摆但锁得快”的值
```

也就是：

$$
\boxed{
先P，后I
}
$$

---

# 十四、对于你这个 20kHz FOC，我不建议一上来用理论公式硬算

理论上可以按照二阶 PLL：

$$
K_p=2\zeta\omega_n
$$

$$
K_i=\omega_n^2
$$

设计。

但你现在这个系统还有：

```text
SMO
+
Sat
+
LPF
+
PWM电压误差
+
ADC噪声
+
ESP32采样延迟
```

所以实际 PLL 的有效动态特性已经不是一个“纯理想二阶 PLL”。

因此你现在更适合：

> **理论给初值 + 实测微调。**

---

# 十五、你这个系统我建议从这一组开始

按照你之前给我的参数：

```text
2208
100KV
7极对
Rs ≈ 8.25Ω
Ls ≈ 4.25mH
VBUS = 12V
FOC = 20kHz
Ts = 50us
```

我建议第一版：

### SMO

```c
Rs       = 8.25f;
Ls       = 0.00425f;
Ts       = 0.00005f;

k_smo    = 15.5f;

delta    = 0.5f;
```

先不要改。

---

### PLL

我反而建议你先不要上特别激进的：

```text
Kp = 1000
Ki = 400000
```

先从：

```c
Kp = 100.0f;
Ki = 5000.0f;
```

开始。

然后：

```text
Kp：
100
150
200
300
```

找到合适点。

再：

```text
Ki：
5000
10000
20000
30000
50000
```

一点一点加。

**这只是你当前实现的工程起始值，不是“理论唯一正确值”。**

---

# 十六、真正调参的时候，你只看这几个波形

我强烈建议你在 ESP32 里把：

```c
i_alpha_err
i_beta_err

z_alpha
z_beta

ebmf_alpha
ebmf_beta

Ed
Eq
error

we
theta_e
```

全部做成可观察变量。

然后按照下面这个判断表：

| 现象              | 优先调整              |
| --------------- | ----------------- |
| `i_est` 跟不上实际电流 | `k_smo ↑`         |
| `z` 高频疯狂跳       | `k_smo ↓`         |
| Eα/Eβ 噪声大       | `k_smo ↓` / LPF增强 |
| Eα/Eβ明显滞后       | LPF减弱             |
| Eα/Eβ不是90°关系    | 先检查SMO/电压电流模型     |
| PLL角度明显滞后       | `Kp ↑`            |
| PLL角度抖          | `Kp ↓`            |
| PLL长期有偏差        | `Ki ↑`            |
| PLL低频摆动         | `Ki ↓`            |
| `we`疯狂跳         | PLL带宽太高/SMO噪声大    |
| `Ed`长期不接近0      | PLL方向、SMO方向或参数有问题 |

---

# 十七、最重要的一点：别拿“电机能不能转”作为唯一判断标准

真正调好的 SMO+PLL 应该是：

$$
\boxed{E_\alpha,E_\beta\text{ 干净}}
$$

↓

$$
\boxed{E_d\rightarrow0}
$$

↓

$$
\boxed{error\rightarrow0}
$$

↓

$$
\boxed{w_e\text{ 平稳}}
$$

↓

$$
\boxed{\theta_{PLL}\text{ 连续}}
$$

↓

$$
\boxed{i_d\approx0,\quad i_q\text{稳定}}
$$

↓

电机平稳运行。

实际厂商调参资料也通常把**位置跟踪误差、速度阻尼以及最终电流波形**作为调参是否成功的重要判断依据，而不是单纯看“电机转没转”。([MathWorks][4])

---

## 你现在可以直接按照这个顺序干

```text
① Rs / Ls
     ↓
② k_smo
     ↓
③ delta
     ↓
④ SMO LPF
     ↓
⑤ 看 Eα/Eβ
     ↓
──────────────
确认 SMO OK
──────────────
     ↓
⑥ Ki = 0
     ↓
⑦ 调 Kp
     ↓
⑧ 固定 Kp
     ↓
⑨ 调 Ki
     ↓
⑩ 看 Ed / error / we
     ↓
最终接入 FOC
```

**你现在最关键的不是继续改代码，而是把 `low_pas_filter()` 的实现贴出来。**因为你现在的 `0.45f` 到底是“滤波系数”还是“截止频率”，决定了我能不能准确告诉你 **SMO 的 LPF 应该设多少**。



-----------
-------------

针对你当前的 ESP32 + 2208-80T 云台电机 + 20kHz 采样频率（$T_s = 50\mu s$），在开始调参之前，**必须先修正你代码中的一个结构性闭环 Bug**，否则任何参数都调不出来。

### 0. 调参前置条件：修正 SMO 结构 Bug

你在之前的代码中，每次进入 `SMO_Update()` 都会把 `ebmf_alpha_est` 和 `ebmf_beta_est` 清零，然后再带入下一次电流模型计算。
**这会导致 SMO 变成开环系统**，滑模电压没有反馈给电流观测器。
**修正方法：** 必须把 `ebmf_alpha_est` 和 `ebmf_beta_est` 设为**全局变量**或**结构体成员**，保留上一次的计算结果，直接带入当前的电流观测方程中。

---

### 1. SMO 参数设置 ($K_{smo}$ 与 $\epsilon$)

SMO 只有两个核心参数需要调：滑模增益 $K_{smo}$ 和 边界层厚度 $\epsilon$（你的代码里叫 `delta`）。

* **$K_{smo}$ (滑模增益)：**
* **物理意义：** $K_{smo}$ 代表滑模观测器能输出的**最大反电动势电压**。
* **设置规则：** 必须大于电机在你目标最高转速下产生的反电动势峰值，但不能大太多，否则会引入极大的高频抖振（导致电机尖叫）。
* **你的电机 (2208-80T)：** 这是一个高内阻、低反电势的云台电机。如果在 12V 供电下运行，最大反电势通常只有几伏特。
* **推荐初值：** 设为 **`8.0f ~ 12.0f`**。如果你发现电机在高速时突然失步，说明反电势超过了 $K_{smo}$，需要适当增大；如果在低速时电机啸叫严重，适当减小。


* **$\epsilon$ 或 `delta` (边界层厚度)：**
* **物理意义：** 决定了电流误差在多大范围内采用线性平滑过渡（代替生硬的 sign 符号函数）。单位是安培(A)。
* **设置规则：** 参考你系统的电流采样噪声水平。
* **推荐初值：** 设为 **`0.1f ~ 0.3f`**。如果设得太小（比如 0.01），系统会像原始 sign 函数一样高频抖振；设得太大（比如 2.0），SMO 会变得迟钝，产生严重的相位滞后。



---

### 2. PLL 参数设置 ($K_p$ 与 $K_i$)

PLL 的参数取决于你期望的**系统带宽**。你的 FOC 运行在 20kHz（$T_s = 50\mu s$），可以将 PLL 的带宽目标定在 $30\text{Hz}$ 左右。

根据标准二阶系统推导公式（阻尼比 $\zeta = 0.707$，带宽 $\omega_n = 2\pi \times 30 \approx 188.5\text{ rad/s}$）：


$$K_p = 2 \times \zeta \times \omega_n \approx 266$$

$$K_i = \omega_n^2 \approx 35530$$

**结合你的云台电机特性，推荐的起步参数如下：**

| 参数 | 推荐初值 | 调节现象与方向 |
| --- | --- | --- |
| **$K_p$ (比例)** | **`150.0f ~ 250.0f`** | **决定角度跟踪的快慢。**<br>

<br>若加速时电机易卡顿失步（跟踪慢），**增大 $K_p$**。<br>

<br>若电机匀速运行时发出高频“滋滋”声或 $I_q$ 电流剧烈震荡，**减小 $K_p$**。 |
| **$K_i$ (积分)** | **`3000.0f ~ 5000.0f`** | **决定稳态相位误差的消除。**<br>

<br>若估算角度和实际角度始终有一个固定偏差，**增大 $K_i$**。<br>

<br>若转速出现低频的“呼吸式”波动（转速忽高忽低），说明积分过饱和，**减小 $K_i$**。 |

*注：上述 $K_i$ 是连续域的值。在代码中如果你的积分公式是 `integral += Ki * error * Ts`，则直接填入 `3000 ~ 5000` 即可。*

---

### 3. 回应：#PLL误差计算如何选择Ed

关于你标签里的疑问，之所以在你的系统中必须选择 $E_d$（或者 $-E_d$）作为相位误差 `error`，是由你的 SMO 反电动势定义直接决定的：

你的代码逻辑导出的反电势方程是：


$$E_\alpha = -E\sin\theta$$

$$E_\beta = E\cos\theta$$

将这个方程代入标准的 Park 变换 $d$ 轴公式中：


$$E_d = E_\alpha\cos\hat\theta + E_\beta\sin\hat\theta$$


得出结果为：


$$E_d = E\sin(\hat\theta - \theta)$$

**因为当估算角度 $\hat\theta$ 等于真实角度 $\theta$ 时，$E_d$ 完美趋近于 $0$。** 因此，利用 $\frac{E_d}{\vert{}E\vert{}}$ 作为 PLL 的误差输入，就能让 PLL 的 PI 控制器自动寻优，直到相位误差归零。你看到的所谓“叉乘鉴相器” $-E_\alpha\cos\hat\theta - E_\beta\sin\hat\theta$，本质上就是 $-E_d$，只是符号取向不同，最终都能通过调节 $K_p$ 极性来正常锁相。



------------
---------------
--------------
----------------

# SMO+PLL当前存在的问题优化
这段无感 FOC 的 SMO（滑模观测器）与 PLL（锁相环）代码整体框架清晰，包含了标准滑模观测器的基本要素（电流误差计算、饱和函数、状态更新、滤波与角度提取）。

但在**算法控制原理**和**嵌入式工程实现**上，存在几处**会导致电机无法正常运行、高速失步或系统卡死的关键缺陷**。

---

### 一、 核心算法缺陷（影响电机运行与稳定性）

#### 1. 低通滤波器（LPF）相位滞后未补偿（致命问题）

在 `SMO_Update` 中使用了低通滤波器提取反电动势：

```c
smo->ebmf_alpha = low_pas_filter(0.45f, smo->ebmf_alpha, z_alpha);

```

* **原理缺陷**：任何一阶低通滤波器都会对信号产生**相位滞后**。滞后角 $\phi$ 与电机当前电角速度 $\omega_e$ 和 LPF 截止频率 $\omega_c$ 相关：$\phi = \arctan\left(\frac{\omega_e}{\omega_c}\right)$。
* **实际后果**：电机转速越高，提取出的反电动势滞后越严重，导致估计出的电角度偏离真实位置。这会引发 FOC 的 $d/q$ 轴解耦失效、电机发热、效率剧降，甚至高速下直接失步断相。
* **修复方案**：必须根据当前转速计算相位补偿量 $\theta_{comp} = \arctan\left(\frac{\omega_e}{\omega_c}\right)$，并加到最终估计角度上；或者将 LPF 截止频率设计为随转速动态变化的自适应滤波器。

#### 2. 饱和函数边界层 $\delta = 0.5\text{A}$ 过大

在滑模控制量计算中：

```c
float z_alpha = smo->k_smo * SMO_Sat(i_alpha_err, 0.5f);

```

* **原理缺陷**：`SMO_Sat(err, delta)` 的边界层设为了 `0.5f`。当电流估算误差小于 $0.5\text{A}$ 时，函数进入线性区：$Z = k_{smo} \cdot \frac{i_{err}}{0.5} = 25 \cdot i_{err}$。
* **实际后果**：在大多数中小功率电机中，稳态电流误差通常小于 $0.5\text{A}$。这导致观测器完全退化为普通线性比例控制器，失去了滑模控制强鲁棒性提取反电动势的能力，估计出的反电动势幅值会严重偏小、变软。
* **修复方案**：缩小边界层 $\delta$（通常设为 $0.01\text{A} \sim 0.05\text{A}$，或额定电流的 1%~3%），仅用微小的线性区来抑制高频抖振（Chattering）。

#### 3. LPF 滤波系数 `0.45f` 偏大，高频噪声无法滤除

* **原理缺陷**：若 `low_pas_filter` 采用标准的数字一阶 LPF 形式 $y_k = (1-\alpha)y_{k-1} + \alpha x_k$，$\alpha = 0.45$ 意味着新采样值的权重高达 45%。
* **实际后果**：假设开关/载波频率为 $20\text{kHz}$（$T_s = 50\mu\text{s}$），此系数对应的截止频率高达到 $1.5\text{kHz}$ 以上，根本无法有效滤除滑模开关量 $Z$ 的高频高幅值开关噪声。
* **修复方案**：根据期望截止频率 $f_c$ 计算系数：$\alpha = 2\pi \cdot f_c \cdot T_s$。例如 $T_s = 50\mu\text{s}, f_c = 100\text{Hz}$ 时，$\alpha \approx 0.0314$。

---

### 二、 PLL 锁相环实现隐患

#### 1. 低速/停机状态下 PLL 状态未复位

在 `PLL_Update` 中：

```c
if(E_mag > 0.1f) {
    ...
} else {
    error = 0.0f;
    pll->we = 0.0f;
}

```

* **原理缺陷**：当反电动势过小（$E_{mag} \le 0.1\text{V}$，低速或切入阶段）时，代码将 `pll->we` 置 0，但**没有更新 `pll->theta_e**`（角度冻结），且**没有清空积分器 `pll->ki_integral**`。
* **实际后果**：电机从低速重新加速、反电动势再次超过 $0.1\text{V}$ 的瞬间，PLL 会带着上次残余的巨大积分值重新工作，导致估算转速和角度发生严重突变（跳变），引起相电流冲击。
* **修复方案**：当 $E_{mag} \le 0.1\text{V}$ 时，清空 `pll->ki_integral = 0.0f`，并让 `theta_e` 保持顺畅积分或由外部 Open-Loop（如 I/F 启动算法）接管。

#### 2. SMO 与 PLL 架构混淆（重复计算角度）

* 在 `SMO_Update` 中已经通过 `-atan2f(ebmf_alpha, ebmf_beta)` 计算了一次 `smo->theta_e`。
* 在 `PLL_Update` 中又通过 Park 变换鉴相器计算了一次 `pll->theta_e`。
* **建议**：在标准的无感 FOC 中，应明确二者分工：
* **SMO 职责**：仅负责根据电流误差更新，并滤出连续的反电动势信号 $E_\alpha, E_\beta$。
* **PLL 职责**：输入 $E_\alpha, E_\beta$，通过锁相环提取出平滑无滞后的电角度 $\theta_e$ 和电角速度 $\omega_e$。无需在 SMO 内部再调用 `atan2f`。



---

### 三、 嵌入式工程与安全性问题

#### 1. `while` 循环在中断服务函数（ISR）中的死锁风险

在 `PLL_Update` 结尾：

```c
while(pll->theta_e >= FOC_2PI)
    pll->theta_e -= FOC_2PI;

while(pll->theta_e < 0.0f)
    pll->theta_e += FOC_2PI;

```

* **工程风险**：若系统受到强干扰，导致 `pll->theta_e` 变为 `NaN`（非数）或 `Inf`（无穷大），`while` 比较条件会永远成立或产生异常，导致控制中断陷入死循环，MCU 看门狗复位或桥臂直通烧毁 MOSFET。
* **修复方案**：在 PWM/ADC 高频中断中，单次周期角度增量极小，改用 `if` 条件语句即可，或使用 `fmodf`。

---

### 四、 改进后的参考代码实现

以下为修正后的 **SMO + PLL** 标准配合架构：

```c
/**
 * @brief 滑模观测器更新（仅负责提取反电动势）
 */
void SMO_Update(smo_ctrl_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta)
{
    if (smo == NULL) return;

    /* 1. 电流估计误差 */
    float i_alpha_err = smo->i_alpha_est - i_alpha;
    float i_beta_err  = smo->i_beta_est  - i_beta;

    /* 2. 滑模控制量 (缩小 delta 边界层至 0.02A，增强滑模效果) */
    const float delta = 0.02f; 
    float z_alpha = smo->k_smo * SMO_Sat(i_alpha_err, delta);
    float z_beta  = smo->k_smo * SMO_Sat(i_beta_err, delta);

    /* 3. 状态方程更新 (前向欧拉离散化) */
    smo->i_alpha_est += smo->Ts * ((-smo->Rs / smo->Ls) * smo->i_alpha_est + (u_alpha - z_alpha) / smo->Ls);
    smo->i_beta_est  += smo->Ts * ((-smo->Rs / smo->Ls) * smo->i_beta_est  + (u_beta - z_beta) / smo->Ls);

    /* 4. 低通滤波提取反电动势 (合理计算截止频率，例如 fc = 200Hz) */
    /* lpf_alpha = 2 * PI * fc * Ts */
    float lpf_alpha = 2.0f * FOC_PI * 200.0f * smo->Ts; 
    smo->ebmf_alpha += lpf_alpha * (z_alpha - smo->ebmf_alpha);
    smo->ebmf_beta  += lpf_alpha * (z_beta  - smo->ebmf_beta);
}

/**
 * @brief 基于反电动势的锁相环 (带滤波相位滞后补偿)
 */
float PLL_Update(pll_t *pll, float Ealpha, float Ebeta)
{
    if (pll == NULL) return 0.0f;

    float sin_theta = sinf(pll->theta_e);
    float cos_theta = cosf(pll->theta_e);

    /* 1. 坐标变换 ( Park 变换 ) */
    float Ed =  (Ealpha * cos_theta) + (Ebeta * sin_theta);
    float E_mag = sqrtf((Ealpha * Ealpha) + (Ebeta * Ebeta));

    /* 2. 鉴相器与 PI 控制 */
    if (E_mag > 0.1f) 
    {
        /* 归一化相位误差 */
        pll->error = -Ed / E_mag;

        /* PI 积分与速度提取 */
        pll->ki_integral += pll->ki * pll->error * pll->Ts;
        
        /* 积分限幅 */
        if (pll->ki_integral > 5000.0f)  pll->ki_integral = 5000.0f;
        if (pll->ki_integral < -5000.0f) pll->ki_integral = -5000.0f;

        pll->we = (pll->kp * pll->error) + pll->ki_integral;
    } 
    else 
    {
        /* 低速状态复位，防止切入突变 */
        pll->error = 0.0f;
        pll->ki_integral = 0.0f;
        pll->we = 0.0f;
    }

    /* 3. 角度累加 */
    pll->theta_e += pll->we * pll->Ts;

    /* 4. 消除 SMO LPF 带来的相位滞后补偿 */
    /* theta_compensated = theta_pll + atan(we / w_cutoff) */
    float w_cutoff = 2.0f * FOC_PI * 200.0f; // 与 SMO LPF 截止频率一致
    float phase_delay = atan2f(pll->we, w_cutoff);
    
    float theta_out = pll->theta_e + phase_delay;

    /* 5. 角度规范化 [0, 2π) (安全校验，避免 while 无限循环) */
    if (theta_out >= FOC_2PI) theta_out -= FOC_2PI;
    if (theta_out < 0.0f)      theta_out += FOC_2PI;

    if (pll->theta_e >= FOC_2PI) pll->theta_e -= FOC_2PI;
    if (pll->theta_e < 0.0f)      pll->theta_e += FOC_2PI;

    return theta_out;
}

```



------------
------------
整体看下来，你这个 **SMO → Eα/Eβ → PLL → 电角度** 的架构是对的，而且你的 PLL 鉴相器**并不是简单地“写错了”**，因为你前面的 `-atan2f(Ealpha,Ebeta)` 已经定义了一套特殊的反电动势坐标关系。

但是，里面有 **3 个比较关键的问题/隐患**，其中第一个我认为是你现在最应该改的。

---

# 先给结论

你的结构：

```text
电压 Uαβ
   ↓
┌────────────┐
│    SMO     │
│ 电流观测器 │
└────────────┘
   ↓
 Zα Zβ
   ↓ LPF
 Eα Eβ
   ↓
┌────────────┐
│    PLL     │
│ 鉴相 + PI  │
└────────────┘
   ↓
 θe
   ↓
 FOC Park
```

**总体结构正确。**

但是：

| 项目                 | 评价                   |
| ------------------ | -------------------- |
| SMO 电流观测方程         | ✅ 基本正确               |
| `Z = K·sat(error)` | ✅ 可以                 |
| Z → LPF → Eαβ      | ✅ 正确思路               |
| `-atan2(Eα,Eβ)`    | ⚠️ 只是调试角度，不建议再作为正式链路 |
| PLL Park           | ✅ 正确                 |
| `-Ed/E_mag`        | ⚠️ **在你当前角度定义下可以成立** |
| PLL 积分             | ✅                    |
| PLL 低速处理           | ⚠️ 有明显问题             |
| PLL 参数             | ⚠️ 需要根据采样周期重新核算      |
| SMO `k_smo=12.5`   | ⚠️ 很值得重新调            |
| LPF `0.45f`        | ⚠️ 必须确认这个参数到底是什么意思   |

---

# 一、你 SMO 最核心的数学结构是对的

你的电机 αβ 电压模型：

$$
u_\alpha=R_s i_\alpha+L_s\frac{di_\alpha}{dt}+e_\alpha
$$

所以：

$$
\frac{di_\alpha}{dt}
=
-\frac{R_s}{L_s}i_\alpha+
\frac{u_\alpha-e_\alpha}{L_s}
$$

SMO 用一个滑模项 `Z` 去逼近反电动势：

$$
\hat e_\alpha \approx Z_\alpha
$$

于是：

$$
\frac{d\hat i_\alpha}{dt}
=
-\frac{R_s}{L_s}\hat i_\alpha+
\frac{u_\alpha-Z_\alpha}{L_s}
$$

你代码：

```c
smo->i_alpha_est +=
    smo->Ts *
    (
        (-smo->Rs / smo->Ls) *
        smo->i_alpha_est
        +
        (u_alpha - z_alpha) /
        smo->Ls
    );
```

这个就是：

$$
\boxed{
\dot{\hat i}
=
-\frac{R}{L}\hat i+
\frac{u-Z}{L}
}
$$

**没问题。**

---

# 二、你的滑模误差方向也没明显问题

你：

```c
float i_alpha_err =
    smo->i_alpha_est - i_alpha;
```

也就是：

$$
s=\hat i-i
$$

然后：

```c
z = k * sat(s)
```

这个符号和你后面的：

```c
u - z
```

是配套的。

所以不要看到别人 SMO 写：

```c
i - i_hat
```

就直接把你这里改过去。

**你的误差定义、Z 的符号、观测器里的 `u-z` 是一个整体。**

---

# 三、但是 `k_smo = 12.5f` 这个值需要重点关注

你的：

```c
smo->k_smo = 12.5f;
```

它实际上不是一个普通的“无量纲增益”。

因为：

```c
z = k_smo * sat(...)
```

最终：

$$
Z
$$

要和反电动势：

$$
E
$$

同量纲，所以 `k_smo` 实际上应该是 **电压量级**。

也就是说：

```text
k_smo ≈ 能覆盖正常运行时反电动势的电压范围
```

如果：

```text
k_smo 太小
```

那么：

```text
Z 无法跟踪 E
        ↓
Eαβ 被削顶
        ↓
PLL 相位失真
        ↓
角度抖动
```

如果：

```text
k_smo 太大
```

那么：

```text
SMO注入非常强
        ↓
电流估计剧烈切换
        ↓
Z高频抖振很大
        ↓
LPF压力增大
        ↓
角度噪声增大
```

所以你之前出现的 **SMO 输出角度抖、噪声大、电机嗡嗡响**，这个参数就非常值得检查。

---

# 四、你这里 `sat()` 的 0.5A 也很关键

你现在：

```c
SMO_Sat(i_alpha_err, 0.5f);
```

相当于：

$$
sat(s)=
\begin{cases}
1&s>0.5\\
s/0.5&|s|\le0.5\\
-1&s<-0.5
\end{cases}
$$

也就是你设置了：

$$
\delta=0.5A
$$

这个实际上是 **滑模边界层厚度**。

你的电流误差：

```text
±0.5A
```

以内，SMO 不使用真正的 `sign()`，而是线性化。

这可以减少抖振。

但如果你的实际电机工作电流只有：

```text
0.2A ~ 0.5A
```

那么这个：

```c
delta = 0.5f
```

就可能偏大。

例如：

```text
实际电流 = 0.3A
估计误差 = 0.1A
```

那么：

$$
sat=\frac{0.1}{0.5}=0.2
$$

滑模作用只有：

$$
Z=12.5\times0.2=2.5V
$$

这时候 SMO 的非线性校正并不是特别强。

所以这个参数不要固定死。

---

# 五、你 SMO 最大的问题：你现在其实做了“两套角度计算”

你这里：

```c
smo->theta_e =
    -atan2f(
        smo->ebmf_alpha,
        smo->ebmf_beta
    );
```

然后：

```c
PLL_Update(
    pll,
    smo->ebmf_alpha,
    smo->ebmf_beta
);
```

实际上是：

```text
SMO
 ↓
Eα Eβ
 ├────────→ atan2 → θSMO
 │
 └────────→ PLL → θPLL
```

如果最终 FOC 使用：

```c
theta = PLL_Update(...)
```

那么：

### `SMO` 里面完全没必要再计算正式的 `theta_e`

建议改成：

```c
SMO
 ↓
Ealpha
Ebeta
 ↓
PLL
 ↓
theta_e
```

SMO 只负责：

> **估计反电动势矢量**

PLL 只负责：

> **从反电动势矢量中提取连续电角度和电角速度**

这才是标准的职责划分。

---

# 六、最重要的问题：你的 PLL 鉴相器到底对不对？

你现在：

```c
float Ed =
    (Ealpha * cos_theta) +
    (Ebeta  * sin_theta);

float Eq =
    (-Ealpha * sin_theta) +
    (Ebeta  * cos_theta);
```

这就是：

$$
\begin{bmatrix}
E_d\\
E_q
\end{bmatrix}
=
Park(E_{\alpha\beta},\theta_{PLL})
$$

这个没问题。

然后你：

```c
error = -Ed / E_mag;
```

很多人看到这里会直接说：

> “错了，应该用 Eq。”

**但是对你这个代码不能这么简单判断。**

因为你前面的角度定义：

```c
theta = -atan2(Ealpha, Ebeta)
```

实际上对应的是：

$$
E_\alpha=-E\sin\theta
$$

$$
E_\beta=E\cos\theta
$$

也就是说，你的反电动势矢量是：

```text
        Eβ
        ↑
        |
        |      E
        |     /
        |    /
        |   /
        |  /
        +------------→ Eα
```

只不过它相对于普通：

$$
E_\alpha=E\cos\theta
$$

$$
E_\beta=E\sin\theta
$$

多了一个 90° 的定义关系。

---

# 七、按照你当前定义推导一下，就很清楚了

假设真实角度：

$$
\theta
$$

PLL估计：

$$
\hat\theta
$$

你的反电动势：

$$
E_\alpha=-E\sin\theta
$$

$$
E_\beta=E\cos\theta
$$

代入你的 `Ed`：

$$
E_d
=
E_\alpha\cos\hat\theta+
E_\beta\sin\hat\theta
$$

得到：

$$
E_d
=
-E\sin\theta\cos\hat\theta
+
E\cos\theta\sin\hat\theta
$$

利用：

$$
\sin A\cos B-\cos A\sin B
=
\sin(A-B)
$$

所以：

$$
E_d
=
-E\sin(\theta-\hat\theta)
$$

因此：

$$
-\frac{E_d}{E}
=
\sin(\theta-\hat\theta)
$$

这正好就是：

$$
\boxed{
error=\sin(\theta-\hat\theta)
}
$$

所以：

```c
error = -Ed / E_mag;
```

**在你当前的坐标定义下，是成立的。**

---

# 八、这也是你这个 PLL 最核心的地方

你的 PLL 实际上在干：

```text
SMO得到反电动势矢量
        ↓
     Eα Eβ
        ↓
拿“自己猜的角度 θPLL”
        ↓
   Park变换
        ↓
       Ed
        ↓
   -Ed / |E|
        ↓
     相位误差
        ↓
       PI
        ↓
     电角速度 ωe
        ↓
     积分
        ↓
     电角度 θe
```

真正的核心就是：

$$
\boxed{
error=\sin(\theta_{true}-\theta_{PLL})
}
$$

当：

$$
\theta_{PLL}=\theta_{true}
$$

那么：

$$
error=0
$$

于是：

```text
PI输出稳定
     ↓
we ≈ 实际电角速度
     ↓
theta持续积分
```

这才是 PLL 锁住。

---

# 九、但是你这里有一个容易踩坑的地方：PLL不能再依赖 atan2

你的 PLL 现在实际上已经可以做到：

```text
Eαβ
 ↓
相位误差
 ↓
PI
 ↓
we
 ↓
积分
 ↓
θ
```

这是正确的。

**不要再这样：**

```c
theta = atan2f(Ealpha, Ebeta);
PLL(theta);
```

否则 PLL 就失去意义了。

你现在虽然没有这么做，但 `SMO_Update()` 里保留：

```c
smo->theta_e =
    -atan2f(...)
```

很容易以后自己把它接回 FOC。

建议直接删掉 SMO 的：

```c
theta_e
```

---

# 十、你 PLL 低速部分存在一个比较明显的问题

你现在：

```c
if(E_mag > 0.1f)
{
    ...
}
else
{
    error = 0.0f;
    pll->we = 0.0f;
}
```

也就是说：

```text
反电动势 < 0.1V
       ↓
PLL直接认为：
we = 0
```

这个逻辑对于**停止/低速**可以理解。

但是运行过程中：

```text
E = 0.11V
```

突然：

```text
E = 0.09V
```

那么：

```text
we → 0
```

再：

```text
E = 0.11V
```

又恢复 PLL。

这样容易产生：

```text
角度突然停住
      ↓
又开始跑
      ↓
角度跳变
```

尤其你做 sensorless FOC 的时候，低速区域本来就是 SMO 最难工作的地方。

---

# 十一、另外一个问题：PLL积分器的限幅

你：

```c
if (pll->ki_integral > 5000.0f)
    pll->ki_integral = 5000.0f;
```

如果单位是：

$$
rad/s
$$

那：

```text
5000 rad/s
```

对应：

$$
RPM_e
=
5000 \times \frac{60}{2\pi}
\approx47746
$$

如果你的电机：

$$
p=7
$$

机械转速约：

$$
RPM_m
=
\frac{47746}{7}
\approx6821RPM
$$

所以对于你的这颗 100KV、12V、7对极电机，这个限制已经非常高了。

它不是不能用，但：

> **这个 5000 更像“防积分发散保护”，而不是正常工作范围。**

这点你注释里写得是对的。

---

# 十二、你的 PLL 参数反而是比较合理的一组

你：

```c
kp = 200.0f;
ki = 3553.0f;
```

如果：

```c
Ts = 50us
```

那么：

$$
\omega_n=\sqrt{K_i}
$$

所以：

$$
\omega_n=\sqrt{3553}
\approx59.6rad/s
$$

约：

$$
9.5Hz
$$

阻尼比：

$$
\zeta=
\frac{K_p}{2\sqrt{K_i}}
$$

得到：

$$
\zeta\approx1.68
$$

所以这是一个：

> **响应比较稳、阻尼比较大的 PLL。**

不是特别激进。

如果你以后发现：

```text
角度跟踪慢
```

可以提高：

```text
Kp
Ki
```

如果：

```text
角度抖动很大
```

优先检查：

```text
SMO噪声
LPF
K_smo
```

而不是一上来疯狂降低 PLL 参数。

---

# 十三、但是有一个非常重要的问题：你的 LPF `0.45f`

你：

```c
smo->ebmf_alpha =
    low_pas_filter(
        0.45f,
        smo->ebmf_alpha,
        z_alpha
    );
```

这里我现在**不能直接判断 0.45 是否合理**。

因为要看你的：

```c
low_pas_filter()
```

到底怎么实现。

如果是：

```c
y = y_old + alpha * (x - y_old);
```

那么：

```text
alpha = 0.45
```

非常大。

相当于：

```text
新值占45%
旧值占55%
```

滤波其实不算特别强。

如果你的 `0.45` 是截止频率参数，那又是另一回事。

所以你最好把：

```c
low_pas_filter()
```

函数也发给我。

这个东西对你 SMO 的最终效果影响非常大。

---

# 十四、我建议你最终把结构改成这样

我更建议你把现在代码整理成：

```text
                ┌──────────────┐
Uα Uβ ─────────→│              │
                │     SMO      │
Iα Iβ ─────────→│              │
                │              │
                └──────┬───────┘
                       │
                    Eα Eβ
                       │
                       ↓
                ┌──────────────┐
                │     LPF      │
                └──────┬───────┘
                       │
                    Eα Eβ
                       │
                       ↓
                ┌──────────────┐
                │     PLL      │
                │              │
                │ Park         │
                │ ↓            │
                │ Ed/Eq        │
                │ ↓            │
                │ Phase Error  │
                │ ↓            │
                │ PI           │
                │ ↓            │
                │ ωe           │
                │ ↓            │
                │ Integrator   │
                └──────┬───────┘
                       │
                    θe / ωe
                       │
              ┌────────┴────────┐
              ↓                 ↓
          FOC Park          Speed Loop
```

---

# 十五、我尤其建议你把 SMO 改成“只输出 Eα/Eβ”

比如：

```c
typedef struct
{
    float Rs;
    float Ls;
    float Ts;

    float k_smo;
    float boundary;

    float i_alpha_est;
    float i_beta_est;

    float e_alpha;
    float e_beta;

} smo_ctrl_t;
```

然后：

```c
void SMO_Update(...)
{
    ...

    // 电流误差
    err_alpha = i_alpha_est - i_alpha;
    err_beta  = i_beta_est  - i_beta;

    // 滑模注入
    z_alpha = k_smo * sat(err_alpha, boundary);
    z_beta  = k_smo * sat(err_beta, boundary);

    // 电流观测
    i_alpha_est += Ts *
        (
            -Rs/Ls * i_alpha_est
            + (u_alpha - z_alpha) / Ls
        );

    i_beta_est += Ts *
        (
            -Rs/Ls * i_beta_est
            + (u_beta - z_beta) / Ls
        );

    // LPF
    e_alpha = LPF(z_alpha);
    e_beta  = LPF(z_beta);
}
```

然后 PLL：

```c
theta_e = PLL_Update(
    &pll,
    smo.e_alpha,
    smo.e_beta
);
```

这样整个系统的职责就非常清楚。

---

# 十六、还有一个非常关键的问题：你现在的 SMO 电压输入必须是真实电压

这个对你这种 ESP32 FOC 尤其重要。

你的：

```c
u_alpha
u_beta
```

不能简单理解成：

```text
Uq × sin/cos
```

而应该尽可能是：

$$
U_{\alpha\beta}
$$

**实际施加到电机绕组上的基波电压。**

至少需要考虑：

```text
SVPWM输出
   ↓
实际占空比
   ↓
母线电压 Vbus
   ↓
Uαβ
```

如果：

```text
SMO使用的Uαβ ≠ 电机实际得到的Uαβ
```

那么 SMO 会把这个电压误差：

```text
电压模型误差
```

当成：

```text
反电动势
```

最后：

```text
Eαβ错误
 ↓
PLL错误
 ↓
θe错误
 ↓
FOC噪声
```

这其实是 **SMO 最常见的问题之一**。

---

# 十七、结合你之前的情况，我认为你现在优先检查这 5 个东西

你的情况不是单纯“PLL公式写错”。

我会按照这个优先级排查：

### ① 第一优先级：确认 `Eα/Eβ` 符号和旋转方向

打印：

```text
Ealpha
Ebeta
theta_PLL
we
Ed
Eq
error
```

手动转电机。

正常情况下：

```text
Eαβ
```

应该形成一个比较干净的圆。

---

### ② 第二优先级：确认 PLL 锁定时

你的结构应该满足：

```text
Ed ≈ 0
```

同时：

```text
Eq ≈ E_mag
```

以及：

```text
error ≈ 0
```

这是你现在最值得观察的一组量。

---

### ③ 第三优先级：确认 PLL 的 `we`

锁定以后：

$$
\boxed{
\omega_e \approx p\omega_m
}
$$

你的电机：

```text
p = 7
```

所以如果机械：

```text
1000 RPM
```

那么：

$$
\omega_m=
1000\frac{2\pi}{60}
$$

电角速度：

$$
\omega_e=
7\omega_m
$$

大约：

$$
733rad/s
$$

PLL 的：

```c
pll->we
```

应该接近这个量。

---

### ④ 第四优先级：检查 `low_pas_filter()`

这个我强烈建议你下一条直接发出来。

因为：

```c
0.45f
```

到底合理不合理，现在无法单独判断。

---

### ⑤ 第五优先级：重新调 `k_smo`

你现在：

```c
k_smo = 12.5f
```

不要认为这是一个固定正确值。

应该结合：

```text
Rs
Ls
Ts
实际电流
实际反电动势
PWM频率
LPF
```

一起调。

---

# 最后给你一个最简洁的判断

你现在这套：

```text
SMO
 ↓
Zαβ
 ↓
LPF
 ↓
Eαβ
 ↓
PLL
 ↓
θe
```

**大方向完全正确。**

其中最容易误判的是：

```c
error = -Ed / E_mag;
```

**你的这句在当前 `theta = -atan2(Ealpha,Ebeta)` 的坐标定义下是可以成立的。** 因为它最终确实形成：

$$
\boxed{
error=\sin(\theta_{true}-\theta_{PLL})
}
$$

真正需要你重点处理的是：

> **SMO 参数、LPF、Eαβ符号/方向、以及低速切换。**

而且你现在 `SMO_Update()` 里面这个：

```c
smo->theta_e = -atan2f(...);
```

我建议**删掉**，让 SMO 只输出 `Eα/Eβ`，PLL 独占 `θe` 和 `we`。

**你下一条把 `low_pas_filter()` 的代码也贴出来，我可以直接结合你这颗 2208-80T、7 对极、20kHz FOC 的参数，把 `K_smo`、边界层、LPF 和 `PLL Kp/Ki` 给你算成一套可以直接试的参数。**


------------
-----------
你的SMO+PLL无感FOC算法整体框架是正确的，但存在几个需要注意的问题：

## 主要问题

### 1. **SMO反电动势符号问题**
```c
/* 你的代码 */
smo->ebmf_alpha = low_pas_filter(0.45f, smo->ebmf_alpha, z_alpha);
```
这里**可能需要取反**：
```c
// 建议改为
smo->ebmf_alpha = low_pas_filter(0.45f, smo->ebmf_alpha, -z_alpha);
smo->ebmf_beta = low_pas_filter(0.45f, smo->ebmf_beta, -z_beta);
```

**原因**：滑模控制量 `z = K*sat(i_hat - i)` 的平均值等于反电动势的负值（取决于电流观测器方程的定义）。你的电流方程是：
```
di_hat/dt = -Rs/Ls * i_hat + 1/Ls * (u - z)
```
标准电机模型是：
```
di/dt = -Rs/Ls * i + 1/Ls * (u - e)
```
对比可知 `z ≈ e`（反电动势），所以滤波后应该直接是反电动势。但实际离散化后可能有符号差异，**建议通过实验验证**。

### 2. **PLL归一化误差计算**
```c
error = -Ed / E_mag;
```
这里用 `-Ed` 是正确的，但建议：
- 当反电动势很小时（低速），直接使用 `atan2f` 的结果作为角度估计
- PLL更适合中高速运行

### 3. **SMO增益可能不合适**
```c
smo->k_smo = 12.5f;
```
滑模增益需要满足：
```
k_smo > max(|e_alpha|, |e_beta|)
```
其中反电动势幅值 `E = ωe * ψf`。建议动态调整：
```c
// 根据转速动态调整
smo->k_smo = 1.5f * motor_speed_rad_s * flux_linkage;
```

### 4. **低通滤波器截止频率**
```c
low_pas_filter(0.45f, ...)  // 系数0.45
```
这个系数需要根据实际运行频率调整。建议：
- 截止频率设为电角频率的2-3倍
- 或使用自适应滤波器

### 5. **角度计算公式**
```c
smo->theta_e = -atan2f(smo->ebmf_alpha, smo->ebmf_beta);
```
这个负号和参数顺序取决于你的坐标定义。标准应该是：
```c
// 标准计算
smo->theta_e = atan2f(smo->ebmf_beta, smo->ebmf_alpha);
// 或者
smo->theta_e = -atan2f(smo->ebmf_alpha, smo->ebmf_beta);
```
需要根据你的Park/Clarke变换定义验证。

## 改进建议

### 1. **添加SMO稳定性保护**
```c
void SMO_Update(...)
{
    // ... 现有代码 ...
    
    // 添加反电动势幅值限制
    float e_mag = sqrtf(smo->ebmf_alpha * smo->ebmf_alpha + 
                        smo->ebmf_beta * smo->ebmf_beta);
    float e_max = 100.0f; // 根据电机参数设置
    if (e_mag > e_max) {
        smo->ebmf_alpha *= (e_max / e_mag);
        smo->ebmf_beta *= (e_max / e_mag);
    }
}
```

### 2. **PLL增加速度滤波**
```c
// PLL输出速度滤波
pll->we_filtered = pll->we_filtered * 0.9f + pll->we * 0.1f;
```

### 3. **低速切换策略**
```c
// 低速时使用直接atan2，高速时使用PLL
float e_mag = sqrtf(Ealpha*Ealpha + Ebeta*Ebeta);
if (e_mag < threshold) {
    // 低速：直接计算
    theta = atan2f(Ebeta, Ealpha);
} else {
    // 高速：使用PLL
    theta = PLL_Update(pll, Ealpha, Ebeta);
}
```

### 4. **添加启动策略**
- 开环强拖启动
- 切换到闭环的过渡
- 初始位置检测（如有需要）

## 调试建议

1. **验证反电动势方向**
   - 开环运行电机，观察SMO输出的Eα/Eβ
   - 确认旋转方向是否正确

2. **分步调试**
   - 先验证电流观测器
   - 再验证反电动势估计
   - 最后调试PLL

3. **参数整定顺序**
   - SMO增益：从小到大调整
   - LPF系数：根据转速范围调整
   - PLL的Kp/Ki：最后调整

总体而言，你的算法框架是正确的，主要需要在实际硬件上调试参数和验证符号方向。