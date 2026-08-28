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