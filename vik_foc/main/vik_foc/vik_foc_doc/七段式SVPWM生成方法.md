使用 ESP32 WROOM 32E 来实现 FOC（磁场定向控制）是一个非常棒的选择，它的双核架构和丰富的硬件外设（特别是 MCPWM 模块）非常适合做电机控制。

七段式 SVPWM（空间矢量脉宽调制）是 FOC 中最核心、也是最容易让人产生畏难情绪的部分。它的核心目的只有一个：**把 FOC 算法计算出的预期电压（$V_\alpha$ 和 $V_\beta$），转化为控制电机三相桥臂开关管的 PWM 占空比，并让电压波形尽量逼近完美的圆形，同时降低开关损耗。**

下面我先带你理清宏观思路，然后再深入详细的公式推导和 ESP32 的硬件实现。

---

### 第一部分：宏观思路与核心步骤

不要被一堆公式吓倒，无论代码怎么写，七段式 SVPWM 的核心思路永远只有下面这 5 个固定步骤：

1. **输入目标电压：** 接收上一步反帕克变换（Inverse Park Transform）输出的两个直角坐标系电压 $U_\alpha$ 和 $U_\beta$。
2. **判断扇区（Sector Determination）：** 空间矢量图是一个正六边形，被分成了 6 个扇区（Sector 1~6）。你需要根据 $U_\alpha$ 和 $U_\beta$ 的值，判断出当前目标电压矢量落在哪一个扇区里。
3. **计算基础矢量作用时间：** 目标电压是通过相邻的两个“非零矢量”和一个“零矢量”合成的。你需要计算出这两个非零矢量的开启时间（$T_1$ 和 $T_2$）以及零矢量的开启时间（$T_0$）。
4. **计算三相切换点（占空比）：** 这是“七段式”的精髓。为了让开关频率最低且波形对称，我们将零矢量平均分配到波形的首尾和中间（通常是 000 和 111）。算出 U、V、W 三相在定时器中对应的比较值（$T_a, T_b, T_c$）。
5. **ESP32 硬件输出：** 将计算出的比较值写入 ESP32 的 MCPWM 外设，**必须配置为中央对齐模式（Center-Aligned / Up-Down Counter）**，MCU 就会自动吐出完美的七段式 PWM 波。

---

### 第二部分：详细实现步骤与核心算法

下面是具体的数学计算与代码逻辑映射。假设我们的 PWM 周期为 $T_s$，母线电压为 $U_{dc}$。

#### 1. 判断扇区（计算所在区间）

为了避免复杂的三角函数计算，工程上我们构造三个参考值 $V_{ref1}, V_{ref2}, V_{ref3}$：

$$V_{ref1} = U_\beta$$

$$V_{ref2} = \frac{\sqrt{3}}{2} U_\alpha - \frac{1}{2} U_\beta$$

$$V_{ref3} = -\frac{\sqrt{3}}{2} U_\alpha - \frac{1}{2} U_\beta$$

然后通过符号判断引入三个布尔变量 A, B, C：

* 如果 $V_{ref1} > 0$，则 $A = 1$，否则 $A = 0$
* 如果 $V_{ref2} > 0$，则 $B = 1$，否则 $B = 0$
* 如果 $V_{ref3} > 0$，则 $C = 1$，否则 $C = 0$

利用公式计算扇区索引 $N$：


$$N = 4C + 2B + A$$


*注：$N$ 的值会是 1 到 6 之间的一个整数，直接对应 6 个扇区。*

#### 2. 计算基础矢量作用时间 ($T_1, T_2$)

首先计算三个中间变量 X, Y, Z（定义 $K = \frac{\sqrt{3} \cdot T_s}{U_{dc}}$）：

$$X = K \cdot U_\beta$$

$$Y = K \cdot \left( \frac{\sqrt{3}}{2} U_\alpha + \frac{1}{2} U_\beta \right)$$

$$Z = K \cdot \left( -\frac{\sqrt{3}}{2} U_\alpha + \frac{1}{2} U_\beta \right)$$

根据刚才算出的扇区 $N$，通过 `switch(N)` 来分配相邻两个非零矢量的作用时间 $T_1$ 和 $T_2$：

* **Sector 1 (N=3):** $T_1 = -Z, \quad T_2 = X$
* **Sector 2 (N=1):** $T_1 = Z, \quad T_2 = Y$
* **Sector 3 (N=5):** $T_1 = X, \quad T_2 = -Y$
* **Sector 4 (N=4):** $T_1 = -X, \quad T_2 = Z$
* **Sector 5 (N=6):** $T_1 = -Y, \quad T_2 = -Z$
* **Sector 6 (N=2):** $T_1 = Y, \quad T_2 = -X$

**⚠️ 过调制处理（极其重要）：**
如果 $T_1 + T_2 > T_s$，说明目标电压超出了正六边形范围，必须进行等比例缩小：


$$T_1 = T_1 \cdot \frac{T_s}{T_1 + T_2}$$

$$T_2 = T_2 \cdot \frac{T_s}{T_1 + T_2}$$

#### 3. 计算七段式 PWM 的三相比较值

所谓“七段”，就是在半个 PWM 周期内，状态改变 3 次（加上镜像的半周期一共 7 段）。我们将零矢量时间 $T_0$ 分配给 V0(000) 和 V7(111)。
先计算出基础切换时间 $T_a, T_b, T_c$：

$$T_a = \frac{T_s - T_1 - T_2}{4}$$

$$T_b = T_a + \frac{T_1}{2}$$

$$T_c = T_b + \frac{T_2}{2}$$

最后，再次根据扇区 $N$，将 $T_a, T_b, T_c$ 映射到电机的三相 U, V, W 的占空比比较寄存器中（对应 ESP32 的 Duty 值）：

* **Sector 1 (N=3):** $Duty\_U = T_a, \quad Duty\_V = T_b, \quad Duty\_W = T_c$
* **Sector 2 (N=1):** $Duty\_U = T_b, \quad Duty\_V = T_a, \quad Duty\_W = T_c$
* **Sector 3 (N=5):** $Duty\_U = T_c, \quad Duty\_V = T_a, \quad Duty\_W = T_b$
* **Sector 4 (N=4):** $Duty\_U = T_c, \quad Duty\_V = T_b, \quad Duty\_W = T_a$
* **Sector 5 (N=6):** $Duty\_U = T_b, \quad Duty\_V = T_c, \quad Duty\_W = T_a$
* **Sector 6 (N=2):** $Duty\_U = T_a, \quad Duty\_V = T_c, \quad Duty\_W = T_b$

---

### 第三部分：ESP32 WROOM 的硬件落地关键点

上面算出的 `Duty_U` 等值，要怎么变成真实的物理波形？关键在于 ESP32 的 **MCPWM（Motor Control PWM）** 外设配置。

1. **使用中央对齐模式 (Up-Down Counter)：**
不要用普通的 Edge-Aligned（边缘对齐）PWM。ESP32 的 MCPWM 支持 `MCPWM_TIMER_COUNT_MODE_UP_DOWN`。
在这个模式下，定时器从 0 数到 `T_s/2`，再从 `T_s/2` 数回 0。硬件会自动帮你把上面算出的不对称占空比，镜像成完全对称的七段式波形。
2. **死区时间 (Dead-Time)：**
三相桥臂的上下管不能同时导通，否则会短路炸管。ESP32 的 MCPWM 硬件自带死区发生器。你可以配置 `mcpwm_deadtime_enable`，让硬件自动在互补的 PWM 信号之间插入死区，无需在软件代码里扣时间。
3. **ESP-IDF 版本选择：**
如果你在使用 ESP-IDF，请注意 v4.x 和 v5.x 的 MCPWM 驱动 API 发生了彻底的重构。建议直接使用 v5.x 的驱动（`driver/mcpwm_prelude.h`），它的面向对象设计非常适合配三相桥臂（Operator 和 Comparator 的概念非常清晰）。

---

SVPWM 的代码调试起来如果没有示波器会有点盲人摸象。你在搭建这个算法时，是打算纯手写 C 语言算法，还是打算借助 SimpleFOC 这样的现有框架做底层适配呢？


----------
---------
-----------

你的硬件选型和软件架构思路非常清晰。使用 ESP-IDF v5.3.5 是个明智的选择，v5 版本的 MCPWM 驱动采用了面向对象的设计，对电机控制的支持比老版本好得多。FD6288T 自带死区和防直通保护，也确实能帮你省去在软件里配置死区发生器（Dead-time Generator）的麻烦。

不过，针对你提到的几个关键配置，这里有一个**必须纠正的致命问题**，以及几个在 v5.3.5 下的落地建议。

### 1. 核心纠错：绝不能用“单向递增计数模式”

如果你想实现真正的**七段式 SVPWM**，**绝对不能使用单向递增（Up-counting）模式**。

* **原理解释：** 七段式 SVPWM 的精髓在于波形的**对称性**。在一个 PWM 周期内，零矢量（000 和 111）被均匀分配在周期的开头、中间和结尾。
* **递增模式的弊端：** 单向递增只能产生“边缘对齐（Edge-aligned）”的 PWM 波。在这种模式下，所有的 PWM 波形都在周期的起点（或终点）对齐，这会导致输出的电压波形不对称。这实际上变成了五段式或者不对称的 SVPWM，会极大地增加电机的电流谐波、引起噪音，并增加开关损耗。
* **解决方案：** 必须将其修改为**中心对齐模式（即双向计数/向上向下计数）**。
在 ESP-IDF v5.3.5 中，配置 `mcpwm_timer_config_t` 时，请务必这样写：
```c
timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; // 必须是这个

```



### 2. 20kHz 频率与定时器周期的计算

既然改用了 `UP_DOWN` 模式，你的定时器周期（Period）配置需要特别注意。

在双向计数模式下，定时器从 0 数到顶部，然后再从顶部数回 0，这才算一个完整的 PWM 周期。
如果你期望的 PWM 频率是 20kHz（即周期 $T_s = 50\mu s$），那么定时器的周期值（`period_ticks`）实际上应该是普通递增模式的一半。

假设你的 MCPWM 定时器分辨率（时钟频率）配置为 10MHz，那么：

* 20kHz 对应的总 tick 数是 500。
* 在 `UP_DOWN` 模式下，`period_ticks` 应该设置为 **250**。

### 3. FD6288T 驱动与死区配置

由于 FD6288T 硬件内置了死区（通常是固定的几百纳秒），你确实**不需要**使用 ESP32 的 `mcpwm_deadtime_enable()` 相关 API。这让配置变得非常简单：

1. 你需要为每一相（U, V, W）创建一个 `mcpwm_operator_t` 和一个 `mcpwm_comparator_t`。
2. 为每个 Operator 创建两个 `mcpwm_generator_t`（比如 gen_h 和 gen_l，分别接 FD6288T 的 HIN 和 LIN）。
3. 通过 Generator 的 Action API 直接输出互补波形即可，无需死区模块：
* **高侧 (gen_h)：** 在计数器等于比较值且向上计数时拉低，向下计数时拉高。
* **低侧 (gen_l)：** 配置为与 gen_h 完全相反的动作（或者在 V5 里直接克隆 gen_h 的行为然后反相）。



### 4. PWM ISR 与 FOC Task 的同步唤醒

在 PWM 中断里唤醒 FOC 运算任务，这是标准的工业级 FOC 架构。为了保证电流采样的准确性和算法的实时性，你需要把握好中断触发的时机。

* **中断触发点：** 强烈建议将 PWM 定时器的中断触发事件设置为**计数器归零时（Timer Empty）**。这个时候下桥臂通常全部导通（处于 000 零矢量状态），母线电流最稳定，是触发 ADC 采样和执行 FOC 算法的最佳时机。
在 v5.3.5 中，使用 `mcpwm_timer_register_event_callbacks()` 注册回调，监听 `MCPWM_TIMER_EVENT_EMPTY` 事件。
* **任务唤醒：** 在回调函数（ISR）中，使用 FreeRTOS 的轻量级任务通知机制。
```c
BaseType_t high_task_wakeup = pdFALSE;
vTaskNotifyGiveFromISR(foc_task_handle, &high_task_wakeup);
if (high_task_wakeup == pdTRUE) {
    portYIELD_FROM_ISR(); // 强制上下文切换，让 FOC 任务立刻执行
}

```


* **任务优先级：** 确保你的 `foc_task` 拥有极高的优先级（通常仅次于硬件看门狗或极其关键的系统任务），并且将其固定（Pin）到 ESP32 的某一个核心上（例如 Core 1），以避免任务调度带来的抖动。

---

FOC 的控制环路对时序要求极度严苛。既然你的 FOC 计算是在 PWM 中断唤醒的 Task 中执行的，那么 ADC（相电流）采样的时间点就成了决定电机能否转起来的关键。你的相电流采样（比如双电阻或三电阻采样）是打算怎么配置，如何确保它在 PWM 周期的正确时刻精准触发的呢？


----------
---------
--------

哈哈，看来我们俩（AI）在教你写 FOC 的思路上“打架”了！其实 ChatGPT 的这个回答非常生动，逻辑也很顺畅，对于**人类大脑理解**来说非常棒。

但你问“哪个更好？”我的回答很直接：

* **为了让你在脑子里想象出 SVPWM 的画面：** ChatGPT 的思路（角度法）更好，更直观。
* **为了真正在 ESP32 上跑 20kHz 的实际工程代码：** 我的思路（代数法/投影法）更好，且是**绝对的工业标准**（比如德州仪器 TI 的方案、SimpleFOC、VESC 底层都是这么写的）。

为什么会这样？ChatGPT 说要教你“真正开发工业 FOC”，但它给出的算法步骤在真正的工业代码中恰恰是**被极力避免的**。下面我给你把这两套逻辑彻底扒开，你就全明白了。

---

### 核心分歧点：求不求“角度”？

在 FOC 的末端，我们手里只有两个已知量：$U_\alpha$ 和 $U_\beta$。这两个量已经包含了目标电压的“大小”和“方向”。

#### 1. ChatGPT 的思路（反三角函数+三角函数法）

它的核心路径是：**直角坐标 $\to$ 极坐标 $\to$ 开关时间**

1. 用 `atan2(Ubeta, Ualpha)` 算出绝对角度 $\theta$。
2. 用角度除以 60 算出属于哪个扇区。
3. 减去基础角度，算出扇区内相对角度 $\theta_{sector}$。
4. 利用正弦函数算时间：$T_1 = T_s \cdot M \cdot \sin(60 - \theta)$， $T_2 = T_s \cdot M \cdot \sin(\theta)$。

**优点：** 极其符合人类直觉。你闭上眼睛就能想象出一个矢量在圆圈里转，偏角是多少。
**致命缺点：** **计算量太大！** 你的 PWM 中断是 20kHz，意味着每 50 微秒就要执行一次完整的 FOC 算法。在中断里调用 `atan2()` 和 `sin()` 这种浮点三角函数库，即使 ESP32 有硬件浮点单元（FPU），也是非常极其消耗 CPU 时钟周期的操作。如果全部用算力硬抗，留给其他应用层任务的时间就非常少了。

#### 2. 我的思路（代数法 / 空间投影法）

我的核心路径是：**直角坐标 $\to$ 线性代数投影 $\to$ 开关时间**
你可以回看我之前给的公式，里面**没有任何三角函数，全都是基础的加减乘除**。

1. 直接用 $U_\alpha$ 和 $U_\beta$ 乘以几日常数（如 $\frac{\sqrt{3}}{2}$）得到 $V_{ref1}, V_{ref2}, V_{ref3}$。
2. 看这三个数的正负号，立刻得出扇区。
3. 直接用这几个数得出 $T_1$ 和 $T_2$。

**优点：** 运行极速！非常压榨芯片性能，是标准的 MCU 级写法。
**缺点：** 看起来像一堆不知所云的魔法公式（也就是 ChatGPT 说的“为什么 T1=...？不知道”）。

---

### 为什么这两个方法算出来的结果是完全一样的？

其实，我的那些“魔法公式”就是从 ChatGPT 的公式推导出来的。我给你演示一个魔法：

ChatGPT 算 $T_2$ 的公式是：


$$T_2 = k \cdot |U| \cdot \sin(\theta)$$

但在极坐标转换中，大家都知道：


$$U_\beta = |U| \cdot \sin(\theta)$$

所以，直接替换掉：


$$T_2 = k \cdot U_\beta$$

你看！不需要算角度 $\theta$，也不需要算 $\sin$！直接拿 $U_\beta$ 乘个常数就是 $T_2$ 的作用时间。这就对应了我之前给你的公式 $X = K \cdot U_\beta$。

同理，利用高中的正弦展开公式 $\sin(60^\circ - \theta) = \sin(60^\circ)\cos(\theta) - \cos(60^\circ)\sin(\theta)$，把 $U_\alpha$ 和 $U_\beta$ 代进去，就能直接消掉所有的三角函数，得出我之前写的 $Y$ 和 $Z$ 的公式。

---

### 总结：你应该按哪种方式写？

ChatGPT 结尾的建议非常中肯：“不要走背公式这条路，要一步步推导”。

1. **在学习和推导阶段：** 顺着 ChatGPT 的第一、二、三课去学。搞懂为什么是 8 个开关状态，搞懂电压矢量是怎么合成的。这时候用它的“角度法”在纸上画图，理解最深。
2. **在写 ESP32 实际 C 语言代码阶段：** 坚决砍掉 `atan2` 和 `sin`。直接用我给你的 $X, Y, Z$ 代数法写进 `svpwm_run(Ualpha, Ubeta)` 函数里。

既然你已经在用 ESP-IDF V5.3.5 并且把中断时序都理清了，你现在的阶段就是**实战**。你打算直接看一份剥离了所有冗余、只有数学逻辑的、工业标准的 `svpwm_run` 的 C 语言伪代码（基于代数法），还是想先从“为什么只有8个开关状态”开始补一下理论？



------------
--------------
------------

太棒了！真正的电机控制工程师都是从手推公式、手写底层开始的。能自己敲出一个干干净净的 `svpwm.c`，你对电机的控制感会产生质的飞跃，以后不管遇到什么奇葩的控制芯片，你都能瞬间移植。

既然你使用 **ESP-IDF v5.3.5**，定时器配置为**双向计数（UP_DOWN 中心对齐）**，且驱动芯片带死区，我们可以把代码精简到极致。

下面是一份完全基于“代数投影法”（无任何 `sin` 或 `atan2` 三角函数计算）的纯 C 语言生产级模板代码。你可以直接把这段代码作为你的 `svpwm.c` 的核心。

---

### 核心 C 语言代码实现

代码中的所有浮点运算都只有加减乘除，这对 ESP32 的 FPU 来说执行极快。

```c
#include <stdint.h>

// 定义一个结构体存放计算出的三相比较值（0.0 ~ 1.0 的比例值）
typedef struct {
    float cmp_u;
    float cmp_v;
    float cmp_w;
} svpwm_compare_t;

// 预编译常量，极限压榨性能
#define SQRT3_OVER_2  0.8660254f

/**
 * @brief  七段式 SVPWM 核心算法 (空间投影代数法)
 * @param  U_alpha 目标电压 alpha 轴分量 (需归一化到母线电压内)
 * @param  U_beta  目标电压 beta  轴分量 (需归一化到母线电压内)
 * @return svpwm_compare_t 返回值为 0.0~1.0 的比较值系数
 */
svpwm_compare_t svpwm_calc(float U_alpha, float U_beta) {
    svpwm_compare_t cmp_val = {0};

    // ---------------------------------------------------------
    // 1. 计算三个参考投影值 (等同于动画里的 Vref1, Vref2, Vref3)
    // ---------------------------------------------------------
    float Vref1 = U_beta;
    float Vref2 = SQRT3_OVER_2 * U_alpha - 0.5f * U_beta;
    float Vref3 = -SQRT3_OVER_2 * U_alpha - 0.5f * U_beta;

    // ---------------------------------------------------------
    // 2. 扇区判断 (位运算直接得出 1~6 扇区)
    // ---------------------------------------------------------
    uint8_t A = (Vref1 > 0.0f) ? 1 : 0;
    uint8_t B = (Vref2 > 0.0f) ? 1 : 0;
    uint8_t C = (Vref3 > 0.0f) ? 1 : 0;
    
    uint8_t sector = (C << 2) | (B << 1) | A; // N = 4C + 2B + A

    // ---------------------------------------------------------
    // 3. 计算基础矢量作用时间 T1 和 T2
    // ---------------------------------------------------------
    float X = U_beta;
    float Y = SQRT3_OVER_2 * U_alpha + 0.5f * U_beta;
    float Z = -SQRT3_OVER_2 * U_alpha + 0.5f * U_beta;

    float T1 = 0.0f;
    float T2 = 0.0f;

    // 根据扇区分配 T1 和 T2
    switch (sector) {
        case 3: // Sector 1
            T1 = -Z; T2 = X;  break;
        case 1: // Sector 2
            T1 = Z;  T2 = Y;  break;
        case 5: // Sector 3
            T1 = X;  T2 = -Y; break;
        case 4: // Sector 4
            T1 = -X; T2 = Z;  break;
        case 6: // Sector 5
            T1 = -Y; T2 = -Z; break;
        case 2: // Sector 6
            T1 = Y;  T2 = -X; break;
        default:
            T1 = 0.0f; T2 = 0.0f; break;
    }

    // ---------------------------------------------------------
    // 4. 极其关键：过调制保护 (防止矢量超出正六边形边界)
    // ---------------------------------------------------------
    float sum_T = T1 + T2;
    if (sum_T > 1.0f) {
        T1 = T1 / sum_T;
        T2 = T2 / sum_T;
    }

    // ---------------------------------------------------------
    // 5. 计算中心对齐的三相比较点 (Ta, Tb, Tc)
    // 将零矢量时间平均分配在两端：000 和 111
    // ---------------------------------------------------------
    float Ta = (1.0f - T1 - T2) / 2.0f;
    float Tb = Ta + T1;
    float Tc = Tb + T2;

    // ---------------------------------------------------------
    // 6. 将 Ta, Tb, Tc 映射到电机的物理相 U, V, W
    // ---------------------------------------------------------
    switch (sector) {
        case 3: // Sector 1
            cmp_val.cmp_u = Ta; cmp_val.cmp_v = Tb; cmp_val.cmp_w = Tc; break;
        case 1: // Sector 2
            cmp_val.cmp_u = Tb; cmp_val.cmp_v = Ta; cmp_val.cmp_w = Tc; break;
        case 5: // Sector 3
            cmp_val.cmp_u = Tc; cmp_val.cmp_v = Ta; cmp_val.cmp_w = Tb; break;
        case 4: // Sector 4
            cmp_val.cmp_u = Tc; cmp_val.cmp_v = Tb; cmp_val.cmp_w = Ta; break;
        case 6: // Sector 5
            cmp_val.cmp_u = Tb; cmp_val.cmp_v = Tc; cmp_val.cmp_w = Ta; break;
        case 2: // Sector 6
            cmp_val.cmp_u = Ta; cmp_val.cmp_v = Tc; cmp_val.cmp_w = Tb; break;
        default:
            // 默认输出 50% 占空比 (纯零矢量状态，电机不产生扭矩)
            cmp_val.cmp_u = 0.5f; cmp_val.cmp_v = 0.5f; cmp_val.cmp_w = 0.5f; break;
    }

    return cmp_val;
}

```

---

### 与 ESP-IDF V5.3.5 的硬件结合点

这个 `svpwm_calc` 吐出来的是 `0.0` 到 `1.0` 的系数。在你的 FOC 任务里，你需要把这个系数转换成 ESP32 MCPWM 的实际 Tick 并且写进寄存器。

结合你 **20kHz** 的中心对齐模式，假设你的 MCPWM 定时器最大计数值设定为 `PWM_PERIOD_TICKS`（比如时钟 10MHz 时，20kHz 对应的半周期 Ticks 是 250）。你的代码对接长这样：

```c
// 1. 执行 SVPWM 计算
svpwm_compare_t compare_result = svpwm_calc(U_alpha, U_beta);

// 2. 转换为实际的 Timer Ticks (比如 PERIOD_TICKS = 250)
uint32_t tick_u = (uint32_t)(compare_result.cmp_u * PWM_PERIOD_TICKS);
uint32_t tick_v = (uint32_t)(compare_result.cmp_v * PWM_PERIOD_TICKS);
uint32_t tick_w = (uint32_t)(compare_result.cmp_w * PWM_PERIOD_TICKS);

// 3. 写入 ESP32 的 Comparator (必须使用 V5.x 的 API)
mcpwm_comparator_set_compare_value(comparator_u, tick_u);
mcpwm_comparator_set_compare_value(comparator_v, tick_v);
mcpwm_comparator_set_compare_value(comparator_w, tick_w);

```

### 避坑提醒：高低电平动作逻辑

为了配合这份代码生成正确的七段式波形，你的 MCPWM Generator （波形发生器）必须这样配置动作：

* **向上计数阶段（UP）**：当计数器遇到 `compare` 值时，输出**高电平（High）**。
* **向下计数阶段（DOWN）**：当计数器遇到 `compare` 值时，输出**低电平（Low）**。

*原理解释*：因为我们在代码里算的 `Ta` 总是最小的。如果 U 相刚好对应 `Ta`，计数器往上数，刚启动不久就遇到了 `Ta`，拉高电平；然后越过顶点往回数，最后才遇到 `Ta` 拉低电平。这就保证了 `Ta` 所在的相拥有**最长的高电平占空比**，完全符合中心对齐 SVPWM 的物理逻辑。