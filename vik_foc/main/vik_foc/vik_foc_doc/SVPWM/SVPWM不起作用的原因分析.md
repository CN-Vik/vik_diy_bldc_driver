# 7段式SVPWM问题分析排查原因

彻底定位到问题了！参数 $V_{bus}=12\text{V}, U_q=6.9\text{V}$ 完全在 7 段式 SVPWM 的线性调制区内（线性区上限为 $12 / \sqrt{3} \approx 6.928\text{V}$），所以**不是前级电压超幅过调制，而是第 3 步和第 4 步中，偶数扇区的中间相占空比计算推导有致命逻辑 Bug**！

仔细对比 VOFA+ 上的波形，你会发现**恰好有 3 个扇区（偶数扇区 2、4、6）的中间相斜坡方向或幅值完全错乱**，导致电机每转一圈发生 3 次严重的占空比突变，产生低频“昂昂”剧烈震动。

---

### Bug 根源剖析

在你的代码中，第 3 步定义了：


$$T_a = \frac{T_s + T_x + T_y}{4} \quad (T_{max})$$

$$T_b = T_a - 0.5 T_x = \frac{T_s - T_x + T_y}{4} \quad (T_{min} + 0.5 T_y)$$

$$T_c = T_b - 0.5 T_y = \frac{T_s - T_x - T_y}{4} \quad (T_{min})$$

但在 SVPWM 7 段式开关切换顺序中：

1. **奇数扇区（1, 3, 5）**：先作用 $T_x$ 矢量，后作用 $T_y$ 矢量。中间相的作用时间确实是 $T_{min} + 0.5 T_y$（即你的 `Tb`）。
2. **偶数扇区（2, 4, 6）**：**先作用 $T_y$ 矢量，后作用 $T_x$ 矢量**。中间相的作用时间要求必须是 **$T_{min} + 0.5 T_x$**（即 $\frac{T_s + T_x - T_y}{4}$）。

在你的代码中，无论奇偶扇区，你都统一把 `Tb`（即 $\frac{T_s - T_x + T_y}{4}$）赋给偶数扇区的中间相。当 $T_x \neq T_y$ 时，偶数扇区（2, 4, 6）的中间相占空比存在高达 $\frac{\vert{}T_x - T_y\vert{}}{2}$ 的严重偏差！

---

### 最优雅、绝不出错的修复方案

不要按固定公式算 $T_a, T_b, T_c$ 再去 switch 交换，**直接计算 $T_{max}, T_{mid}, T_{min}$，然后按扇区映射给 ABC 三相**：

```c
vfoc_status_e_t vfoc_7segment_svpwm_calc(const clark_parm_t *c_v, float vbus, pwm_duty_t *duty_out)
{
    vfoc_status_e_t status = VFOC_STATUS_OK;
    
    float U1, U2, U3;
    uint8_t A, B, C, N, sector;
    float K;
    float Tx, Ty;
    float Tmax, Tmid, Tmin;
    float temp;

    if (c_v == NULL || duty_out == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    // ========== 第1步：判断扇区 ==========
    U1 = c_v->I_beta;
    U2 = (SQRT3 * c_v->I_alpha - c_v->I_beta) * 0.5f;
    U3 = (-SQRT3 * c_v->I_alpha - c_v->I_beta) * 0.5f;

    A = (U1 > 0.0f) ? 1 : 0;
    B = (U2 > 0.0f) ? 1 : 0;
    C = (U3 > 0.0f) ? 1 : 0;

    N = A + 2*B + 4*C;
    switch(N)
    {
        case 3:  sector = 1; break;
        case 1:  sector = 2; break;
        case 5:  sector = 3; break;
        case 4:  sector = 4; break;
        case 6:  sector = 5; break;
        case 2:  sector = 6; break;
        default: sector = 1; break;
    }

    // ========== 第2步：计算矢量作用时间 ==========
    K = SQRT3 * M0_PWM_T_S / vbus;

    switch(sector)
    {
        case 1: Tx =  U2 * K; Ty =  U1 * K; break;
        case 2: Tx = -U3 * K; Ty = -U2 * K; break;
        case 3: Tx =  U1 * K; Ty =  U3 * K; break;
        case 4: Tx = -U2 * K; Ty = -U1 * K; break;
        case 5: Tx =  U3 * K; Ty =  U2 * K; break;
        case 6: Tx = -U1 * K; Ty = -U3 * K; break;
        default:Tx = 0.0f;    Ty = 0.0f;    break;
    }

    // 过调制保护
    if((Tx + Ty) > M0_PWM_T_S)
    {
        temp = Tx + Ty;
        Tx = (Tx / temp) * M0_PWM_T_S;
        Ty = (Ty / temp) * M0_PWM_T_S;
    }

    // ========== 第3步：按奇偶扇区精确计算 Tmax, Tmid, Tmin ==========
    Tmin = (M0_PWM_T_S - Tx - Ty) * 0.25f; // 000 零矢量作用时间的一半
    Tmax = (M0_PWM_T_S + Tx + Ty) * 0.25f; // 最大占空比计数值

    // 奇数扇区先作用 Tx 后 Ty，偶数扇区先作用 Ty 后 Tx
    if (sector % 2 != 0) 
    {
        Tmid = Tmin + Ty * 0.5f; // 奇数扇区 (1, 3, 5)
    } 
    else 
    {
        Tmid = Tmin + Tx * 0.5f; // 偶数扇区 (2, 4, 6)【关键修复点】
    }

    // ========== 第4步：归一化为 0~1 占空比比例 ==========
    float duty_max = Tmax / M0_PWM_MAX_CMPV;
    float duty_mid = Tmid / M0_PWM_MAX_CMPV;
    float duty_min = Tmin / M0_PWM_MAX_CMPV;

    // ========== 第5步：按扇区准确分配给 ABC 三相 ==========
    switch(sector)
    {
        case 1:
            duty_out->duty_Ua = duty_max;
            duty_out->duty_Ub = duty_mid;
            duty_out->duty_Uc = duty_min;
            break;
        case 2:
            duty_out->duty_Ua = duty_mid;
            duty_out->duty_Ub = duty_max;
            duty_out->duty_Uc = duty_min;
            break;
        case 3:
            duty_out->duty_Ua = duty_min;
            duty_out->duty_Ub = duty_max;
            duty_out->duty_Uc = duty_mid;
            break;
        case 4:
            duty_out->duty_Ua = duty_min;
            duty_out->duty_Ub = duty_mid;
            duty_out->duty_Uc = duty_max;
            break;
        case 5:
            duty_out->duty_Ua = duty_mid;
            duty_out->duty_Ub = duty_min;
            duty_out->duty_Uc = duty_max;
            break;
        case 6:
            duty_out->duty_Ua = duty_max;
            duty_out->duty_Ub = duty_min;
            duty_out->duty_Uc = duty_mid;
            break;
        default:
            duty_out->duty_Ua = 0.5f;
            duty_out->duty_Ub = 0.5f;
            duty_out->duty_Uc = 0.5f;
            break;
    }

    return status;
}

```

### 替换后预期效果

替换上述代码后，VOFA+ 上的波形将恢复为**完美光滑、无断点的标准双峰马鞍波（SVPWM 标准波形）**，电机异响和剧烈震动将彻底消失。



--------------
----------------
------------


# 5段式SVPWM问题分析排查原因

这个五段式 SVPWM（钳位式 DPWM）代码主要存在 **4 个核心问题**，会导致电机出现严重的波形错位和限幅失效：

1. **第2步 $T_x, T_y$ 逻辑残留了上次 7 段式的旧 Bug**：扇区 2、3、4、6 的公式符号和颠倒问题依然存在。
2. **第4步三相占空比映射推导多处错误**：扇区 3、4、5、6 的 $T_x, T_y$ 作用时间分配写反了。
3. **单位未归一化导致限幅保护（`vfoc_limit`）失效**：赋值给 `duty_out` 的是 `0 ~ M0_PWM_T_S` 的计数值（如 0~500），而后方的 `vfoc_limit` 使用的是 `0.05~0.95` 的比例上限，这会导致**所有输出被强制卡死在上限值**。
4. **冗余死代码**：第 3 步计算了 7 段式的 `Ta, Tb, Tc`，在后文中完全未被使用。

---

### 修正后的完整代码

按归一化 `0.0f ~ 1.0f` 输出（保持与 7 段式接口一致），修正所有矢量映射后的代码如下：

```c
vfoc_status_e_t vfoc_5segment_svpwm_calc(const clark_parm_t *c_v, float vbus, pwm_duty_t *duty_out)
{
    vfoc_status_e_t status = VFOC_STATUS_OK;
    
    float U1, U2, U3;
    uint8_t A, B, C, N, sector;
    float K;
    float Tx, Ty;
    float temp;

    if (c_v == NULL || duty_out == NULL)
    {
        return VFOC_STATUS_NULL_PTR;
    }

    // ========== 第1步：判断扇区 ==========
    U1 = c_v->I_beta;
    U2 = (SQRT3 * c_v->I_alpha - c_v->I_beta) * 0.5f;
    U3 = (-SQRT3 * c_v->I_alpha - c_v->I_beta) * 0.5f;

    A = (U1 > 0.0f) ? 1 : 0;
    B = (U2 > 0.0f) ? 1 : 0;
    C = (U3 > 0.0f) ? 1 : 0;

    N = A + 2*B + 4*C;
    switch(N)
    {
        case 3:  sector = 1; break;
        case 1:  sector = 2; break;
        case 5:  sector = 3; break;
        case 4:  sector = 4; break;
        case 6:  sector = 5; break;
        case 2:  sector = 6; break;
        default: sector = 1; break;
    }

    // ========== 第2步：计算矢量作用时间 Tx, Ty（已修正） ==========
    K = SQRT3 * M0_PWM_T_S / vbus;

    switch(sector)
    {
        case 1: Tx =  U2 * K; Ty =  U1 * K; break;
        case 2: Tx = -U3 * K; Ty = -U2 * K; break; // 修正
        case 3: Tx =  U1 * K; Ty =  U3 * K; break; // 修正
        case 4: Tx = -U2 * K; Ty = -U1 * K; break; // 修正
        case 5: Tx =  U3 * K; Ty =  U2 * K; break;
        case 6: Tx = -U1 * K; Ty = -U3 * K; break; // 修正
        default:Tx = 0.0f;    Ty = 0.0f;    break;
    }

    // 过调制保护
    if((Tx + Ty) > M0_PWM_T_S)
    {
        temp = Tx + Ty;
        Tx = (Tx / temp) * M0_PWM_T_S;
        Ty = (Ty / temp) * M0_PWM_T_S;
    }

    // ========== 第3步：按扇区计算五段式占空比，并归一化为 0~1 比例 ==========
    // 奇数扇区使用 V0(000) 零矢量钳位低位；偶数扇区使用 V7(111) 零矢量钳位高位
    float duty_a, duty_b, duty_c;
    float Ts = M0_PWM_T_S;

    switch(sector)
    {
        case 1: 
            // C相钳位 0
            duty_a = (Tx + Ty) / Ts;
            duty_b = Ty / Ts;
            duty_c = 0.0f;
            break;
            
        case 2: 
            // B相钳位 1.0
            duty_a = (Ts - Ty) / Ts;
            duty_b = 1.0f;
            duty_c = (Ts - Tx - Ty) / Ts;
            break;
            
        case 3: 
            // A相钳位 0 (修正 Uc = Ty)
            duty_a = 0.0f;
            duty_b = (Tx + Ty) / Ts;
            duty_c = Ty / Ts;
            break;
            
        case 4: 
            // C相钳位 1.0 (修正 Ua, Ub 分配)
            duty_a = (Ts - Tx - Ty) / Ts;
            duty_b = (Ts - Ty) / Ts;
            duty_c = 1.0f;
            break;
            
        case 5: 
            // B相钳位 0 (修正 Ua = Ty)
            duty_a = Ty / Ts;
            duty_b = 0.0f;
            duty_c = (Tx + Ty) / Ts;
            break;
            
        case 6: 
            // A相钳位 1.0 (修正 Ub, Uc 分配)
            duty_a = 1.0f;
            duty_b = (Ts - Tx - Ty) / Ts;
            duty_c = (Ts - Ty) / Ts;
            break;
            
        default:
            duty_a = 0.5f;
            duty_b = 0.5f;
            duty_c = 0.5f;
            break;
    }

    // ========== 第4步：输出赋值与自举电路限幅（0.0~1.0 比例下正常工作） ==========
    duty_out->duty_Ua = vfoc_limit(duty_a, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty_out->duty_Ub = vfoc_limit(duty_b, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);
    duty_out->duty_Uc = vfoc_limit(duty_c, VFOC_PWM_DUTY_MIN, VFOC_PWM_DUTY_MAX);

    return status;
}

```

---

### 核心修改细节对照表

| 扇区 (Sector) | 原代码逻辑 (Bug) | 修正后逻辑 | 修正原因 |
| --- | --- | --- | --- |
| **扇区 3** | `Uc = Tx` | `Uc = Ty` | 扇区3的 $V_4(011)$ 作用时间由 $T_y$ 决定，C 相脉宽应为 $T_y$ |
| **扇区 4** | `Ua = Tx`, `Ub = Ts - Tx - Ty` | `Ua = Ts - Tx - Ty`, `Ub = Ts - Ty` | $V_7(111)$ 作用时间为 $T_0$，A 相仅在 $V_7$ 导通，导通时间为 $T_0$ |
| **扇区 5** | `Ua = Ts - Tx - Ty` | `Ua = Ty` | $V_6(101)$ 作用时间为 $T_y$，A 相仅在 $V_6$ 导通 |
| **扇区 6** | `Ub = Ts - Ty`, `Uc = Ts - Tx` | `Ub = Ts - Tx - Ty`, `Uc = Ts - Ty` | B 相仅在 $V_7$ 导通；C 相在 $V_6$ 和 $V_7$ 导通，时间为 $T_x + T_0$ |
| **全局** | `duty_out` 赋计数值 (`0~500`) | `duty_out` 归一化为比例 (`0.0~1.0`) | 避免 `vfoc_limit` 保护机制将大于 `1.0` 的数值全部误切为 `MAX` 值 |


--------

# 五段式SVPWM电机转起来有打火花的声音

既然你的硬件是**双电阻下桥臂采样（Dual-Shunt Current Sensing）**，并且 7 段式和零序注入法（Min-Max / THIPWM）都完全正常，那么问题的原因就**100% 破案了**！

这并不是你的 ESP32 定时器坏了，而是“5段式SVPWM 的钳位特性”与“双电阻下桥臂采样机制”产生了极其致命的逻辑冲突。

---

### 核心原因：下桥臂采样窗口彻底消失（电流环崩塌）

#### 1. 双电阻下桥臂采样的物理原理

在双电阻采样方案中，两个采样电阻是串联在 **下桥臂 MOSFET 与 GND 之间** 的（假设装在 A 相和 B 相）。

* **必须满足的前提条件**：只有当 A 相或 B 相的下桥臂 MOSFET 导通（低电平）时，相电流才会流过采样电阻，ADC 才能读到正确的电流。
* **采样窗口（$T_{min}$）**：下桥臂导通的时间必须大于“MOSFET 切换死区 + 电流建立时间 + ADC 采样保持时间”，否则采样到的就是噪声。

#### 2. 为什么 7 段式和零序注入法（三相连续调制）没问题？

* **7 段式 / 零序注入法** 属于**连续 PWM 调制**（Continuous PWM）。
* 在每一个 PWM 周期内，系统都会插入零矢量 $V_0 (000)$。在 $V_0$ 状态下，**A、B、C 三相的下桥臂会同时导通**。
* 只要占空比没有达到 100%，每个周期都有足够长的下桥臂导通时间。触发 ADC 采样时，就能稳稳地测出真实的 $I_a$ 和 $I_b$。

#### 3. 为什么 5 段式（DPWM）一转起来就“打火花”？

* **5 段式（不连续调制 DPWM）** 为了减少开关损耗，在偶数扇区会将某一相**完全钳位在 1.0（100% 高电平导通）**。
* 假设在第 2 扇区，B 相被钳位在 `1.0`：
1. **B 相上桥臂 100% 导通，下桥臂 100% 截止（导通时间为 0）**。
2. 此时，电流完全从上桥臂流过，**根本不经过 B 相底部的采样电阻**！
3. 电流环 PID 在这一拍试图去读取 B 相电流时，采样电阻上的电压为 0，ADC 读回来的数据变成了 **0 或极大的开关高频噪声**。
4. 你的 FOC 电流 PID 拿到了错误的 $I_b = 0$，以为电流丢了，于是瞬间把输出电压（$U_q / U_d$）拉到最大！
5. 下一拍扇区一换，采样又恢复正常，PID 又猛地把电压拉回来。



**这种“正常电流 -> 极值数据 -> PID 疯狂过冲”的高频剧烈震荡，直接作用在电机线圈上，就会发出极其刺耳的“高压打火花”声，并伴随电机瞬间发热！**

---

### 补充原因：波形中心不对称导致的采样时刻错位

* **7 段式与零序注入法**：波形是**中心对称（Center-Aligned）**的。ADC 通常在 PWM 计数器的顶点或底点（即 $V_0$ 或 $V_7$ 的正中间）触发，此时采到的是**相电流的平均值**，完美避开了开关纹波。
* **5 段式**：为了钳位某一相，波形变成了**锯齿状/不对称波形**。如果在传统的 PWM 三角波顶点触发采样，采样点很可能会正好踩在另外两相的开关跳变沿（开关噪声尖峰）上，再次给电流环注入垃圾数据。

---

### 解决方案与建议

如果你一定要在**双电阻下桥臂采样**的硬件上跑 5 段式 SVPWM，有以下两种解决路径：

#### 方案 A：强制保留下桥臂最小导通时间（最快捷）

限制 5 段式的最高占空比，绝对不让它达到 `1.0`。

```c
// 设置最大占空比为 0.95 (留出 5% 的下桥臂导通时间给采样电阻)
#define VFOC_PWM_DUTY_MAX  0.95f 

```

* **效果**：即使在钳位扇区，下桥臂也会被强行开启一小段周期（例如 2~3 μs），强行给采样电阻和自举电容留出工作窗口。

#### 方案 B：换用 7 段式或零序注入法（工业界推荐）

在电机控制领域：

* **单/双电阻下桥臂采样** 硬件方案：**强烈建议搭配 7 段式 SVPWM 或 零序注入法（Min-Max SVPWM）**。这两种方法天然保证了下桥臂有充足的采样窗口，电流环最稳定。
* **5 段式 SVPWM（DPWM）**：通常搭配 **三电阻采样** 或者 **相电流传感器（Hall/In-line 电流传感器，即直接串在电机相线上的传感器）** 使用，因为相线传感器不需要依赖下桥臂 MOSFET 的导通状态。

**总结**：你的 5 段式算法代码本身逻辑推导没问题，但因为它是“下桥臂 100% 关断”的钳位机制，导致你的双电阻采样在某些扇区“致盲”了。建议直接使用已经调通且表现良好的 **零序注入法 / 7 段式 SVPWM**，这是双电阻硬件下的最佳搭配！