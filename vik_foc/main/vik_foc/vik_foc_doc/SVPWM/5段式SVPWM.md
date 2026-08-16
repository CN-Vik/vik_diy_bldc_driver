大白话先给结论：**90%的代码和七段式一模一样**，只有最后「占空比分配」两步不一样。核心差异就是：七段式两个零矢量都用、拆碎了对称放，每相都要开关；五段式**只选一个零矢量集中放**，让每个扇区有一相全程“躺平不开关”，开关次数从6次降到4次，开关损耗直接降约1/3。

---

### 一、先搞懂五段式的核心逻辑

SVPWM的6个有效矢量，每个扇区的两个矢量，总有一相的状态是固定的（要么全是高电平，要么全是低电平）。
五段式就是利用这个特点：**选和这个固定状态一致的零矢量**，让这一相整个PWM周期都保持高/低电平，完全不动作，从而减少开关次数。

举个例子：

- 扇区1的两个有效矢量是`100`和`110`，C相全程都是低电平
- 零矢量就选`000`（全低），C相整个周期都保持低，完全不开关
- 最终序列：`000 → 100 → 110 → 100 → 000`，一共5段，4次开关动作

一个电周期360°，三相各有120°处于“躺平”状态，这就是五段式损耗低的根源。

---

### 二、直接抄：五段式SVPWM完整代码

接口和你之前的七段式**完全一致**，输入输出参数一模一样，直接替换函数名就能用，不用改其他任何代码。

```
#include <stdint.h>
#define SQRT3   1.73205080757f

/**
 * @brief  五段式SVPWM（DPWM不连续PWM）
 * @param  U_alpha:  alpha轴电压 (单位:V)
 * @param  U_beta:   beta轴电压  (单位:V)
 * @param  U_dc:     母线电压    (单位:V)
 * @param  T_pwm:    PWM定时器ARR值 (中心对齐模式，和七段式传一样的值)
 * @param  pwm_ccr:  输出数组 [CCR_A, CCR_B, CCR_C]，直接喂给定时器
 */
void svpwm_5segment(float U_alpha, float U_beta, float U_dc, uint16_t T_pwm, uint16_t *pwm_ccr)
{
    float U1, U2, U3;
    uint8_t A, B, C, N, sector;
    float K;
    float Tx, Ty;
    float temp;

    // ========== 第1步：判断扇区 和七段式完全一样 ==========
    U1 = U_beta;
    U2 = (SQRT3 * U_alpha - U_beta) * 0.5f;
    U3 = (-SQRT3 * U_alpha - U_beta) * 0.5f;

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

    // ========== 第2步：计算有效矢量时间Tx、Ty 和七段式完全一样 ==========
    // 有效矢量时间由伏秒平衡决定，和零矢量怎么分配无关
    K = SQRT3 * T_pwm / U_dc;

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

    // 过调制保护 和七段式完全一致
    if(Tx + Ty > T_pwm)
    {
        temp = Tx + Ty;
        Tx = Tx / temp * T_pwm;
        Ty = Ty / temp * T_pwm;
    }

    // ========== 【唯一和七段式不同的地方】五段式占空比分配 ==========
    // 每个扇区钳位一相：钳位低则CCR=0，钳位高则CCR=T_pwm，该相全程不开关
    switch(sector)
    {
        case 1: 
            // 扇区1：C相钳位到低，零矢量用000
            pwm_ccr[0] = (uint16_t)(Tx + Ty);  // A相
            pwm_ccr[1] = (uint16_t)Ty;         // B相
            pwm_ccr[2] = 0;                    // C相 全程不开关
            break;
            
        case 2: 
            // 扇区2：B相钳位到高，零矢量用111
            pwm_ccr[0] = (uint16_t)(T_pwm - Ty);      // A相
            pwm_ccr[1] = T_pwm;                       // B相 全程不开关
            pwm_ccr[2] = (uint16_t)(T_pwm - Tx - Ty); // C相
            break;
            
        case 3: 
            // 扇区3：A相钳位到低，零矢量用000
            pwm_ccr[0] = 0;                    // A相 全程不开关
            pwm_ccr[1] = (uint16_t)(Tx + Ty);  // B相
            pwm_ccr[2] = (uint16_t)Tx;         // C相
            break;
            
        case 4: 
            // 扇区4：C相钳位到高，零矢量用111
            pwm_ccr[0] = (uint16_t)Tx;         // A相
            pwm_ccr[1] = (uint16_t)(T_pwm - Tx - Ty); // B相
            pwm_ccr[2] = T_pwm;                // C相 全程不开关
            break;
            
        case 5: 
            // 扇区5：B相钳位到低，零矢量用000
            pwm_ccr[0] = (uint16_t)(T_pwm - Tx - Ty); // A相
            pwm_ccr[1] = 0;                    // B相 全程不开关
            pwm_ccr[2] = (uint16_t)(Tx + Ty);  // C相
            break;
            
        case 6: 
            // 扇区6：A相钳位到高，零矢量用111
            pwm_ccr[0] = T_pwm;                // A相 全程不开关
            pwm_ccr[1] = (uint16_t)(T_pwm - Ty);  // B相
            pwm_ccr[2] = (uint16_t)(T_pwm - Tx);  // C相
            break;
            
        default:
            pwm_ccr[0] = T_pwm / 2;
            pwm_ccr[1] = T_pwm / 2;
            pwm_ccr[2] = T_pwm / 2;
            break;
    }
}
```

---

### 三、和七段式的核心差异对比

| 对比项 | 七段式SVPWM | 五段式SVPWM |
| --- | --- | --- |
| 零矢量使用 | V0(000)和V7(111)都用，拆分对称放置 | 每个扇区只用一个零矢量，集中放置 |
| 每周期开关次数 | 6次 | 4次（减少1/3） |
| 开关损耗 | 较高 | 降低30%~40% |
| 电流谐波/转矩脉动 | 小，运行平稳 | 大，低速噪音明显 |
| 共模电压波动 | 小 | 大，EMI更差 |
| 单电阻采样难度 | 低，窗口充足 | 高，窄脉冲风险大 |

---

### 四、优缺点与适用场景

#### 优点

1. **开关损耗显著降低**：大功率、高开关频率（>20kHz）场景下，MOS管/IGBT温升能降5~8℃，散热器可以做更小。
2. **代码兼容度高**：前两步和七段式完全复用，切换成本极低。
3. **每相1/3时间不动作**：特别适合IGBT这类开关损耗占比高的功率器件。

#### 缺点

1. 电流THD（总谐波失真）比七段式高30%以上，低速转矩脉动大，电机电磁噪音更明显。
2. 共模电压跳变大，对电机轴承绝缘、系统EMC不友好。
3. 低调制比下容易出现窄脉冲，单电阻采样方案需要额外做脉冲平移。

#### 工程选型建议

- 低速高精度、低噪音需求（伺服、机器人）：用**七段式**。
- 大功率、高开关频率、效率优先（变频器、车载驱动、大功率风机水泵）：用**五段式**。
- 全工况最优方案：低速用七段式保平稳，中高速自动切五段式提效率，工业驱动器基本都是这个策略。

---

### 五、小白调试提醒

1. 先跑通七段式，确认电机转起来、电流波形正常，再换五段式对比，出问题更容易定位。
2. `T_pwm` 参数和七段式传一样的ARR值就行，单位不用改。
3. 用示波器看某相PWM波形，会看到有1/3周期占空比固定为0或100%，这就是钳位区间，说明五段式跑对了。

需要我补充一个「七段/五段自动切换」的混合策略代码吗？