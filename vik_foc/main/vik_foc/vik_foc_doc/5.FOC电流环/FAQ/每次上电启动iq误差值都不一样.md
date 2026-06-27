这是一个非常经典的 FOC 电流环调试“大坑”。你遇到的“每次上电实际 $I_q$ 都不一样、误差不一样、导致 $U_q$ 狂飙”的现象，结合你提供的源码和日志，可以非常精准地定位出原因。

核心问题出在你的**电角度零点电子对齐（Calibration）逻辑**以及D/Q轴完全耦合（错位）上。

---

## ## 核心原因分析

### 1. 致命伤：零点校准时引入了“随机振荡过程”

看看你的 `foc_task` 中未对齐时的逻辑（第 158-161 行）：

```c
}else{/*未进行电角度零点对齐*/
    curent_loop_park.Uq = 0.0f;
    curent_loop_park.Ud = 4.5f;
    ++zero_e_cnt;
    zero_e_mech_sum += get_vfoc_theta_m_deg(); // ← 问题在这里！

```

当系统刚上电时，转子停在哪个**随机物理位置**是不固定的。当你突然施加 `Ud = 4.5f` 时，转子会受到电磁力吸引向 D 轴 0 度位置拉过去。
但是在拉过去的过程中，转子由于惯性会**来回剧烈震荡、摆动**，最后才慢慢静止。
你从 `zero_e_cnt = 1`（也就是刚上电的第一微秒）就开始把机械角度累加进 `zero_e_mech_sum`。这意味着**转子在去往 0 度途中的所有狂摆、随机初始位置的偏差，全部被算进了平均值里！**
由于每次上电初始位置不同，震荡轨迹不同，导致每次算出来的 `zero_e_mech`（机械零点偏置）都不一样。零点偏置错了，后面运行时的电角度全盘皆错。

### 2. 日志暴露的秘密：D/Q 轴错位了 90 度

仔细看你抓取的这行关键日志：

> `I (3553) FOC_TASK: iq: 0.20,0.03,6.50, 0.00,0.27,0.00`

* **期望值**：$I_q = 0.20$， $I_d = 0.00$
* **实际反馈**：$I_q = 0.03$（几乎为0），$I_d = 0.27$
* **PID输出**：$U_q = 6.50$（已经飙到限幅最大值），$U_d = 0.00$

**诊断结果：** 你的电流环把所有的力气（$U_q = 6.5\text{V}$）都推向了所谓的“Q轴”，结果在物理上却全部产生在了“D轴”（实际 $I_d$ 顶到了 0.27A）。这说明**你的电角度计算和真实的物理轴线，刚好错位了接近 90 度！**

> **为什么你堵转电机时能达到 0.2？**
> 因为当你用手强行堵转/扭动电机时，你强行把转子的物理位置扭了 90 度，刚好对齐了它算错的那个轴，这时候反馈电流才看起来正常了。但这属于“歪打正着”，一旦松手或重新上电就会立刻失效。

---

## ## 解决方案

### 第一步：修改零点对齐逻辑（延迟采样）

对齐时，必须**先给电压让转子静止平稳下来，最后 1 秒再去读取角度求平均**。
修改 `foc_task` 的对齐部分：

```c
} else { /* 未进行电角度零点对齐 */
    curent_loop_park.Uq = 0.0f;
    curent_loop_park.Ud = 4.5f; // 注入 D 轴电压锁定位置
    ++zero_e_cnt;
    
    // 假设 50us 一个周期，20000 次 = 1000ms (1秒)
    // 前 2 秒 (40000次) 让电机充分震荡并静止，不记录数据
    // 后 1 秒 (40000 ~ 60000次) 当电机完全静止后，再累加角度
    if (zero_e_cnt > 40000) {
        zero_e_mech_sum += get_vfoc_theta_m_deg();
    }
    
    if (zero_e_cnt >= (20 * 3000)) /* 60000 次 = 3000ms */
    {
        // 强清电角度
        set_vfoc_theta_e_rad(0.0f);
        
        // 计算平均值时，除以实际参与累加的有效次数 (60000 - 40000 = 20000)
        set_theta_e_offset_mech(zero_e_mech_sum / (zero_e_cnt - 40000));
        
        // 【重要】在进入电流环前，必须把上一次 LPF 滤波值和 PID 积分全部清零！
        curent_loop_iq_pid.now_v = 0.0f;
        curent_loop_id_pid.now_v = 0.0f;
        curent_loop_iq_pid.err_v = 0.0f;
        curent_loop_id_pid.err_v = 0.0f;
        // 如果你的 PID 结构体有积分项（如 integral），也要在这里清零：
        // curent_loop_iq_pid.ki_integral = 0.0f; 
        
        set_zero_theta_e_calib_flag(true);
        zero_e_cnt = 0;
        
        ESP_LOGI(TAG, "真正平稳后的零点机械角度: %.2f", get_theta_e_offset_mech());
    }
}

```

### 第二步：确认电角度计算方向

你在代码里写了一句注释：`/*和克拉克，帕克变换的方向反了*/`。
请务必确保：**当给电机注入正的 $U_q$ 时，电机转动引起编码器输出的机械角度 `get_vfoc_theta_m_deg()` 必须是增加的。**
如果是减少的，说明编码器方向和电机电磁旋转方向相反。你需要在计算电角度的地方加上负号，或者将电机的任意两相相线（如 A、B 相）对调。

### 第三步：防止上电瞬间 LPF 历史值作怪

你的 $I_q$ 反馈使用了低通滤波器（LPF）：

```c
curent_loop_iq_pid.now_v = current_lpf(park_temp.Uq, curent_loop_iq_pid.now_v);

```

在上電对齐阶段，电流环没有运行，`now_v` 一直是 `0.0f`。当对齐结束切换到闭环的第一帧，由于电机里本来就憋着 `Ud = 4.5V` 的巨大电流，突然进行 Park 变换会产生一个很大的瞬时电流值。
如果不对 `now_v` 进行初始化复位（见第一步修改的代码），第一帧计算出的 `err_v` 会非常诡异，加上你高达 `50.0` 的 $K_p$，直接会让 $U_q$ 瞬间饱和。

按照第一步的方法**推迟对齐采样时间**并**在切换时复位电流环变量**，你每次上电测得的机械零点就会极其固定，$I_q$ 乱跳和 $U_q$ 狂飙的问题就能迎刃而解。

你目前使用的 AS5600 编码器，在不手动干预的情况下，给电机通入正向速度时，读取到的机械角度值是递增的还是递减的？


-------
--------
-------
你这个现象**实际上是闭环电流控制中非常典型的问题**，几乎所有做FOC的人都会遇到。

> **同样的程序，每次上电，Iq反馈值都不同，例如：**
>
> 第一次上电：Iq = -0.02A
>
> 第二次：Iq = +0.05A
>
> 第三次：Iq = -0.08A
>
> 导致：
>
> ```
> err = Iq_ref - Iq
> ```
>
> 每次都不同，所以
>
> ```
> Uq = PID(err)
> ```
>
> 一开始输出的Uq也不同。

这个一般**不是PID的问题**，而是反馈量没有真正稳定。

---

# 我看了你的代码

你的Iq反馈是：

```c
curent_loop_iq_pid.now_v =
    current_lpf(
        park_temp.Uq,
        curent_loop_iq_pid.now_v
    );
```

然后

```c
err = exp - now;
```

所以现在问题变成：

> **为什么 park_temp.Uq 每次上电都不一样？**

---

# 我觉得有几个原因（按概率排序）

---

# 第一名：ADC零漂每次都有一点点不同（★★★★★）

这个概率最高。

因为你的日志里：

```
CURRENT_ADC:
M0_Ia zero_vref :1668mv
M0_Ib zero_vref :1669mv
```

实际上如果你重启很多次，大概率会变成

```
1667
1669
1670
1668
...
```

ADC Offset 本来就不会完全一样。

ADC：

```
±1LSB
```

甚至

```
±2LSB
```

都是正常。

经过：

```
ADC
↓

Ia

↓

Clark

↓

Park
```

最后

```
Iq
```

可能就是

```
±0.02A

±0.05A

±0.08A
```

这已经很常见。

---

# 第二名：AS5600机械角度启动位置不同（★★★★★）

你这里：

```
AS5600:
193.80°
```

然后你又做了

```
zero_e_mech
```

例如

```
174.97°
```

虽然最后

```
theta_e(rad)=0
```

但是注意：

你取的是

```
3000次平均
```

```c
zero_e_mech_sum += get_vfoc_theta_m_deg();
```

AS5600自己就有

```
±1LSB

±2LSB
```

噪声。

所以：

```
174.96°

174.99°

175.02°
```

都会出现。

转换成

```
电角度
```

再经过

```
Park
```

最后

```
Iq
```

也会偏一点。

---

# 第三名：MOS开启瞬间电流没有稳定（★★★★★）

你的流程：

```
ADC校准

↓

MOS打开

↓

100ms

↓

开始FOC
```

但是：

MOS打开以后：

```
死区

MOS温漂

OPA建立时间

采样电阻热漂
```

都还没完全稳定。

很多工业驱动都会：

```
MOS ON

↓

等待100~300ms

↓

开始PID
```

甚至：

```
等待500ms
```

---

# 第四名：Iq低通滤波器初值问题（★★★★★）

你的滤波：

```c
current_lpf(in, old)
```

里面：

```
old
```

来自：

```c
curent_loop_iq_pid.now_v
```

而：

```
第一次：

old=0
```

例如：

真正Iq

```
0.08A
```

第一次：

```
0+0.4*(0.08)

=

0.032
```

第二次：

```
0.051
```

第三次：

```
0.062
```

所以：

刚开始几十个周期：

```
Iq
```

都是假的。

PID已经开始工作了。

工业FOC一般都会：

```
先采100个周期

↓

LPF稳定

↓

再打开PI
```

而不是：

```
第一拍

↓

直接PI
```

---

# 第五名：PI积分器没有初始化（★★★★）

你这里：

```c
vfoc_pid_t curent_loop_iq_pid={0};
```

虽然第一次启动积分是0。

但是：

如果以后：

```
重新校准

重新Enable

重新启动FOC
```

一定要：

```c
pid.integral = 0;
pid.last_err = 0;
pid.out = 0;
```

否则：

上一轮积分还在。

---

# 第六名：Park变换角度抖动（★★★★）

你的：

```c
park_tansform(
    ...
    get_vfoc_theta_e_rad(...)
);
```

如果：

```
theta

每次：

0.02°

0.1°

0.3°
```

那么：

```
Iq

Id
```

都会变。

特别堵转的时候最明显。

---

# 第七名：PWM开启时刻不同（★★★）

你的：

```
PWM ISR

↓

ADC

↓

Task

↓

SVPWM
```

理论没问题。

但是：

第一次：

```
ADC刚启动
```

很多ADC：

第一帧数据都是垃圾。

所以工业代码经常：

```
丢弃前

10

20

50

100

次ADC
```

再开始PID。

---

# 我建议你现在重点排查

先不要看PID。

先打印：

```c
ESP_LOGI(TAG,
"ia=%f ib=%f ic=%f iq=%f id=%f theta=%f",
...
);
```

**刚上电（PI还没打开）**打印：

连续100个周期：

看看：

```
Iq
```

是不是：

```
第一次

-0.07

第二次

0.03

第三次

-0.02
```

如果这里已经不同，

说明：

**PID根本没问题。**

反馈就已经漂了。

---

# 工业FOC一般都会这样做

真正的工业驱动，上电顺序一般是：

```
① ADC Offset校准

↓

② MOS打开

↓

③ 等待100~300ms

↓

④ 连续采100~500次ADC

↓

⑤ 更新LPF

↓

⑥ Iq、Id稳定

↓

⑦ PID积分清零

↓

⑧ 打开Iq环
```

而不是：

```
MOS ON

↓

马上PID
```

---

## 我还注意到你代码里的一个细节

你在零电角校准完成后，立刻就开始进入电流环：

```c
if (get_zero_theta_e_calib_flag())
{
    vfoc_curent_loop();
}
```

建议在这里增加一个"稳定等待"状态，例如：

```c
if (get_zero_theta_e_calib_flag())
{
    static uint16_t warmup_cnt = 0;

    if (warmup_cnt < 200)   // 20kHz下约10ms
    {
        // 只更新滤波器，不运行PID
        curent_loop_iq_pid.now_v = current_lpf(park_temp.Uq, curent_loop_iq_pid.now_v);
        curent_loop_id_pid.now_v = current_lpf(park_temp.Ud, curent_loop_id_pid.now_v);
        warmup_cnt++;
        return;
    }

    vfoc_curent_loop();
}
```

等滤波器和采样稳定后，再开启PI控制，这样每次上电的初始 `Iq` 和 `Uq` 一致性会明显改善。

---

**我还有一个问题想确认一下：**

你说的"每次上电实际 `Iq` 都不一样"，是指**电机完全没有堵转、没有施加任何 `Uq` 之前**测到的 `Iq` 就已经不同，还是指**进入闭环以后**打印出来的 `Iq` 不同？这两种情况对应的排查方向会有所不同。

---------
---------
----------

# 一、先定位：每次上电iq反馈不一致、uq输出不同的根本原因
结合你代码+日志分4大类根源，按出现概率从高到低排序：
## 1. 电流ADC采样零点漂移（最核心、最常见）
### 现象匹配你的日志
上电会打印电流零漂校准：
```
CURRENT_ADC: M0_Ia ,zero_vref:1668mv
CURRENT_ADC: M0_Ib ,zero_vref:1669mv
```
### 原理
运放、电阻、ESP32内部ADC基准**温漂、上电电压波动**，每次开机三相电流零点电压不完全相同；
虽然你做了零漂校准，但存在3个漏洞：
1. **校准时机缺陷**：校准只在上电开机执行一次，电机静止时温度、电源纹波每次上电都有差异；
2. **静态滤波初值未复位**
```c
static inline float current_lpf(float in, float old)
{
    const float alpha = 0.40;
    return old + alpha * (in - old);
}
```
`curent_loop_iq_pid.now_v`、`curent_loop_id_pid.now_v`是**static隐式静态变量**，FreeRTOS任务重启/芯片复位后RAM随机脏值，第一次滤波`old`是随机浮点数，前几十次周期Iq反馈持续偏移，上电每次初始值完全不同；
3. Clark/Park变换依赖Ia/Ib/Ic原始采样，零点偏移直接体现在Iq反馈直流偏置，`exp_v=0.2`固定，误差`err_v=0.2-now_iq`每次上电不一样，PI输出Uq自然不同。

## 2. 电角度初始偏移校准存在波动（零点标定重复性差）
上电执行d轴注入4.5V找机械零点：
```c
curent_loop_park.Ud = 4.5f;
zero_e_mech_sum += get_vfoc_theta_m_deg();
```
日志打印`zero_e_mech:174.97`，每次上电该值会小幅浮动：
- AS5600上电初始读数随机；
- 4.5V d轴定位时电机轻微抖动，每次采样平均机械角度偏移不一致；
Park变换电角度`theta_e`存在固定偏差，直接导致Iq/Id解耦出错，Iq反馈带固定偏置，每次上电偏差量不同。

## 3. PID积分项静态存储，上电随机脏值
```c
vfoc_pid_t curent_loop_iq_pid = {0};
```
全局结构体初始化看似清零，但分两种场景：
1. **冷上电复位**：全局变量段会清零，积分无问题；
2. **软件看门狗/软重启**：RAM不复位，积分保留上次停机数值；
关键问题：**滤波输出now_v是static临时状态，不受全局初始化管控**，每次上电第一帧滤波初始值随机，Iq反馈偏置随机。

## 4. 硬件层面：母线电压、采样电阻温漂、MOS管导通压降不一致
每次上电瞬间芯片温度不同，采样电阻阻值微小变化，电流换算系数出现微小偏差，叠加ADC零点偏移，放大Iq静态误差。

# 二、分步骤解决方案（按优先级修改代码）
## 方案1：修复一阶低通滤波静态随机初值（必改，解决上电Iq初始随机）
### 问题根源
`current_lpf`使用的`curent_loop_iq_pid.now_v`、`curent_loop_id_pid.now_v`是持续保存滤波历史的状态变量，上电第一次运行old是随机值。
### 修改方法
1. 在**电角度标定完成、FOC正式运行前**，重置滤波历史值；
2. 标定阶段不启用电流滤波，标定完成后初始化滤波缓存为当前采样值。

修改`foc_task`标定分支：
```c
if ( get_zero_theta_e_calib_flag() )
{
    vfoc_curent_loop();
}else{
    curent_loop_park.Uq = 0.0f;
    curent_loop_park.Ud = 4.5f;
    ++zero_e_cnt;
    zero_e_mech_sum += get_vfoc_theta_m_deg();
    if ( zero_e_cnt >=(20*3000) )
    {
        set_vfoc_theta_e_rad(0.0f);
        set_theta_e_offset_mech( zero_e_mech_sum / zero_e_cnt );
        set_zero_theta_e_calib_flag(true);
        zero_e_cnt = 0;
        ESP_LOGI(TAG,"zero_e_mech:%.2f",get_theta_e_offset_mech());

        // =========新增：标定完成，初始化滤波缓存，消除上电随机初值========
        clark_parm_t clark_temp = clark_tansform(get_vfoc_ia_current(),get_vfoc_ib_current(),get_vfoc_ic_current());
        park_parm_t park_temp = park_tansform(clark_temp.I_alpha,clark_temp.I_beta,get_vfoc_theta_e_rad(get_theta_e_offset_mech()));
        curent_loop_iq_pid.now_v = park_temp.Uq; // 滤波初始值设为当前真实采样，不再随机
        curent_loop_id_pid.now_v = park_temp.Ud;
        // 同时清零PID积分，避免上电积分残留
        curent_loop_iq_pid.integral = 0.0f;
        curent_loop_id_pid.integral = 0.0f;
    }
}
```

## 方案2：优化ADC电流零点校准（解决采样直流偏置）
你当前仅上电一次性校准，优化两点：
1. **电机启动前二次校准**：MOS打开、电机静止无电流时再跑一次零漂采集；
2. **运行时动态小范围校准**：低速/零Iq时缓慢修正零点，抑制温漂；
3. 代码保证校准期间PWM完全关闭，杜绝电机感应电流干扰零点采样。

## 方案3：提升电角度零点标定重复性（减小Park变换带来的Iq偏移）
原标定逻辑是长时间平均机械角度，优化：
1. d轴注入电压稳定一段时间（延长采样周期），等待电机完全吸合定位后再采集角度；
2. 多次采样剔除极值再取平均，减少单次抖动带来的偏移；
3. 标定完成后固定偏移值，可存入NVS，下次上电直接读取，无需每次重新标定（大幅提升一致性）。
示例NVS保存偏移：
```c
// 标定完成后存入NVS
nvs_set_float("motor0","theta_offset",zero_e_mech_sum/zero_e_cnt);
// 上电先读取，存在则直接使用，跳过标定
float saved_offset = 0;
if(nvs_get_float("motor0","theta_offset",&saved_offset)==ESP_OK){
    set_theta_e_offset_mech(saved_offset);
    set_zero_theta_e_calib_flag(true);
}
```

## 方案4：电流环增加静态消除（抵消采样零点带来的固定偏置）
每次上电Iq存在固定直流偏移，可增加Iq静态补偿量：
1. 电机堵转稳态时，记录`exp_iq - steady_now_iq`作为补偿值；
2. 下一次周期反馈Iq减去补偿值，消除ADC零点带来的固定偏差；
```c
// vfoc_curent_loop内
static float iq_static_offset = 0.0f;
float raw_iq = park_temp.Uq;
curent_loop_iq_pid.now_v = current_lpf(raw_iq - iq_static_offset, curent_loop_iq_pid.now_v);
```
稳态自动学习offset，每次上电漂移会被补偿。

## 方案5：PID上电积分限幅+积分清零保护
当前代码仅限制积分上下限，但上电随机滤波值会导致积分快速累积：
- 标定完成后强制清零Id/Iq PID积分项（方案1已加）；
- 增加积分分离：Iq误差过大时冻结积分，防止上电瞬间积分饱和导致Uq输出偏移。

# 三、补充验证方法（确认问题是否解决）
1. 打印原始未滤波Park输出`park_temp.Uq`，对比滤波后`curent_loop_iq_pid.now_v`
   - 若第一次`park_temp.Uq`每次上电数值不同 → ADC零点/角度标定问题；
   - 若`park_temp.Uq`稳定、仅滤波后now_v随机 → 滤波静态初值问题；
2. 屏蔽PID输出，直接固定Uq，观察Iq反馈是否每次上电一致：
   ```c
   //#if 1
   //curent_loop_park.Uq = curent_loop_iq_pid.pid_out;
   curent_loop_park.Uq = 6.5f; // 固定输出
   ```
   此时Iq反馈仍每次不同 → 硬件采样/角度标定问题；
3. 关闭d轴定位标定，手动写死`theta_e_offset_mech`固定值，观察Iq一致性：
   - 一致：零点标定重复性差；
   - 依旧不一致：ADC电流零点漂移为主因。

# 四、最简快速修复（优先改这一段，立刻改善）
只修改标定完成分支，初始化滤波缓存+清零PID积分，最低改动、见效最快：
```c
if ( zero_e_cnt >=(20*3000) )
{
    set_vfoc_theta_e_rad(0.0f);
    float avg_mech = zero_e_mech_sum / zero_e_cnt;
    set_theta_e_offset_mech(avg_mech);
    set_zero_theta_e_calib_flag(true);
    zero_e_cnt = 0;
    ESP_LOGI(TAG,"zero_e_mech:%.2f",avg_mech);

    // 重置滤波与积分，消除上电随机脏值
    clark_parm_t clark_temp = clark_tansform(get_vfoc_ia_current(),get_vfoc_ib_current(),get_vfoc_ic_current());
    park_parm_t park_temp = park_tansform(clark_temp.I_alpha,clark_temp.I_beta,0);
    curent_loop_iq_pid.now_v = park_temp.Uq;
    curent_loop_id_pid.now_v = park_temp.Ud;
    curent_loop_iq_pid.integral = 0.0f;
    curent_loop_id_pid.integral = 0.0f;
}
```

