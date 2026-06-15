如果是**ESP32（不是STM32）做FOC + 二轮平衡车**，那么要求比普通位置控制要高，因为平衡车对**延迟**非常敏感。

结合ESP32（240MHz）、二轮平衡车、FOC控制，我推荐下面这套配置。

---

# 推荐方案（★★★★★）

| 模块                            | 推荐频率             | 周期     |
| ----------------------------- | ---------------- | ------ |
| PWM(SVPWM)                    | **20 kHz**       | 50 us  |
| ADC电流采样                       | **20 kHz**       | 50 us  |
| 电角度读取（Encoder/MT6701/AS5600等） | **20 kHz**       | 50 us  |
| 电流环(Id/Iq PID)                | **20 kHz**       | 50 us  |
| Clarke + Park                 | **20 kHz**       | 50 us  |
| SVPWM更新                       | **20 kHz**       | 50 us  |
| 速度环                           | **500 Hz~1 kHz** | 1~2 ms |
| 平衡角度环(IMU)                    | **500 Hz**       | 2 ms   |
| 外层位置环（可选）                     | **100 Hz**       | 10 ms  |

这是很多工业FOC以及开源项目（如VESC、ODrive）的设计思路：**最内层电流环最高频，外环逐级降低频率。**

---

# 如果ESP32算力吃紧

可以采用下面这个方案。

| 模块      | 推荐频率       |
| ------- | ---------- |
| PWM     | 20 kHz     |
| ADC     | 20 kHz     |
| Encoder | 20 kHz     |
| 电流环     | **10 kHz** |
| 速度环     | 500 Hz     |
| IMU姿态环  | 500 Hz     |

也就是说：

```text
PWM：
20kHz

↓↓

ADC：
20kHz

↓↓

Encoder：
20kHz

↓↓

每2次ADC

执行一次Current PID
```

这样CPU压力会明显下降。

---

# 为什么推荐20kHz？

因为：

```text
20kHz

↓

50us
```

50us一次控制。

例如电机：

```
3000RPM
```

机械角速度：

```
3000/60

=

50rps
```

电角速度（7极对）：

```
50×7

=

350Hz
```

电角周期：

```
1/350

≈2.85ms
```

50us更新一次：

```
2.85ms

/

50us

≈57次
```

也就是说一个电角周期会更新约57次，足够平滑。

即使：

```
5000RPM
```

仍然有三十多次更新。

---

# Encoder频率为什么建议20kHz？

很多人觉得Encoder不用这么快。

其实FOC里：

```text
Current PID

↓

Park

↓

必须知道当前角度
```

所以最好：

```
ADC

↓

Encoder

↓

Current PID
```

保持同步。

如果Encoder只有1kHz：

```
1ms
```

而Current PID：

```
50us
```

那么20次PID都会使用同一个角度，电流控制性能会明显下降。

---

# IMU不要20kHz

平衡车真正控制的是：

```
IMU

↓

姿态角

↓

速度

↓

目标Iq
```

IMU更新速度通常是：

```
200Hz

500Hz

1000Hz
```

已经足够。

例如：

```
MPU6050
```

一般：

```
500Hz
```

效果就很好。

BMI270等高性能IMU可运行到800Hz甚至1kHz。

没必要20kHz。

---

# 推荐任务划分（ESP32 + FreeRTOS）

可以这样划分：

```text
Core0

50us（20kHz）
    ADC
    Encoder
    Clarke
    Park
    Current PID
    InvPark
    SVPWM

--------------------

Core1

2ms（500Hz）
    IMU读取
    卡尔曼/互补滤波
    Balance PID

--------------------

Core1

2ms（500Hz）
    Speed PID

--------------------

Core1

10ms（100Hz）
    Position PID
```

这样CPU负载均衡，也能减少控制链路的抖动。

---

# 我最推荐你当前项目的配置

考虑到你现在做的是**ESP32 + FOC + 二轮平衡车**，并且后续还要做速度环、姿态环，我建议直接采用下面这一套：

| 模块            | 频率                              |
| ------------- | ------------------------------- |
| PWM           | **20 kHz**                      |
| ADC电流采样       | **20 kHz（PWM定时触发）**             |
| 编码器/磁编码器角度读取  | **20 kHz**                      |
| FOC电流环（Id/Iq） | **20 kHz**                      |
| IMU读取与姿态解算    | **500 Hz**（若IMU支持且CPU充裕可到1 kHz） |
| 平衡控制环         | **500 Hz**                      |
| 速度环           | **500 Hz**                      |
| 位置环（如需要）      | **100 Hz**                      |

### 还有一个关键建议

**不要用普通的 FreeRTOS 周期 Task（`vTaskDelayUntil()`）来跑 20 kHz 的电流环。**

20 kHz意味着每50 μs执行一次，普通任务调度很难保证这个周期稳定。更好的方式是：

* **MCPWM** 输出20 kHz PWM。
* **ADC** 由PWM定时触发采样。
* ADC采样完成后，通过**DMA或中断**通知。
* 在ADC完成中断（或高优先级ISR）中完成FOC计算，或者唤醒一个最高优先级任务立即执行FOC。

这样电流采样、角度读取和SVPWM更新都严格与PWM同步，控制效果会比单纯依赖RTOS定时任务稳定得多。

--------
----------
---------
# 角度值采样频率必须大于电流采样频率
**这句话基本是对的，但要准确理解它的真正含义。**

准确的说法应该是：

> **电角度数据不能比电流数据"更旧"（stale），而不是角度采样频率必须大于或等于电流采样频率。**

这是两个完全不同的概念。

---

## 为什么会有这个说法？

FOC里面，Park变换是：

```text
Ia
Ib
+
θe

↓

Id
Iq
```

如果：

```text
电流：
t = 100us 时刻

角度：
t = 50us 时刻
```

那么实际上计算的是：

```text
100us 的电流

+

50us 的角度
```

电流和角度不是同一时刻。

结果：

```text
Park变换错误

↓

Id、Iq计算错误

↓

PID输出错误

↓

SVPWM方向错误
```

于是就会出现：

* 电机没劲
* 电流变大
* 发热
* 抖动
* 高频啸叫

所以大家才总结一句：

> **角度更新不能落后于电流。**

---

## 举个例子

### 错误情况

```text
Encoder

1kHz

↓

每1ms更新一次
```

而：

```text
ADC

20kHz
```

50us采一次。

那么：

```text
1000us

↓

Encoder更新一次
```

期间：

```text
ADC

采20次
```

20次FOC都在使用：

```text
旧角度
```

如果电机高速：

例如：

```
3000RPM
```

1ms期间：

机械角：

```
18°
```

7极对：

```
18×7

=

126°
```

意味着：

**1ms以后角度已经差126°了！**

Park几乎完全错了。

当然会抖。

---

## 那为什么很多项目角度也是20kHz？

因为：

每次ADC之后：

```text
PWM

↓

ADC

↓

读取Encoder

↓

FOC
```

所以：

```text
ADC：

50us
```

对应：

```text
Encoder：

50us
```

两者属于同一时刻。

这是最好的。

---

## 如果Encoder只有10kHz呢？

例如：

```text
ADC

20kHz

Encoder

10kHz
```

是不是一定不行？

**不一定。**

例如：

```
0us

Encoder更新
```

```
0us

ADC
```

```
50us

ADC
```

```
100us

Encoder更新
```

第二次ADC：

```
50us
```

仍然使用：

```
0us
```

角度。

如果：

电机：

```
100RPM
```

50us：

机械角：

```
0.03°
```

几乎没有误差。

所以：

完全没问题。

---

## 真正决定因素是什么？

不是：

> **频率。**

而是：

> **角度在一次Current Loop期间变化了多少。**

例如：

如果：

```
Encoder

10kHz
```

电机：

```
100RPM
```

没问题。

如果：

```
Encoder

10kHz
```

电机：

```
8000RPM
```

那就废了。

---

# 对于你的二轮平衡车

平衡车一般：

```
0~800RPM
```

甚至：

```
500RPM
```

都很多了。

以：

```
500RPM

7极对
```

计算：

机械：

```
8.3rps
```

电角：

```
58Hz
```

电角周期：

```
17ms
```

如果：

```
Encoder

10kHz
```

100us更新一次。

100us：

电角变化：

```
360°

×

100us

/

17ms

≈2°
```

只有2°。

对于FOC来说：

**几乎没有影响。**

---

# 我的建议

对于你的项目（ESP32 + 二轮平衡车 + MT6701），我仍然建议：

| 模块      | 推荐频率                |
| ------- | ------------------- |
| PWM     | **20 kHz**          |
| ADC电流采样 | **20 kHz**          |
| 电角度更新   | **20 kHz**（最佳）      |
| 电流环     | **20 kHz**（或10 kHz） |

原因不是因为**角度频率必须≥电流频率**，而是因为：

* 同样都是20 kHz时，**每次电流采样后都能立即读取最新角度**。
* 每一次FOC计算使用的是**同一时刻**的电流和角度数据。
* 时序简单、同步性最好，也是工业FOC最常见的实现方式。

**因此，你看到的那句话更准确地应理解为：**

> **角度信息不能比电流信息滞后到影响Park变换，而不是机械地要求"角度采样频率必须高于或等于电流采样频率"。**如果你的角度读取和电流采样能够同步完成，那么二者同为20 kHz就是非常理想的配置。


-------
--------
------

我看了你的代码，**我不建议搞两个20kHz的Task。**

你的思路已经越来越接近工业FOC了，但是**架构还差最后一步**。

---

# 我先说结论

**推荐程度：**

⭐⭐⭐⭐⭐ **方案一（推荐）**

> ADC完成回调 → 唤醒一个FOC Task（唯一20kHz Task）

里面依次执行：

```text
ADC DMA完成
      │
      ▼
Current Task醒来
      │
      ├── 读取ADC
      ├── 计算Ia Ib
      ├── 读取AS5600角度
      ├── Clarke
      ├── Park
      ├── PID(Id/Iq)
      ├── InvPark
      ├── SVPWM
      └── 返回等待
```

**不要再单独搞一个Encoder Task。**

---

# 为什么？

因为FOC其实是一条流水线。

工业里面基本都是：

```text
ADC完成

↓

Current

↓

Angle

↓

FOC

↓

PWM
```

不是：

```text
Current Task

↓

Encoder Task

↓

FOC Task
```

因为这样：

Task切换：

```
Current

↓

Context Switch

↓

Encoder

↓

Context Switch

↓

FOC
```

ESP32：

每次Task切换：

```
几微秒
```

20kHz：

```
50us
```

两次切换：

```
5~10us
```

已经占掉20%。

---

# 你的Current Task其实已经很合适

你的ADC完成回调：

```c
vTaskNotifyGiveFromISR(
    motor_current_adc_task_handle,
    &must_yield
);
```

这个非常标准。

然后：

```c
ulTaskNotifyTake(...)
```

读取DMA。

然后：

```
Ia

Ib
```

计算出来。

**下一步不要通知Encoder Task。**

直接：

```c
motor_encoder_get_angle(&angle);

set_vfoc_theta_m_deg(angle);

vfoc_current_loop();

svpwm();
```

全部在Current Task完成。

---

# 为什么？

因为：

这一刻：

```
Ia

Ib
```

就是：

```
t = 100us
```

那么：

马上：

```
读Encoder
```

得到：

```
t = 102us
```

几乎就是同一个时刻。

如果：

通知另一个Task：

```
Task切换

↓

Scheduler

↓

Encoder Task

↓

I2C

↓

回来
```

可能：

```
t =115us
```

已经晚了。

---

# GPTimer呢？

我反而不推荐。

例如：

```
GPTimer

↓

Current Task

Encoder Task
```

两个Task：

一起Ready。

然后：

```
Current

↓

Encoder
```

或者：

```
Encoder

↓

Current
```

完全取决于：

Priority。

如果：

Priority一样：

FreeRTOS：

```
时间片
```

谁先谁后都可能。

---

# 你说：

> timer中断里释放两个task通知

**不推荐。**

因为：

你真正需要的是：

```
Current

↓

Angle

↓

FOC
```

固定顺序。

不是：

```
Current

Encoder
```

同时跑。

---

# 也不要这样：

> ADC回调通知Current，再Current通知Encoder

这样：

```
ISR

↓

Current

↓

Notify

↓

Encoder

↓

Notify

↓

FOC
```

Task越来越多。

Latency越来越大。

---

# 工业FOC怎么干？

例如：

STM32

TIM1

↓

PWM Update

↓

ADC Trigger

↓

ADC End

↓

FOC_ISR()

里面：

```c
current_get();

encoder_get();

clarke();

park();

pid();

svpwm();
```

结束。

没有：

```
Encoder Task
```

---

# 你的AS5600还有一个问题

我看了你的代码。

AS5600：

```c
motor_encoder_get_angle()
```

里面：

实际上：

```c
as5600_get_angle_degrees()
```

走的是：

I2C。

这意味着：

**这里才是真正的问题。**

I2C：

400kHz。

读一次：

一般：

```
40~100us
```

而：

你的Current Loop：

```
50us
```

已经不够了。

所以：

**AS5600根本不适合20kHz Current Loop。**

---

# 如果坚持AS5600

建议：

```
Current

10kHz
```

```
Encoder

10kHz
```

全部放一起。

---

# 如果以后换MT6701 SPI

就可以：

```
20kHz

↓

ADC

↓

SPI

↓

FOC
```

完全没问题。

---

# 我的建议（非常推荐）

结合你目前的代码，我建议最终改成下面这个架构：

```text
ADC DMA完成中断
        │
        ▼
vTaskNotifyGiveFromISR(CurrentTask)
        │
        ▼
CurrentTask（唯一20kHz高优先级任务）
        │
        ├── adc_continuous_read()
        ├── 计算Ia、Ib、Ic
        ├── motor_encoder_get_angle()   // 立即读取最新角度
        ├── Clarke
        ├── Park
        ├── Id/Iq PID
        ├── 反Park
        ├── SVPWM更新
        └── 再次阻塞等待ADC通知
```

外层控制则保持独立：

* **IMU姿态任务**：500Hz～1kHz
* **平衡环PID**：500Hz
* **速度环PID**：200Hz～500Hz
* **位置环PID（如果需要）**：50Hz～100Hz

整个系统中**只有一个20kHz实时任务**，这样时序最稳定、上下文切换最少，也是工业电机控制器最常见的架构。

**不过还有一个关键问题：**我注意到你目前使用的是 **AS5600 + I2C（400kHz）**。如果你真的打算把FOC电流环提升到20kHz，我建议先讨论一下这一点，因为**I2C版AS5600很可能会成为整个控制链路的瓶颈**。如果后续要做高性能平衡车，我会建议你换成SPI磁编码器（如MT6701 SPI、AS5047P等），或者至少不要让I2C读取成为20kHz环路的一部分。


---------
--------
-------
这是一个非常好的问题，也是**FOC实时控制最核心的时序**。

很多人以为FOC就是：

```text
读取角度
读取电流
Park
PID
```

其实**工业FOC并不是这样随便排列的**，而是严格按照**数据同步**来设计。

---

# 最推荐的时序（工业FOC）

假设PWM=20kHz（50us）。

每个PWM周期：

```text
                第N个PWM周期

PWM输出
    │
    │（MOS导通一段时间，电流稳定）
    ▼
ADC采样（Ia、Ib）
    │
    ▼
ADC DMA完成中断
    │
    ▼
FOC Task开始
    │
    ├──① 获取ADC结果(Ia、Ib)
    │
    ├──② 获取Encoder角度θ
    │
    ├──③ Clarke变换
    │
    ├──④ Park变换
    │
    ├──⑤ Id PID
    │
    ├──⑥ Iq PID
    │
    ├──⑦ 反Park
    │
    ├──⑧ SVPWM计算
    │
    └──⑨ 更新PWM比较值
```

下一PWM周期开始输出新的占空比。

整个过程就是一条流水线。

---

# 为什么先读ADC？

因为ADC采样已经完成了。

例如：

```text
PWM开始

↓

ADC Trigger

↓

ADC转换

↓

DMA完成
```

DMA完成以后：

说明：

```
Ia
Ib
```

已经准备好了。

所以：

第一件事：

```c
adc_continuous_read(...);
```

得到：

```
Ia

Ib
```

---

# 为什么第二步读取角度？

因为：

Encoder不像ADC。

Encoder：

```
随时都可以读。
```

所以：

紧接着：

```c
theta = encoder_get_angle();
```

这样：

```
Ia

Ib

θ
```

时间差：

一般：

```
2~5us
```

几乎可以认为：

属于同一时刻。

---

# 然后再Clarke

例如：

ADC：

```
Ia

Ib
```

那么：

```text
Ia

Ib

↓

Clarke

↓

Iα

Iβ
```

这里只涉及电流。

跟角度没关系。

---

# 再Park

Park需要：

```
Iα

Iβ

θ
```

得到：

```
Id

Iq
```

即：

```text
Iα
Iβ
θ

↓

Park

↓

Id

Iq
```

---

# 然后PID

例如：

目标：

```
Id_ref

Iq_ref
```

计算：

```
err_d

↓

PID

↓

Ud
```

再：

```
err_q

↓

PID

↓

Uq
```

得到：

```
Ud

Uq
```

---

# 然后反Park

利用：

```
Ud

Uq

θ
```

得到：

```
Uα

Uβ
```

---

# 最后SVPWM

利用：

```
Uα

Uβ
```

计算：

```
Ta

Tb

Tc
```

更新PWM。

---

# 所以真正顺序就是

```text
ADC完成
      │
      ▼
① 读取ADC
      │
      ▼
② 获取Encoder角度
      │
      ▼
③ Clarke
      │
      ▼
④ Park
      │
      ▼
⑤ Id PID
      │
      ▼
⑥ Iq PID
      │
      ▼
⑦ 反Park
      │
      ▼
⑧ SVPWM
      │
      ▼
⑨ PWM Compare更新
```

---

# 为什么不是先获取角度？

很多人写：

```text
读Encoder

↓

读ADC
```

其实也能跑。

但是：

Encoder：

```
t=100us
```

ADC：

```
t=105us
```

那么：

```
角度：

100us

电流：

105us
```

时间不同。

虽然只有5us。

但是：

如果ADC已经采好了。

当然：

先把ADC拿出来。

然后：

立即读Encoder。

这样：

```
电流：

100us

角度：

101us
```

时间更接近。

---

# ESP32推荐流程

你现在：

```c
current_adc_conv_done_cb()
{
    vTaskNotifyGiveFromISR(current_task);
}
```

CurrentTask：

```c
for(;;)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* ① DMA数据 */
    motor_adc_get_current();

    /* ② Encoder */
    motor_encoder_get_angle();

    /* ③ Clarke */
    vfoc_clarke();

    /* ④ Park */
    vfoc_park();

    /* ⑤ 电流环PID */
    vfoc_current_pid();

    /* ⑥ 反Park */
    vfoc_inv_park();

    /* ⑦ SVPWM */
    vfoc_svpwm();
}
```

这是我建议的实现顺序。

---

# 如果后面还有速度环、位置环怎么办？

工业FOC通常采用**多环串级控制**：

```text
位置环（100Hz）
        │
        ▼
输出目标速度 ω_ref
        │
        ▼
速度环（500Hz~1kHz）
        │
        ▼
输出目标Iq_ref
        │
        ▼
电流环（20kHz）
        │
        ▼
输出Ud/Uq
        │
        ▼
SVPWM
```

因此，**20kHz实时任务里通常只做"最内层"的事情**：

1. 获取最新电流（ADC）
2. 获取最新电角度（Encoder）
3. Clarke → Park
4. Id/Iq电流PID（目标值 `Id_ref`、`Iq_ref` 由外层速度环提供）
5. 反Park
6. SVPWM更新

外层速度环、姿态环、位置环分别以更低频率运行，只负责更新目标值，不参与20kHz实时流水线。这样实时性最好，代码结构也最清晰。
