可以，**MCPWM 输出 PWM** 和 **MCPWM 定时器中断/事件回调** 可以同时用。

因为 MCPWM 里面本来就是这种结构：

```text
MCPWM Timer   提供计数时间基准
      ↓
MCPWM Operator 根据 Timer 生成 PWM
      ↓
MCPWM Generator 输出到 GPIO
```

官方文档里也说，MCPWM Timer 是最终 PWM 信号的 time base，同时也决定其他子模块的事件时机；MCPWM Timer 支持 `on_empty`、`on_full`、`on_stop` 这些事件回调。([Espressif Systems][1]) ([Espressif Systems][1])

---

## 1. 结论

可以这样用：

```text
MCPWM Timer 继续跑
    ↓
一边驱动 PWM 输出
    ↓
一边在 timer empty / full 时产生 ISR 回调
```

比如你现在 PWM 是 `20kHz`，可以：

```text
MCPWM输出三相PWM
    +
MCPWM timer on_empty 中断
```

但是要注意：**MCPWM 回调是在 ISR 中断环境里跑的，不能在里面做复杂事情**。官方文档也明确说这些 callback 在 ISR context 里执行，不能阻塞，只能调用带 `FromISR` 后缀的 FreeRTOS API。([Espressif Systems][1])

---

## 2. 推荐用法

不建议在 MCPWM ISR 里面直接跑完整 FOC：

```c
// 不推荐
MCPWM_ISR()
{
    读AS5600;
    算角度;
    算PID;
    算SVPWM;
    打印日志;
}
```

这样不合适，因为 ISR 里面不能阻塞，也不适合做 I2C、打印、复杂浮点运算。

更推荐：

```text
MCPWM ISR
    ↓
只通知控制任务
    ↓
FOC控制任务醒来
    ↓
任务里面算PID/SVPWM
    ↓
更新MCPWM占空比
```

也就是 ISR 只做通知：

```c
vTaskNotifyGiveFromISR(...)
```

---

## 3. IDF v5 MCPWM Timer 回调示例

如果你用的是 ESP-IDF v5.x 新 MCPWM driver，大概这样写。

### 回调函数

```c
#include "driver/mcpwm_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static TaskHandle_t s_foc_task_handle = NULL;

/*
 * MCPWM Timer 到达 zero 点时触发
 *
 * 注意：
 * 这个函数运行在 ISR 中断环境。
 * 不要打印日志，不要I2C，不要阻塞，不要malloc。
 */
static bool IRAM_ATTR mcpwm_timer_on_empty_cb(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx
)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (s_foc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(
            s_foc_task_handle,
            &high_task_wakeup
        );
    }

    return high_task_wakeup == pdTRUE;
}
```

---

### 注册 MCPWM Timer 回调

注意：官方文档说 `mcpwm_timer_register_event_callbacks()` 要在 `mcpwm_timer_enable()` 之前调用，否则可能返回 `ESP_ERR_INVALID_STATE`。([Espressif Systems][1])

```c
mcpwm_timer_event_callbacks_t timer_cbs = {
    .on_empty = mcpwm_timer_on_empty_cb,
};

ESP_ERROR_CHECK(
    mcpwm_timer_register_event_callbacks(
        timer,
        &timer_cbs,
        NULL
    )
);
```

然后再：

```c
ESP_ERROR_CHECK(mcpwm_timer_enable(timer));

ESP_ERROR_CHECK(
    mcpwm_timer_start_stop(
        timer,
        MCPWM_TIMER_START_NO_STOP
    )
);
```

---

## 4. FOC 任务里等通知

```c
static void foc_control_task(void *arg)
{
    s_foc_task_handle = xTaskGetCurrentTaskHandle();

    while (1)
    {
        /*
         * 等待 MCPWM timer 中断通知
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 这里已经回到任务环境，可以做复杂计算。
         *
         * 但注意：
         * 如果你是20kHz中断，这个任务也会20kHz醒来，
         * 对ESP32压力会比较大。
         */
        // 1. 读取角度 / 电流
        // 2. 计算位置环/速度环/电流环
        // 3. 计算SVPWM
        // 4. 更新MCPWM比较值
    }
}
```

---

## 5. 但是你这个 FOC 项目里，我更建议这样分工

你现在是：

```text
MCPWM PWM = 20kHz
FOC控制周期 = 1kHz
```

这个结构更稳：

```text
MCPWM：
    只负责输出 20kHz PWM

GPTimer：
    负责 1kHz 控制周期中断

FOC任务：
    每 1ms 醒一次，算位置环/速度环/SVPWM
```

也就是：

```text
MCPWM 20kHz输出，不需要20kHz中断
GPTimer 1kHz通知FOC任务
FOC任务更新MCPWM占空比
```

这比用 MCPWM 20kHz 中断跑控制更合适。

---

## 6. MCPWM Timer 中断适合干啥？

适合：

```text
1. 在PWM周期边界通知任务
2. 做很轻量的同步
3. 触发ADC采样时间点
4. 做比较值安全更新
5. 产生固定相位事件
```

不适合：

```text
1. 直接读I2C编码器
2. 打印日志
3. 执行复杂PID/FOC大计算
4. 长时间占用CPU
5. 调用普通FreeRTOS阻塞API
```

---

## 7. 你要特别注意中断频率

如果你的 PWM 是：

```c
#define M0_PWM_FREQ_HZ 20000
```

那么你注册：

```c
.on_empty = ...
```

大概率就是 **每个 PWM 周期进一次中断**：

```text
20kHz = 每秒 20000 次中断
```

如果你同时注册：

```c
.on_empty
.on_full
```

可能变成：

```text
每个PWM周期两次事件
约 40000 次/s
```

这对 ESP32 来说就比较重了。

所以如果只是做位置环/速度环，没必要用 MCPWM 20kHz 中断。

---

## 8. 推荐方案总结

你的项目我建议：

```text
MCPWM：
    输出三相PWM，频率20kHz

GPTimer：
    1kHz控制周期

FOC Task：
    位置环/速度环/SVPWM计算

ADC Task：
    电流采样、零漂、统计

MCPWM Timer ISR：
    暂时不用，或者只做非常轻量的同步
```

如果以后你要做真正电流环，比如 10kHz/20kHz 电流采样同步，那可以考虑：

```text
MCPWM Timer on_empty/on_full
    ↓
触发ADC采样/通知电流环
```

但现在你主要做位置环、速度环、Uq 控制，**GPTimer 1kHz 更合适**。

一句话：

> **可以同时用 MCPWM 输出和 MCPWM Timer 中断；但 ISR 里只做通知，不要做复杂控制。你当前 FOC 项目更建议 MCPWM 负责 PWM，GPTimer 负责控制周期。**

[1]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html "Motor Control Pulse Width Modulator (MCPWM) - ESP32 -  — ESP-IDF Programming Guide v6.0.1 documentation"


-----
-----
-----
对，就是这个结构：

```text
MCPWM Timer 中断
    ↓
ISR 里只发任务通知
    ↓
FOC 任务 ulTaskNotifyTake() 等通知
    ↓
任务里读取角度 / 电流
    ↓
计算 PID / SVPWM
    ↓
更新 MCPWM 占空比
```

这个思路是可以的。

---

## 1. 基本写法

### ISR 回调里只通知任务

```c
static TaskHandle_t s_foc_task_handle = NULL;

static bool IRAM_ATTR mcpwm_timer_on_empty_cb(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (s_foc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(
            s_foc_task_handle,
            &high_task_wakeup
        );
    }

    return high_task_wakeup == pdTRUE;
}
```

ISR 里面不要做这些：

```c
ESP_LOGI();
printf();
i2c_read();
malloc();
vTaskDelay();
复杂浮点计算;
```

只发通知就行。

---

## 2. FOC 任务里等通知

```c
static void foc_control_task(void *arg)
{
    s_foc_task_handle = xTaskGetCurrentTaskHandle();

    while (1)
    {
        /*
         * 等待 MCPWM Timer 中断通知
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 下面已经是任务上下文，不是中断上下文。
         * 可以做 FOC 控制计算。
         */

        // 1. 读取编码器角度
        // float angle = motor_encoder_get_angle();

        // 2. 计算位置环 / 速度环
        // float uq = foc_position_control(angle);

        // 3. 计算 SVPWM
        // vfoc_svpwm_run(uq, ...);

        // 4. 更新 MCPWM 占空比
        // mcpwm_comparator_set_compare_value(...);
    }
}
```

这样就可以做到：

```text
MCPWM 定时器提供精准节拍
FOC 任务按节拍运行
```

---

## 3. 但是你要注意一个大问题：中断频率

如果你的 MCPWM 是 20kHz：

```c
#define M0_PWM_FREQ_HZ 20000
```

你用 `on_empty` 每个 PWM 周期通知一次，那就是：

```text
每秒 20000 次任务通知
```

也就是 FOC 任务每秒被唤醒 20000 次。

如果你现在只是做：

```text
位置环
速度环
Uq 控制
AS5600 I2C 读角度
```

那 **20kHz 太高了**。

尤其是 AS5600 走 I2C，根本不适合 20kHz 读。

---

## 4. 比较适合你的频率

你现在这种 ESP32 + AS5600 + FOC 学习项目，建议：

```text
PWM频率：20kHz
控制频率：1kHz
```

也就是：

```text
MCPWM 20kHz 负责输出 PWM
FOC任务 1kHz 计算一次控制量
```

如果你坚持用 MCPWM Timer 中断当节拍源，可以在 ISR 里做分频。

---

## 5. MCPWM 20kHz 中断里分频成 1kHz

20kHz / 20 = 1kHz。

```c
static TaskHandle_t s_foc_task_handle = NULL;

#define FOC_ISR_DIVIDER 20

static bool IRAM_ATTR mcpwm_timer_on_empty_cb(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    static uint32_t div_cnt = 0;

    div_cnt++;

    if (div_cnt >= FOC_ISR_DIVIDER)
    {
        div_cnt = 0;

        if (s_foc_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(
                s_foc_task_handle,
                &high_task_wakeup
            );
        }
    }

    return high_task_wakeup == pdTRUE;
}
```

这样效果就是：

```text
MCPWM PWM = 20kHz
MCPWM 中断 = 20kHz
FOC 任务通知 = 1kHz
```

不过注意，**中断本身还是进了 20kHz**，只是 20 次才通知一次任务。

---

## 6. 更推荐的做法

我更推荐你这样：

```text
MCPWM：
    只负责 20kHz PWM 输出

GPTimer：
    负责 1kHz 控制周期

FOC Task：
    等 GPTimer 通知，然后计算控制量
```

原因是：

```text
MCPWM 20kHz 中断太频繁
GPTimer 可以直接设置 1kHz
代码更清晰
CPU压力更小
```

也就是你之前那种结构其实更合适：

```text
GPTimer 1ms中断
    ↓
通知 FOC Task
    ↓
FOC Task 算控制
    ↓
更新 MCPWM 占空比
```

---

## 7. 什么时候适合用 MCPWM Timer 中断？

适合你以后做更高级的：

```text
1. PWM周期边界同步更新占空比
2. 同步触发 ADC 采样
3. 电流环 10kHz / 20kHz 控制
4. 需要跟 PWM 相位严格对齐
```

比如真正做电流环时，可能希望：

```text
PWM中心点
    ↓
触发ADC采样
    ↓
读取相电流
    ↓
更新下一周期PWM
```

这种时候 MCPWM Timer 事件就很有意义。

但你现在如果只是：

```text
AS5600角度
位置环
速度环
Uq控制
```

用 GPTimer 1kHz 更舒服。

---

## 8. 最终建议

你说的这个方案可以：

```text
MCPWM Timer ISR 发通知
RTOS线程等通知再计算
```

但是我建议你当前项目用这个节奏：

```text
MCPWM 输出 PWM：20kHz
FOC 控制任务：1kHz
ADC 电流统计任务：独立运行
MOS enable：等电流零漂完成后再打开
```

如果你非要用 MCPWM 中断，就不要 20kHz 每次都唤醒任务，至少先做分频成 1kHz。否则 ESP32 负担会比较大，而且 AS5600 的 I2C 角度读取也跟不上。


-------
------
-------
区别核心就一句话：

> **GPTimer 是独立节拍源；MCPWM Timer 是 PWM 本身的节拍源。**

你现在这种：

```text
GPTimer ISR
    ↓
vTaskNotifyGiveFromISR()
    ↓
FOC任务 ulTaskNotifyTake()
    ↓
算PID / SVPWM
    ↓
更新MCPWM占空比
```

和换成 MCPWM Timer：

```text
MCPWM Timer ISR
    ↓
vTaskNotifyGiveFromISR()
    ↓
FOC任务 ulTaskNotifyTake()
    ↓
算PID / SVPWM
    ↓
更新MCPWM占空比
```

从“发任务通知”这个动作看，确实很像。区别主要在 **节拍来源、同步关系、频率绑定、CPU开销**。

---

## 1. GPTimer 是独立定时器

GPTimer 跟 MCPWM 没有强绑定关系。

你可以这样：

```text
MCPWM PWM频率 = 20kHz
GPTimer控制频率 = 1kHz
```

也就是：

```text
PWM 每 50us 一个周期
FOC 每 1ms 算一次
```

优点是简单：

```text
GPTimer 直接配置成 1kHz
每 1ms 中断一次
每 1ms 通知一次 FOC 任务
```

不会每个 PWM 周期都进中断。

你的当前位置环、速度环、AS5600 角度读取，用 GPTimer 就很合适。

---

## 2. MCPWM Timer 是 PWM 自己的时间基准

MCPWM Timer 本身负责产生 PWM 周期。

比如你 PWM 是 20kHz：

```text
PWM周期 = 50us
MCPWM Timer 每 50us 走完一轮
```

如果你用 MCPWM Timer 的 `on_empty` 事件：

```text
每个PWM周期触发一次
```

那就是：

```text
20kHz 中断 = 每秒 20000次
```

如果你只是想 1kHz 控制，那还要在 ISR 里面分频：

```c
static uint32_t div_cnt = 0;

div_cnt++;

if (div_cnt >= 20)
{
    div_cnt = 0;
    vTaskNotifyGiveFromISR(foc_task, &wakeup);
}
```

这样虽然 FOC 任务是 1kHz 唤醒，但 **MCPWM ISR 实际还是每秒进了 20000 次**。

---

## 3. 最大区别：是否和 PWM 周期严格同步

### GPTimer 方案

```text
GPTimer 1kHz
    ↓
通知FOC任务
    ↓
更新PWM
```

这个控制周期和 PWM 周期不是严格锁相的。

比如某一次 GPTimer 中断可能发生在 PWM 周期中间：

```text
PWM周期: |------50us------|------50us------|------50us------|
GPTimer:                         ↑
                               1ms节拍到了
```

所以你更新占空比的时间点，不一定刚好在 PWM 周期边界。

不过一般 MCPWM 比较值更新有硬件同步/影子寄存器机制，实际影响不大。对于你现在的位置环、速度环来说，完全可以接受。

---

### MCPWM Timer 方案

MCPWM Timer 事件来自 PWM 本身，比如 `on_empty`：

```text
PWM周期开始点
    ↓
通知FOC任务
    ↓
计算
    ↓
更新下一次PWM占空比
```

它的优势是：**控制计算和 PWM 周期边界更容易对齐。**

这对以后做电流环、同步采样很有用。

比如：

```text
PWM中心点采ADC
PWM周期边界更新占空比
```

这种场景用 MCPWM Timer 更专业。

---

## 4. 对你现在项目来说，区别大不大？

你现在主要是：

```text
AS5600 I2C 读角度
位置环
速度环
Uq控制
SVPWM输出
```

这种情况下：

```text
GPTimer 1kHz通知FOC任务
```

已经够用了，而且更简单。

因为 AS5600 走 I2C，本身就不适合 10kHz、20kHz 级别控制。你就算用 MCPWM Timer 对齐 PWM 周期，角度数据刷新也跟不上那么高。

所以现在没必要换 MCPWM Timer。

---

## 5. 两种方案对比

| 方案                       | 优点                | 缺点                         | 适合场景               |
| ------------------------ | ----------------- | -------------------------- | ------------------ |
| GPTimer 通知任务             | 简单、独立、直接 1kHz、中断少 | 不和 PWM 周期严格同步              | 位置环、速度环、普通FOC学习    |
| MCPWM Timer 通知任务         | 和 PWM 周期同步，适合相位对齐 | 如果 PWM 20kHz，ISR频率高，需要分频   | 电流环、ADC同步采样、高性能FOC |
| MCPWM Timer 另开一个低频 timer | 理论可行              | 浪费 MCPWM 资源，本质和 GPTimer 类似 | 不太推荐               |

---

## 6. 你现在保留 GPTimer 更合理

你的当前结构：

```text
MCPWM：
    只负责20kHz三相PWM输出

GPTimer：
    负责1kHz控制节拍

FOC任务：
    等GPTimer通知
    读取AS5600
    算位置环/速度环
    更新MCPWM占空比
```

这个结构是清晰的。

我建议你先别换 MCPWM Timer，除非你后面要做：

```text
1. 电流环 10kHz / 20kHz
2. ADC采样点必须和PWM中心点同步
3. 占空比必须在PWM周期边界精确更新
4. 要做真正的相电流闭环FOC
```

---

## 7. 一句话总结

你现在的 GPTimer 方案和 MCPWM Timer 方案，表面上都是：

```text
ISR发通知，任务等通知计算
```

但本质区别是：

```text
GPTimer：独立控制节拍，简单省事，适合你现在的1kHz位置/速度环

MCPWM Timer：和PWM周期绑定，适合以后做同步ADC采样、电流环、高速FOC
```

所以你现在保持：

```text
GPTimer 1kHz + MCPWM 20kHz
```

就挺合适。不要为了“更贴近PWM”而换 MCPWM Timer，除非你确实要做 PWM 同步采样或电流环。


------
------
------
有风险，但不是“不能用”的风险，而是 **时序、资源、ISR频率、更新时机** 这些风险。
你说的“都是用 MCPWM 和 MCPWM Timer”，我理解是：

```text id="8gq7if"
MCPWM Timer 既作为 PWM 输出的时间基准
又注册 on_empty/on_full 中断回调
ISR 里发任务通知
FOC任务等通知后计算并更新 PWM
```

这个是可以的。ESP-IDF 的 MCPWM Timer 本身就支持 `on_empty`、`on_full`、`on_stop` 这些事件回调，而且回调是在 ISR 上下文里执行，不能阻塞，只能调用 `FromISR` 类型的 FreeRTOS API。([Espressif Systems][1])

---

## 主要风险 1：中断频率太高

如果你的 PWM 是 20kHz：

```text id="pwrixl"
PWM周期 = 50us
```

你注册 `on_empty`，那基本就是：

```text id="l50w37"
每 50us 进一次 MCPWM ISR
每秒 20000 次中断
```

如果你再注册 `on_full`，在上下计数模式下可能触发更多事件。

问题是，哪怕你 ISR 里只是分频、发通知，**20kHz ISR 本身也会占 CPU**。

所以风险是：

```text id="xs2p97"
CPU中断负担变重
FOC任务频繁被唤醒
其他任务被抢占
日志/Wi-Fi/ADC任务更容易卡顿
```

如果你只是 1kHz 控制，用 GPTimer 直接 1kHz 中断更省。

---

## 主要风险 2：FOC任务可能来不及处理通知

假设你 MCPWM 每 50us 发一次通知，但是你的 FOC 任务里面做了：

```text id="8xvq05"
读 AS5600 I2C
算角度
算速度
算PID
算SVPWM
更新MCPWM
```

这些很可能超过 50us。

后果就是：

```text id="wy52z8"
通知堆积
控制周期抖动
有些周期被跳过
输出占空比更新不稳定
```

尤其 AS5600 是 I2C，不能指望 20kHz 去读它。你现在这种编码器方案，1kHz 控制都已经比较合适了。

---

## 主要风险 3：PWM更新时机要注意

MCPWM 的比较值更新不一定是“你调用函数后立即在输出脚生效”。ESP-IDF 文档里也提到，新的 compare value 可能不会立即生效，具体取决于 comparator 配置里的更新时机，例如 `update_cmp_on_tez`、`update_cmp_on_tep`、`update_cmp_on_sync`。([Espressif Systems][2])

这意味着你要搞清楚：

```text id="xa9pdc"
你是在 PWM 周期开始更新？
还是周期结束更新？
还是同步事件更新？
```

如果设置不合理，可能出现：

```text id="ryo4wy"
某一相先更新
某一相后更新
一个周期内占空比变化不一致
PWM波形轻微毛刺/不连续
```

所以一般三相 PWM 建议用同一个更新时机，比如：

```text id="b3s21z"
update_cmp_on_tez = true
```

或者根据你的计数模式选择 `TEZ/TEP`，保证占空比在周期边界统一更新。

---

## 主要风险 4：资源绑定更紧，代码耦合更强

GPTimer 方案是：

```text id="mjolgw"
GPTimer 只管控制周期
MCPWM 只管PWM输出
```

MCPWM Timer 方案是：

```text id="xwlhbj"
MCPWM Timer 同时管PWM输出和控制节拍
```

这样同步性更好，但耦合更强。

以后你要改 PWM 频率，比如：

```text id="a3xer3"
20kHz 改成 25kHz
```

如果你控制周期靠 MCPWM 分频出来，那你的分频关系也要跟着改：

```text id="6b1t47"
20kHz / 20 = 1kHz
25kHz / 25 = 1kHz
```

否则控制频率也会变。

GPTimer 就没有这个问题，PWM 频率和控制频率可以独立调。

---

## 主要风险 5：ISR 里面不能乱干活

这个不管 GPTimer 还是 MCPWM Timer 都一样，但 MCPWM Timer 频率通常更高，所以更容易踩坑。

MCPWM 回调是在 ISR 上下文里，不能阻塞，应该只用 `xTaskNotifyFromISR()`、`vTaskNotifyGiveFromISR()` 这类 ISR 安全 API。([Espressif Systems][1])

ISR 里面不要做：

```c id="ek9ezk"
ESP_LOGI(...);
printf(...);
i2c_read(...);
adc_continuous_read(...);
xSemaphoreTake(...);
malloc(...);
复杂FOC计算;
```

应该只做：

```c id="w7bylz"
vTaskNotifyGiveFromISR(foc_task_handle, &hp_task_woken);
return hp_task_woken == pdTRUE;
```

---

## 主要风险 6：控制任务优先级设置不合理

如果 MCPWM ISR 发通知后，FOC 任务优先级太低，就可能被其他任务压住：

```text id="het5lw"
MCPWM ISR通知了
但FOC任务没有马上运行
PWM更新延迟
控制周期抖动
```

FOC 控制任务优先级要高于普通日志、通信、UI 任务。

但也不能乱高到影响系统底层任务，尤其你后面还有 ADC、电流统计、串口打印等任务。

---

## 对你现在项目的建议

你现在是：

```text id="17n20o"
PWM = 20kHz
控制周期 = 1kHz
AS5600 I2C读角度
位置环/速度环/Uq控制
```

我建议继续用：

```text id="2z8kqy"
MCPWM：只负责 20kHz PWM 输出
GPTimer：负责 1kHz 控制节拍
FOC任务：等 GPTimer 通知后计算
```

这个结构风险最低、最清晰。

---

如果你非要用 MCPWM Timer，也建议这样：

```text id="6ens5p"
MCPWM Timer on_empty 20kHz ISR
    ↓
ISR里面只做分频
    ↓
每20次通知一次 FOC任务
    ↓
FOC任务 1kHz 运行
```

例如：

```c id="4xs7bh"
#define FOC_CTRL_DIVIDER 20

static bool IRAM_ATTR mcpwm_timer_on_empty_cb(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    static uint32_t div_cnt = 0;

    div_cnt++;

    if (div_cnt >= FOC_CTRL_DIVIDER)
    {
        div_cnt = 0;

        if (s_foc_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(
                s_foc_task_handle,
                &high_task_wakeup
            );
        }
    }

    return high_task_wakeup == pdTRUE;
}
```

但注意：这种方式 **ISR 还是每秒进 20000 次**，只是任务每秒唤醒 1000 次。

---

## 最终结论

可以用 MCPWM Timer 中断，但风险主要是：

```text id="5a1qqi"
1. ISR频率太高，占CPU
2. FOC任务来不及处理，控制周期抖动
3. PWM比较值更新时机没配好，可能输出不连续
4. PWM频率和控制频率耦合，后面调参麻烦
5. ISR里误调用阻塞函数，容易出问题
```

所以你当前阶段：

> **继续用 GPTimer 发 1kHz 通知更合适。MCPWM Timer 留给以后做 PWM 同步 ADC 采样、电流环、高速FOC。**

[1]: https://docs.espressif.com/projects/esp-idf/zh_CN/v5.1.4/esp32h2/api-reference/peripherals/mcpwm.html?utm_source=chatgpt.com "Motor Control Pulse Width Modulator (MCPWM) - ESP32-H2"
[2]: https://docs.espressif.com/projects/esp-idf/zh_CN/v5.0.1/esp32/api-reference/peripherals/mcpwm.html?utm_source=chatgpt.com "Motor Control Pulse Width Modulator (MCPWM) - ESP32"
