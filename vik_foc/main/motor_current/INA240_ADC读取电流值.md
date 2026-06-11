你这里理解反了：**INA240不是测电机相线电压的，而是测电机相电流的。**

## 你这张原理图测的是什么

以电机 M0 为例：

```text
逆变桥输出                    电机相线
M0_OUT1_P ─── R38 10mΩ ─── M0_OUT1
                ↑     ↑
               IN+   IN-
                INA240
                  │
               M0_CS1 → MCU ADC
```

R38、R40 是串联在两根电机相线上的采样电阻。

INA240检测采样电阻两端的微小压差：

$$
V_{SHUNT}=V_{IN+}-V_{IN-}=I_{PHASE}\times R_{SHUNT}
$$

然后放大后输出给 MCU：

$$
V_{OUT}=V_{REF}+G\times I_{PHASE}\times R_{SHUNT}
$$

因此：

$$
I_{PHASE}=\frac{V_{OUT}-V_{REF}}
{G\times R_{SHUNT}}
$$

假设：

```text
RSHUNT = 10mΩ = 0.01Ω
INA240A1 增益 G = 20
VREF = 1.65V
```

那么：

```text
输出灵敏度 = 20 × 0.01
           = 0.2V/A
```

也就是：

```text
0A   → 1.65V
+1A  → 1.85V
+5A  → 2.65V
-1A  → 1.45V
-5A  → 0.65V
```

所以 `M0_CS1`、`M0_CS2` 是两路相电流采样信号，不是相电压。

---

## 为什么三相电机只测两相电流

三相电流满足：

$$
I_U+I_V+I_W=0
$$

所以只需要检测两相：

```text
IU = INA240通道1测量
IV = INA240通道2测量
IW = -(IU + IV)
```

你图中的：

```text
R38 + INA240 → 测一相电流
R40 + INA240 → 测另一相电流
第三相电流   → 软件计算
```

这属于**双电阻相电流采样**。

---

# 那电机相线电压怎么测

需要另外增加电阻分压电路，不能直接使用 INA240。

三相逆变桥的相线瞬时电压通常在：

```text
0V ～ VBUS
```

之间高速PWM跳变。例如 VBUS=40V，相线相对直流母线负极会不断在接近0V和40V之间切换。

可以给每一相增加：

```text
电机U相 ── 高阻值电阻 ──┬── MCU_ADC_U
                        │
                      低阻值电阻
                        │
                       GND
```

计算公式：

$$
V_{ADC}=V_{PHASE}\times
\frac{R_{DOWN}}{R_{UP}+R_{DOWN}}
$$

例如40V母线、3.3V ADC，可以考虑留出较大的尖峰余量：

```text
RUP   = 220kΩ
RDOWN = 15kΩ
```

40V时：

$$
V_{ADC}=40\times\frac{15}{220+15}
\approx2.55V
$$

建议电路：

```text
电机相线
   │
  220kΩ
   │
   ├── 1kΩ ── MCU ADC
   │
  15kΩ       1nF～10nF
   │            │
  GND          GND
```

实际设计还应该加：

* ADC输入钳位保护；
* RC低通滤波；
* 足够耐压的分压电阻；
* PWM同步采样；
* 对电机反电动势和开关尖峰留余量。

不要让40V相线直接连接MCU ADC。

---

## 相电压有三种不同概念

必须先区分你想测哪一种。

### 1. 相线对母线负极电压

例如：

```text
VU_GND
VV_GND
VW_GND
```

最容易测，每相使用电阻分压接ADC。

瞬时波形是PWM方波：

```text
0V ↔ VBUS
```

### 2. 两根相线之间的线电压

例如：

$$
V_{UV}=V_U-V_V
$$

可以分别采样 U、V 两相对地电压，然后软件相减：

```c
v_uv = v_u - v_v;
v_vw = v_v - v_w;
v_wu = v_w - v_u;
```

不要简单把普通ADC直接跨接在两根相线上。

### 3. 电机真正的相对中性点电压

星形电机通常没有引出中性点，可以通过三个相线电压计算虚拟中性点：

$$
V_N=\frac{V_U+V_V+V_W}{3}
$$

然后：

$$
V_{UN}=V_U-V_N
$$

$$
V_{VN}=V_V-V_N
$$

$$
V_{WN}=V_W-V_N
$$

---

## 做FOC时通常不需要实际测量相电压

一般FOC控制器只采集：

```text
两相电流 IU、IV
母线电压 VBUS
转子角度
```

三相输出电压可以根据 `Ud、Uq`、SVPWM占空比和母线电压推算。

例如桥臂相对母线负极的平均电压近似为：

$$
V_U=D_U\times V_{BUS}
$$

$$
V_V=D_V\times V_{BUS}
$$

$$
V_W=D_W\times V_{BUS}
$$

其中 `DU、DV、DW` 是三个PWM占空比。

所以你的FOC硬件一般应当是：

```text
INA240 × 2       → 两相电流采样
母线电阻分压     → VBUS电压采样
AS5600/编码器    → 电角度
PWM占空比        → 推算输出相电压
```

## 你图里的结论

```text
R38、R40：10mΩ相电流采样电阻
M0_OUT1_P、M0_OUT1：第一相采样电阻两端
M0_OUT2_P、M0_OUT2：第二相采样电阻两端
M0_CS1、M0_CS2：INA240放大后的电流采样电压
```

因此，这套电路只能检测**两相电流**。想检测三相线电压，需要另外增加三路高压电阻分压采样电路。

--------
-----------
--------
你用的是 **INA240A2，增益是 50 V/V**，所以前面的计算要改成下面这样。

你的采样电阻是：

$$
R_{shunt}=10m\Omega=0.01\Omega
$$

INA240A2增益：

$$
G=50
$$

因此输出与电流的关系是：

$$
V_{OUT}=V_{REF}+I_{phase}\times R_{shunt}\times G
$$

代入参数：

$$
V_{OUT}=V_{REF}+I_{phase}\times0.01\times50
$$

即：

$$
V_{OUT}=V_{REF}+0.5I_{phase}
$$

所以你的电流采样灵敏度是：

$$
\boxed{0.5V/A}
$$

也就是相电流每变化 `1A`，INA240输出变化 `0.5V`。

## 双向相电流测量

电机相电流有正有负，所以通常把参考电压设置在 ADC 电源中点。

假设：

```text
INA240供电：3.3V
VREF：1.65V
```

则输出关系为：

| 相电流 | INA240输出 |
| --: | -------: |
| -3A |    0.15V |
| -2A |    0.65V |
| -1A |    1.15V |
|  0A |    1.65V |
| +1A |    2.15V |
| +2A |    2.65V |
| +3A |    3.15V |

计算相电流：

$$
\boxed{I_{phase}=\frac{V_{OUT}-V_{REF}}{0.5}}
$$

也可以写成：

$$
\boxed{I_{phase}=2(V_{OUT}-V_{REF})}
$$

例如 MCU ADC 测得：

```text
VOUT = 2.35V
VREF = 1.65V
```

则：

$$
I=\frac{2.35-1.65}{0.5}=1.4A
$$

如果测得：

```text
VOUT = 0.90V
```

则：

$$
I=\frac{0.90-1.65}{0.5}=-1.5A
$$

负号表示电流方向与 INA240 的 `IN+ → IN-` 定义方向相反。

## 你的理论测量范围

如果 ADC 和 INA240 都工作在 3.3V，参考电压是 1.65V，那么理想双向测量范围为：

$$
I_{max}=\frac{3.3-1.65}{0.5}=3.3A
$$

所以理想范围大约是：

$$
\boxed{-3.3A\sim+3.3A}
$$

但实际不能让 INA240 输出完全贴近 0V 和 3.3V，还需要给输出摆幅、采样噪声和电机尖峰留余量。

比较实用的测量范围建议为：

$$
\boxed{-3A\sim+3A}
$$

如果电机相电流可能超过 3A，INA240A2 配合 `10mΩ` 就容易饱和。

例如 5A 电流：

$$
\Delta V=5\times0.01\times50=2.5V
$$

输出将是：

```text
正向：1.65 + 2.5 = 4.15V
反向：1.65 - 2.5 = -0.85V
```

显然已经超过 3.3V供电允许范围，无法正常测量。

## ESP32 12位ADC换算示例

假设 ADC 满量程按 3.3V、12位计算：

$$
V_{OUT}=\frac{ADC_{raw}}{4095}\times3.3
$$

相电流为：

$$
I_{phase}=
\frac{\frac{ADC_{raw}}{4095}\times3.3-V_{zero}}
{50\times0.01}
$$

代码可以这样写：

```c
/**
 * @brief INA240A2 ADC值转换为相电流
 *
 * @param adc_raw      ADC原始值，12位范围0~4095
 * @param zero_voltage 实际零电流输出电压，建议启动时校准
 *
 * @return float 相电流，单位A
 */
float ina240a2_adc_to_current(uint16_t adc_raw, float zero_voltage)
{
    const float adc_vref = 3.3f;          /* ADC参考电压 */
    const float adc_max = 4095.0f;        /* 12位ADC最大值 */
    const float ina_gain = 50.0f;         /* INA240A2增益 */
    const float shunt_resistor = 0.01f;   /* 采样电阻10mΩ */

    /* 将ADC原始值转换成电压 */
    float output_voltage =
        ((float)adc_raw / adc_max) * adc_vref;

    /* 根据分流电阻两端压差计算相电流 */
    float phase_current =
        (output_voltage - zero_voltage) /
        (ina_gain * shunt_resistor);

    return phase_current;
}
```

这里不要直接固定认为零电流一定是 `1.650V`。实际会受到以下因素影响：

* INA240自身失调电压；
* 1.65V参考电压误差；
* ADC参考电压误差；
* PCB走线压降；
* 采样电阻误差；
* PWM开关噪声。

更推荐在电机不输出PWM、确定相电流为0A时，采集多次 ADC 求平均，得到 `zero_voltage` 或 `zero_adc_raw`。

## 你的硬件搭配总结

```text
INA240A2：增益50
采样电阻：10mΩ
电流灵敏度：0.5V/A
零电流中心：建议约1.65V
理论双向范围：约±3.3A
建议有效范围：约±3A
```

因此，如果你的电机峰值相电流可能达到 `5A、10A`，就需要减小采样电阻，例如：

| 采样电阻 | INA240A2灵敏度 | 约±1.5V输出范围对应电流 |
| ---: | ----------: | -------------: |
| 10mΩ |      0.5V/A |            ±3A |
|  5mΩ |     0.25V/A |            ±6A |
|  2mΩ |      0.1V/A |           ±15A |

但采样电阻减小后，低电流分辨率也会降低。


-------
-------
-------


从你的 ESP32 原理图看，4 路 INA240 输出接线是：

| 电流采样信号   |           ESP32 引脚 |    ADC通道 |
| -------- | -----------------: | -------: |
| `M0_CS2` | SENSOR_VP / GPIO36 | ADC1_CH0 |
| `M0_CS1` | SENSOR_VN / GPIO39 | ADC1_CH3 |
| `M1_CS2` |             GPIO34 | ADC1_CH6 |
| `M1_CS1` |             GPIO35 | ADC1_CH7 |

ESP32 的 ADC1 通道对应关系确实是：CH0→GPIO36、CH3→GPIO39、CH6→GPIO34、CH7→GPIO35。([Espressif Systems][1])

所以你当前例子：

```c
static adc_channel_t channel[2] = {
    ADC_CHANNEL_6,
    ADC_CHANNEL_7
};
```

实际上只读取了：

```text
ADC1_CH6 → GPIO34 → M1_CS2
ADC1_CH7 → GPIO35 → M1_CS1
```

也就是只读取了 M1 电机的两相电流。

---

# 一、最简单的修改

改成4路：

```c
static adc_channel_t channel[4] = {
    ADC_CHANNEL_0,     /* GPIO36，M0_CS2 */
    ADC_CHANNEL_3,     /* GPIO39，M0_CS1 */
    ADC_CHANNEL_6,     /* GPIO34，M1_CS2 */
    ADC_CHANNEL_7,     /* GPIO35，M1_CS1 */
};
```

这样你的初始化代码基本不需要再改：

```c
continuous_adc_init(channel,
                    sizeof(channel) / sizeof(channel[0]),
                    &adc_handle);
```

但是只打印每一个原始采样值，会产生大量日志，而且无法直接看到电流，所以更推荐下面这个完整版本。

---

# 二、完整读取及电流换算代码

下面代码按照你贴出来的 `adc_continuous_parse_data()` 接口编写，包含：

* 4路ADC连续采样；
* 每路采样平均；
* 上电零电流偏置校准；
* ADC校准后的毫伏值；
* INA240A2电流换算；
* 每100ms打印一次；
* 全部中文注释。

```c
/*
 * ESP32 + INA240A2 双电机四路相电流采样
 *
 * M0_CS2 -> GPIO36 -> ADC1_CH0
 * M0_CS1 -> GPIO39 -> ADC1_CH3
 * M1_CS2 -> GPIO34 -> ADC1_CH6
 * M1_CS1 -> GPIO35 -> ADC1_CH7
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/*----------------------------------------------------------
 * ADC基本配置
 *----------------------------------------------------------*/

/* 使用ADC1 */
#define CURRENT_ADC_UNIT                    ADC_UNIT_1

/* ESP32只使用ADC1 */
#define CURRENT_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1

/* 使用较大衰减，允许测量接近3.3V的电压 */
#define CURRENT_ADC_ATTEN                   ADC_ATTEN_DB_12

/* 使用芯片支持的最大ADC位宽，ESP32通常为12位 */
#define CURRENT_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

/* 一共有4路电流采样 */
#define CURRENT_ADC_CHANNEL_NUM             4

/*
 * ADC总采样率。
 *
 * ADC会依次轮询4个通道，因此：
 *
 * 80000次/秒 ÷ 4路 = 每路约20000次/秒
 */
#define CURRENT_ADC_TOTAL_SAMPLE_HZ         (80 * 1000)

/* 每次从DMA读取的字节数 */
#define CURRENT_ADC_READ_LEN                256

/* ADC驱动内部缓冲区大小 */
#define CURRENT_ADC_STORE_BUFFER_SIZE       2048

/* 每路零点校准采样次数 */
#define CURRENT_ZERO_SAMPLE_NUM             2000

/* 每隔100ms打印一次 */
#define CURRENT_REPORT_PERIOD_US            (100 * 1000)

/* ESP32 ADC1总共有8个通道：0～7 */
#define ESP32_ADC1_CHANNEL_MAX              8

/*----------------------------------------------------------
 * INA240A2配置
 *----------------------------------------------------------*/

/* INA240A2增益为50V/V */
#define INA240_GAIN                         50.0f

/* 电流采样电阻为10mΩ */
#define CURRENT_SHUNT_RESISTOR_OHM          0.010f

/*
 * 输出灵敏度：
 *
 * 50 × 0.01Ω = 0.5V/A
 *
 * 即：
 * 500mV/A
 */
#define INA240_MV_PER_AMP                   \
    (INA240_GAIN * CURRENT_SHUNT_RESISTOR_OHM * 1000.0f)

/*----------------------------------------------------------
 * 四路ADC通道
 *----------------------------------------------------------*/

static const adc_channel_t current_adc_channels[CURRENT_ADC_CHANNEL_NUM] = {
    ADC_CHANNEL_0,     /* GPIO36：M0_CS2 */
    ADC_CHANNEL_3,     /* GPIO39：M0_CS1 */
    ADC_CHANNEL_6,     /* GPIO34：M1_CS2 */
    ADC_CHANNEL_7,     /* GPIO35：M1_CS1 */
};

/* ADC中断通知的任务句柄 */
static TaskHandle_t current_adc_task_handle = NULL;

/* ADC校准句柄 */
static adc_cali_handle_t current_adc_cali_handle = NULL;

/* ADC校准功能是否成功初始化 */
static bool current_adc_cali_enable = false;

/* 日志TAG */
static const char *TAG = "CURRENT_ADC";

/*----------------------------------------------------------
 * 获取ADC通道对应的信号名称
 *----------------------------------------------------------*/

static const char *current_adc_get_name(adc_channel_t channel)
{
    switch (channel) {
        case ADC_CHANNEL_0:
            return "M0_CS2";

        case ADC_CHANNEL_3:
            return "M0_CS1";

        case ADC_CHANNEL_6:
            return "M1_CS2";

        case ADC_CHANNEL_7:
            return "M1_CS1";

        default:
            return "UNKNOWN";
    }
}

/*----------------------------------------------------------
 * ADC原始值转换成毫伏
 *----------------------------------------------------------*/

static int current_adc_raw_to_mv(uint32_t raw)
{
    int voltage_mv = 0;

    /*
     * 优先使用ESP-IDF提供的ADC校准功能。
     *
     * ESP32的ADC参考电压、非线性和芯片个体差异较大，
     * 直接使用 raw / 4095 × 3300 只能得到近似结果。
     */
    if (current_adc_cali_enable) {
        esp_err_t ret = adc_cali_raw_to_voltage(
            current_adc_cali_handle,
            raw,
            &voltage_mv
        );

        if (ret == ESP_OK) {
            return voltage_mv;
        }
    }

    /*
     * 如果ADC校准初始化失败，临时使用理论公式换算。
     *
     * 注意：这个结果只能用于初步调试，不适合精确电流测量。
     */
    voltage_mv = (int)((raw * 3300UL) / 4095UL);

    return voltage_mv;
}

/*----------------------------------------------------------
 * ADC校准初始化
 *----------------------------------------------------------*/

static bool current_adc_calibration_init(void)
{
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = CURRENT_ADC_UNIT,
        .atten = CURRENT_ADC_ATTEN,
        .bitwidth = CURRENT_ADC_BIT_WIDTH,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(
        &cali_config,
        &current_adc_cali_handle
    );

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC线性拟合校准初始化成功");
        return true;
    }

    ESP_LOGW(
        TAG,
        "ADC校准初始化失败：%s，后续使用理论电压换算",
        esp_err_to_name(ret)
    );

#else

    ESP_LOGW(TAG, "当前芯片或IDF不支持ADC线性拟合校准");

#endif

    return false;
}

/*----------------------------------------------------------
 * ADC转换完成回调
 *----------------------------------------------------------*/

static bool IRAM_ATTR current_adc_conv_done_cb(
    adc_continuous_handle_t adc_handle,
    const adc_continuous_evt_data_t *edata,
    void *user_data)
{
    BaseType_t must_yield = pdFALSE;

    /*
     * 这里只通知任务读取数据。
     *
     * 不要在中断回调中：
     * 1. 打印日志；
     * 2. 进行浮点运算；
     * 3. 执行阻塞操作。
     */
    vTaskNotifyGiveFromISR(
        current_adc_task_handle,
        &must_yield
    );

    return must_yield == pdTRUE;
}

/*----------------------------------------------------------
 * ADC连续模式初始化
 *----------------------------------------------------------*/

static void current_adc_continuous_init(
    adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t adc_handle = NULL;

    /*
     * 创建ADC连续采样句柄。
     *
     * ADC转换结果会通过DMA进入内部缓冲区。
     */
    adc_continuous_handle_cfg_t adc_handle_config = {
        .max_store_buf_size = CURRENT_ADC_STORE_BUFFER_SIZE,
        .conv_frame_size = CURRENT_ADC_READ_LEN,
    };

    ESP_ERROR_CHECK(
        adc_continuous_new_handle(
            &adc_handle_config,
            &adc_handle
        )
    );

    /*
     * ADC数字控制器配置。
     *
     * 4路通道轮流采样。
     */
    adc_continuous_config_t adc_config = {
        .sample_freq_hz = CURRENT_ADC_TOTAL_SAMPLE_HZ,
        .conv_mode = CURRENT_ADC_CONV_MODE,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[
        CURRENT_ADC_CHANNEL_NUM
    ] = {0};

    adc_config.pattern_num = CURRENT_ADC_CHANNEL_NUM;

    for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++) {
        adc_pattern[i].atten = CURRENT_ADC_ATTEN;
        adc_pattern[i].channel =
            current_adc_channels[i] & 0x07;
        adc_pattern[i].unit = CURRENT_ADC_UNIT;
        adc_pattern[i].bit_width =
            CURRENT_ADC_BIT_WIDTH;

        ESP_LOGI(
            TAG,
            "配置%s：ADC1_CH%d",
            current_adc_get_name(current_adc_channels[i]),
            current_adc_channels[i]
        );
    }

    adc_config.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(
        adc_continuous_config(
            adc_handle,
            &adc_config
        )
    );

    *out_handle = adc_handle;
}

/*----------------------------------------------------------
 * 判断所有通道是否完成零点校准
 *----------------------------------------------------------*/

static bool current_adc_zero_calibration_finished(
    const uint32_t zero_count[ESP32_ADC1_CHANNEL_MAX])
{
    for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++) {
        adc_channel_t channel = current_adc_channels[i];

        if (zero_count[channel] < CURRENT_ZERO_SAMPLE_NUM) {
            return false;
        }
    }

    return true;
}

/*----------------------------------------------------------
 * 主程序
 *----------------------------------------------------------*/

void app_main(void)
{
    esp_err_t ret;

    uint32_t ret_num = 0;

    uint8_t result[CURRENT_ADC_READ_LEN] = {0};

    /*
     * 零点校准累计值。
     *
     * 下标直接使用ADC通道号0～7。
     */
    uint64_t zero_sum[ESP32_ADC1_CHANNEL_MAX] = {0};
    uint32_t zero_count[ESP32_ADC1_CHANNEL_MAX] = {0};
    uint32_t zero_raw[ESP32_ADC1_CHANNEL_MAX] = {0};
    int zero_voltage_mv[ESP32_ADC1_CHANNEL_MAX] = {0};

    /*
     * 正常运行期间的平均值累计。
     */
    uint64_t report_sum[ESP32_ADC1_CHANNEL_MAX] = {0};
    uint32_t report_count[ESP32_ADC1_CHANNEL_MAX] = {0};

    bool zero_calibration_done = false;

    int64_t last_report_time_us = esp_timer_get_time();

    memset(result, 0, sizeof(result));

    /*
     * 保存当前任务句柄，ADC中断会通知这个任务读取数据。
     */
    current_adc_task_handle = xTaskGetCurrentTaskHandle();

    /*
     * 初始化ADC电压校准。
     */
    current_adc_cali_enable =
        current_adc_calibration_init();

    /*
     * 初始化ADC连续采样。
     */
    adc_continuous_handle_t adc_handle = NULL;

    current_adc_continuous_init(&adc_handle);

    /*
     * 注册ADC转换完成回调。
     */
    adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = current_adc_conv_done_cb,
    };

    ESP_ERROR_CHECK(
        adc_continuous_register_event_callbacks(
            adc_handle,
            &callbacks,
            NULL
        )
    );

    /*
     * 开始连续采样。
     */
    ESP_ERROR_CHECK(
        adc_continuous_start(adc_handle)
    );

    ESP_LOGW(TAG, "开始零电流偏置校准");

    ESP_LOGW(
        TAG,
        "校准期间必须关闭电机PWM，确保采样电阻中没有电流"
    );

    while (1) {
        /*
         * 等待ADC一帧数据转换完成。
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            ret = adc_continuous_read(
                adc_handle,
                result,
                CURRENT_ADC_READ_LEN,
                &ret_num,
                0
            );

            if (ret == ESP_ERR_TIMEOUT) {
                /*
                 * ADC内部当前已经没有可读取的数据。
                 */
                break;
            }

            if (ret != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "读取ADC失败：%s",
                    esp_err_to_name(ret)
                );

                break;
            }

            /*
             * 一个ADC采样结果占SOC_ADC_DIGI_RESULT_BYTES字节。
             */
            adc_continuous_data_t parsed_data[
                CURRENT_ADC_READ_LEN /
                SOC_ADC_DIGI_RESULT_BYTES
            ];

            uint32_t parsed_sample_num = 0;

            esp_err_t parse_ret =
                adc_continuous_parse_data(
                    adc_handle,
                    result,
                    ret_num,
                    parsed_data,
                    &parsed_sample_num
                );

            if (parse_ret != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "ADC数据解析失败：%s",
                    esp_err_to_name(parse_ret)
                );

                continue;
            }

            /*
             * 处理这一帧中的所有ADC采样结果。
             */
            for (uint32_t i = 0;
                 i < parsed_sample_num;
                 i++) {

                if (!parsed_data[i].valid) {
                    continue;
                }

                /*
                 * parsed_data中的unit通常从0开始：
                 *
                 * 0表示ADC1
                 * 1表示ADC2
                 */
                if (parsed_data[i].unit != CURRENT_ADC_UNIT) {
                    continue;
                }

                uint32_t channel =
                    parsed_data[i].channel;

                uint32_t raw =
                    parsed_data[i].raw_data;

                if (channel >= ESP32_ADC1_CHANNEL_MAX) {
                    continue;
                }

                /*
                 * 启动阶段先进行零电流偏置校准。
                 */
                if (!zero_calibration_done) {
                    if (zero_count[channel] <
                        CURRENT_ZERO_SAMPLE_NUM) {

                        zero_sum[channel] += raw;
                        zero_count[channel]++;
                    }
                } else {
                    /*
                     * 正常工作阶段累计采样值，
                     * 每100ms计算并打印一次平均值。
                     */
                    report_sum[channel] += raw;
                    report_count[channel]++;
                }
            }

            /*
             * 检查4路ADC是否全部收集够零点样本。
             */
            if (!zero_calibration_done &&
                current_adc_zero_calibration_finished(
                    zero_count)) {

                for (int i = 0;
                     i < CURRENT_ADC_CHANNEL_NUM;
                     i++) {

                    adc_channel_t channel =
                        current_adc_channels[i];

                    zero_raw[channel] =
                        zero_sum[channel] /
                        zero_count[channel];

                    zero_voltage_mv[channel] =
                        current_adc_raw_to_mv(
                            zero_raw[channel]
                        );

                    ESP_LOGI(
                        TAG,
                        "%s零点：raw=%" PRIu32
                        "，voltage=%dmV",
                        current_adc_get_name(channel),
                        zero_raw[channel],
                        zero_voltage_mv[channel]
                    );
                }

                zero_calibration_done = true;

                ESP_LOGI(TAG, "四路电流零点校准完成");
            }
        }

        /*
         * 每100ms输出一次平均电流。
         */
        int64_t now_time_us = esp_timer_get_time();

        if (zero_calibration_done &&
            now_time_us - last_report_time_us >=
            CURRENT_REPORT_PERIOD_US) {

            last_report_time_us = now_time_us;

            for (int i = 0;
                 i < CURRENT_ADC_CHANNEL_NUM;
                 i++) {

                adc_channel_t channel =
                    current_adc_channels[i];

                if (report_count[channel] == 0) {
                    continue;
                }

                uint32_t average_raw =
                    report_sum[channel] /
                    report_count[channel];

                int voltage_mv =
                    current_adc_raw_to_mv(average_raw);

                /*
                 * INA240A2电流换算：
                 *
                 * I = (Vout - Vzero) /
                 *     (增益 × 采样电阻)
                 *
                 * 使用mV作为单位时：
                 *
                 * I = (Vout_mV - Vzero_mV) / 500
                 */
                float current_a =
                    ((float)voltage_mv -
                     (float)zero_voltage_mv[channel]) /
                    INA240_MV_PER_AMP;

                ESP_LOGI(
                    TAG,
                    "%s：raw=%" PRIu32
                    "，voltage=%dmV，current=%.3fA",
                    current_adc_get_name(channel),
                    average_raw,
                    voltage_mv,
                    current_a
                );

                /*
                 * 清空本次统计，准备下一个100ms周期。
                 */
                report_sum[channel] = 0;
                report_count[channel] = 0;
            }
        }
    }

    /*
     * 当前程序不会运行到这里，保留标准释放流程。
     */
    ESP_ERROR_CHECK(
        adc_continuous_stop(adc_handle)
    );

    ESP_ERROR_CHECK(
        adc_continuous_deinit(adc_handle)
    );
}
```

ADC连续模式会自动循环采集多个通道，并通过DMA把结果送入内存；转换完成回调运行在中断环境中，所以回调里只做任务通知，不应该打印日志或执行阻塞处理。([Espressif Systems][2])

ESP32支持线性拟合ADC校准，使用 `adc_cali_raw_to_voltage()` 比简单按 `raw × 3.3 / 4095` 换算更适合做电流测量。([Espressif Systems][3])

---

# 三、CMakeLists.txt

如果编译时提示找不到ADC校准或者连续采样接口，确认组件依赖：

```cmake
idf_component_register(
    SRCS
        "main.c"

    INCLUDE_DIRS
        "."

    PRIV_REQUIRES
        esp_adc
)
```

---

# 四、上电零点校准非常重要

程序启动后会采集每路2000次，计算实际零电流输出。

校准期间必须满足：

```text
MOS管不输出PWM
电机没有相电流
驱动器没有制动电流
电机最好处于静止状态
```

不能一边让电机转，一边进行零点校准，否则程序会把真实电流当成零点。

你可以先：

```c
关闭M0 PWM；
关闭M1 PWM；
禁用FD6287；
启动ADC；
完成零点校准；
再启动电机。
```

---

# 五、INA240A2的电流换算

你这里是：

```text
INA240A2增益 = 50
采样电阻     = 10mΩ = 0.01Ω
```

因此：

$$
50\times0.01=0.5V/A
$$

也就是：

```text
每1A相电流，INA240输出变化500mV
```

程序中的计算就是：

```c
current_a =
    (voltage_mv - zero_voltage_mv) / 500.0f;
```

例如实际零点：

```text
zero_voltage = 1642mV
```

运行时测得：

```text
voltage = 2142mV
```

则：

$$
I=\frac{2142-1642}{500}=1A
$$

如果测得：

```text
voltage = 1142mV
```

则：

$$
I=\frac{1142-1642}{500}=-1A
$$

---

# 六、必须确认INA240的REF接法

程序不能直接假设零点一定是1.65V，要看你的INA240：

```text
REF1、REF2接到了哪里？
```

如果零电流时输出约为：

```text
1.65V
```

说明配置成了双向电流采样，可以测正负相电流。

如果零电流时输出接近：

```text
0V
```

说明参考端可能接地，只适合单向采样。电机相电流为反方向时，INA240输出会被限制在接近0V，无法正确恢复负电流。

烧录后先观察零点日志，例如：

```text
M0_CS2零点：raw=2040，voltage=1648mV
M0_CS1零点：raw=2031，voltage=1641mV
M1_CS2零点：raw=2046，voltage=1652mV
M1_CS1零点：raw=2038，voltage=1646mV
```

这种结果是比较正常的。

如果出现：

```text
zero_voltage=20mV
```

就要检查INA240的REF配置。

---

# 七、FOC中不能只这样自由连续采样

这份代码适合：

```text
确认硬件通不通
查看零点电压
确认电流方向
观察电流大致大小
做过流检测
```

但是你做20kHz PWM的FOC电流环时，ADC不能完全自由运行。

因为相线上存在高速PWM开关噪声：

```text
MOS刚开通时采样 → 可能采到尖峰
MOS死区期间采样 → 电流路径发生变化
PWM边沿采样     → 数值跳动严重
PWM中间采样     → 通常更加稳定
```

正式做FOC时应让ADC采样点与PWM同步，尽量安排在某个桥臂导通稳定区域或PWM周期中心附近。自由运行连续采样可以先验证硬件，但后续电流环建议改成：

```text
MCPWM周期事件
       ↓
在指定PWM计数位置触发采样
       ↓
获取Ia、Ib
       ↓
计算Ic = -(Ia + Ib)
       ↓
Clarke / Park
       ↓
电流PI环
```

另外，`CURRENT_ADC_TOTAL_SAMPLE_HZ = 80000` 是四路总采样率，四路轮询后，每一路大约得到20k次/秒。若仍使用原例子的20k总频率，四路情况下每路只有约5k次/秒。

[1]: https://docs.espressif.com/projects/esp-adf/en/latest/api-reference/peripherals/periph_adc_button.html "ADC Button Peripheral -  -  — Espressif Audio Development Framework Guide latest documentation"
[2]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc/adc_continuous.html "Analog to Digital Converter (ADC) Continuous Mode Driver - ESP32 -  — ESP-IDF Programming Guide v6.0.1 documentation"
[3]: https://docs.espressif.com/projects/esp-idf/en/v5.0.2/esp32/api-reference/peripherals/adc_calibration.html?utm_source=chatgpt.com "Analog to Digital Converter (ADC) Calibration Driver - ESP32"

-----
-----
-----


这版代码确实是我前面写炸了。**不是 INA240、ADC 引脚或 `adc_continuous_new_handle()` 本身的问题，而是 `app_main()` 栈太大，栈溢出后把堆管理数据踩坏了。**

日志非常明确：

```text
ADC线性拟合校准初始化成功
Guru Meditation Error: LoadProhibited

adc_continuous_new_handle()
  ↓
heap_caps_calloc()
  ↓
tlsf_malloc()
  ↓
block_size()
```

也就是说，ADC 驱动准备申请内存时，发现堆链表已经损坏，因此在堆分配器内部崩溃。

## 为什么会炸

我前面的代码在 `app_main()` 里放了一堆局部数组：

```c
uint8_t result[256];

uint64_t zero_sum[8];
uint32_t zero_count[8];
uint32_t zero_raw[8];
int zero_voltage_mv[8];

uint64_t report_sum[8];
uint32_t report_count[8];
```

尤其还有这个大数组：

```c
adc_continuous_data_t parsed_data[
    CURRENT_ADC_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES
];
```

从回溯地址看：

```text
main_task：0x3ffb4400
app_main：0x3ffb3960
```

一进入 `app_main()` 就已经消耗约 2.7KB 栈；调用到堆分配器时大约消耗了 3KB 以上。默认 main task 栈空间比较紧，继续嵌套调用就越界，把附近的堆管理结构踩坏了。

ESP-IDF 中 `app_main()` 是由 main task 调用的，其栈大小由 `CONFIG_ESP_MAIN_TASK_STACK_SIZE` 决定。ADC 连续模式的 `adc_continuous_new_handle()` 又会动态申请驱动上下文和内部采样缓冲区，所以堆结构一旦被栈踩坏，就会在这个函数中暴雷。([Espressif Systems][1])

# 先让程序立即运行起来

执行：

```bash
idf.py menuconfig
```

进入：

```text
Component config
    → ESP System Settings
        → Main task stack size
```

改成：

```text
8192
```

然后：

```bash
idf.py fullclean
idf.py build
idf.py flash monitor
```

也可以在 `sdkconfig.defaults` 写：

```text
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
```

这能快速验证确实是栈溢出。

但更正确的做法，是不要把大数组放在 `app_main()` 栈里。

# 正确修改：所有大数组改成静态全局变量

在文件顶部增加：

```c
#define CURRENT_ADC_READ_LEN        256
#define ESP32_ADC1_CHANNEL_MAX      8

/*
 * 大数组必须放到静态存储区，
 * 不要全部压在app_main任务栈中。
 */
static uint8_t s_adc_result[CURRENT_ADC_READ_LEN];

/* 零电流校准统计 */
static uint64_t s_zero_sum[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_zero_count[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_zero_raw[ESP32_ADC1_CHANNEL_MAX];
static int s_zero_voltage_mv[ESP32_ADC1_CHANNEL_MAX];

/* 运行时平均值统计 */
static uint64_t s_report_sum[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_report_count[ESP32_ADC1_CHANNEL_MAX];
```

然后删除 `app_main()` 中对应的局部变量：

```c
uint8_t result[CURRENT_ADC_READ_LEN] = {0};

uint64_t zero_sum[ESP32_ADC1_CHANNEL_MAX] = {0};
uint32_t zero_count[ESP32_ADC1_CHANNEL_MAX] = {0};
uint32_t zero_raw[ESP32_ADC1_CHANNEL_MAX] = {0};
int zero_voltage_mv[ESP32_ADC1_CHANNEL_MAX] = {0};

uint64_t report_sum[ESP32_ADC1_CHANNEL_MAX] = {0};
uint32_t report_count[ESP32_ADC1_CHANNEL_MAX] = {0};
```

所有变量名对应替换：

```c
result          → s_adc_result
zero_sum        → s_zero_sum
zero_count      → s_zero_count
zero_raw        → s_zero_raw
zero_voltage_mv → s_zero_voltage_mv
report_sum      → s_report_sum
report_count    → s_report_count
```

# 不要再创建 `parsed_data` 大数组

ESP32 使用 `ADC_DIGI_OUTPUT_FORMAT_TYPE1`，可以直接解析 DMA 原始数据，不需要再生成一份解析结果数组。

把原来的：

```c
adc_continuous_data_t parsed_data[
    CURRENT_ADC_READ_LEN /
    SOC_ADC_DIGI_RESULT_BYTES
];

uint32_t parsed_sample_num = 0;

esp_err_t parse_ret =
    adc_continuous_parse_data(
        adc_handle,
        result,
        ret_num,
        parsed_data,
        &parsed_sample_num
    );

for (uint32_t i = 0; i < parsed_sample_num; i++) {
    ...
}
```

全部替换成：

```c
/*
 * ESP32连续ADC使用TYPE1格式。
 *
 * DMA缓冲区中每SOC_ADC_DIGI_RESULT_BYTES字节
 * 代表一次ADC转换结果。
 */
for (uint32_t i = 0;
     i < ret_num;
     i += SOC_ADC_DIGI_RESULT_BYTES) {

    /*
     * 直接把DMA数据解释为ADC转换结果，
     * 不再创建额外的大型解析数组。
     */
    const adc_digi_output_data_t *adc_data =
        (const adc_digi_output_data_t *)&s_adc_result[i];

    /*
     * ESP32使用TYPE1数据格式。
     */
    uint32_t channel = adc_data->type1.channel;
    uint32_t raw = adc_data->type1.data;

    /*
     * 只处理ADC1通道0～7。
     */
    if (channel >= ESP32_ADC1_CHANNEL_MAX) {
        continue;
    }

    if (!zero_calibration_done) {
        /*
         * 上电零电流校准阶段。
         */
        if (s_zero_count[channel] <
            CURRENT_ZERO_SAMPLE_NUM) {

            s_zero_sum[channel] += raw;
            s_zero_count[channel]++;
        }
    } else {
        /*
         * 正常工作阶段累计ADC采样值。
         */
        s_report_sum[channel] += raw;
        s_report_count[channel]++;
    }
}
```

读取也改成：

```c
ret = adc_continuous_read(
    adc_handle,
    s_adc_result,
    sizeof(s_adc_result),
    &ret_num,
    0
);
```

# 初始化配置保留这样

```c
static void current_adc_continuous_init(
    adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t adc_handle = NULL;

    adc_continuous_handle_cfg_t adc_handle_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = CURRENT_ADC_READ_LEN,
    };

    ESP_ERROR_CHECK(
        adc_continuous_new_handle(
            &adc_handle_config,
            &adc_handle
        )
    );

    adc_continuous_config_t adc_config = {
        /*
         * 四路总采样率80kHz，
         * 平均每路约20kHz。
         */
        .sample_freq_hz = 80 * 1000,

        /*
         * 只使用ADC1。
         */
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,

        /*
         * 经典ESP32使用TYPE1格式。
         */
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    static adc_digi_pattern_config_t adc_pattern[4];

    const adc_channel_t channels[4] = {
        ADC_CHANNEL_0,     /* GPIO36：M0_CS2 */
        ADC_CHANNEL_3,     /* GPIO39：M0_CS1 */
        ADC_CHANNEL_6,     /* GPIO34：M1_CS2 */
        ADC_CHANNEL_7,     /* GPIO35：M1_CS1 */
    };

    memset(adc_pattern, 0, sizeof(adc_pattern));

    adc_config.pattern_num = 4;

    for (int i = 0; i < 4; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = channels[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width =
            SOC_ADC_DIGI_MAX_BITWIDTH;
    }

    adc_config.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(
        adc_continuous_config(
            adc_handle,
            &adc_config
        )
    );

    *out_handle = adc_handle;
}
```

## 建议增加栈检查日志

在调用 ADC 初始化前打印：

```c
ESP_LOGI(
    TAG,
    "main任务剩余栈：%u",
    (unsigned int)uxTaskGetStackHighWaterMark(NULL)
);
```

例如：

```c
current_adc_cali_enable =
    current_adc_calibration_init();

ESP_LOGI(
    TAG,
    "ADC初始化前剩余栈：%u",
    (unsigned int)uxTaskGetStackHighWaterMark(NULL)
);

current_adc_continuous_init(&adc_handle);
```

修复后应该能够看到：

```text
ADC线性拟合校准初始化成功
ADC初始化前剩余栈：xxxx
配置M0_CS2：ADC1_CH0
配置M0_CS1：ADC1_CH3
配置M1_CS2：ADC1_CH6
配置M1_CS1：ADC1_CH7
开始零电流偏置校准
```

不会再死在：

```text
adc_continuous_new_handle()
```

另外这个警告：

```text
Detected size(8192k) larger than the size in the binary image header(2048k)
```

说明实际 Flash 是 8MB，但工程配置成了 2MB。**它不是本次崩溃原因**，不过后面应在：

```text
Serial flasher config
    → Flash size
        → 8 MB
```

改成 8MB。

[1]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/performance/ram-usage.html?utm_source=chatgpt.com "Minimizing RAM Usage - ESP32 - — ESP-IDF ..."
