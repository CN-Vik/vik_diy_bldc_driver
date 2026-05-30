ESP32-WROOM-32-N8 可以理解成：**乐鑫 ESP32 系列的经典 Wi-Fi + 蓝牙双模模组，板载 PCB 天线，内置 8MB SPI Flash，无 PSRAM**。

> 注意：乐鑫现在把老的 **ESP32-WROOM-32** 标为 **NRND / 不推荐用于新设计**，新项目一般更推荐用 **ESP32-WROOM-32E-N8** 或其它更新系列，比如 ESP32-S3/C3/C6 等。官方 ESP32-WROOM-32 规格书里标准版本是 4MB Flash，而 **N8** 这种后缀通常表示 8MB Flash 配置；ESP32-WROOM-32E-N8 官方型号表明确是 **8MB Quad SPI Flash，无 PSRAM**。([Espressif Systems][1])

## 一、核心参数

| 项目    | 参数                                               |
| ----- | ------------------------------------------------ |
| 模组型号  | ESP32-WROOM-32-N8                                |
| 核心芯片  | ESP32 系列 SoC，常见为 ESP32-D0WDQ6 / ESP32-D0WD-V3 系列 |
| CPU   | Xtensa 32-bit LX6 双核                             |
| 主频    | 最高 240 MHz                                       |
| Flash | **8 MB SPI Flash**                               |
| PSRAM | **无 PSRAM**                                      |
| SRAM  | 520 KB SRAM                                      |
| ROM   | 448 KB ROM                                       |
| Wi-Fi | 2.4 GHz 802.11 b/g/n                             |
| 蓝牙    | Bluetooth v4.2 BR/EDR + BLE                      |
| 天线    | 板载 PCB 天线                                        |
| 供电电压  | 通常 3.0 ~ 3.6 V                                   |
| 工作温度  | -40 ~ 85 ℃                                       |
| 模组尺寸  | 约 18 × 25.5 × 3.1 mm                             |
| 封装    | SMD 贴片模组                                         |

Digi-Key 对 ESP32-WROOM-32-N8 的页面也列出了 2.4~2.5GHz、150Mbps、20.5dBm 输出功率、-98dBm 灵敏度、I2C/I2S/PWM/SDIO/UART 等接口信息。([DigiKey][2])

## 二、这个模组能干啥？

它本质上是一个 **带 Wi-Fi 和蓝牙的 MCU 模组**，不是单纯 Wi-Fi 模块。你可以直接在 ESP32 上跑程序，比如：

开发板/产品主控、Wi-Fi 联网、MQTT、HTTP、TCP/UDP、蓝牙 BLE 配网、传感器采集、继电器控制、小屏幕显示、音频简单处理、智能家居设备、数据采集器等。

它内部已经集成了：

* ESP32 主控芯片
* 40 MHz 晶振
* 8MB SPI Flash
* 射频匹配电路
* PCB 天线
* 金属屏蔽罩

所以你做产品时，不需要自己画复杂的射频电路，只要按照参考设计把电源、EN、BOOT、串口下载、GPIO 引出来即可。

## 三、外设资源

ESP32 的外设非常丰富，常用的有：

| 外设       | 说明                                 |
| -------- | ---------------------------------- |
| GPIO     | 多个可复用 GPIO                         |
| UART     | 通常有 3 路 UART                       |
| SPI      | 可接屏幕、Flash、传感器等                    |
| I2C      | 可接温湿度、IMU、RTC、触摸芯片等                |
| I2S      | 可接音频 Codec、麦克风、功放                  |
| ADC      | 12-bit ADC，注意 Wi-Fi 开启时 ADC2 使用有限制 |
| DAC      | 2 路 8-bit DAC                      |
| PWM      | LED PWM、电机 PWM                     |
| Touch    | 电容触摸输入                             |
| SDIO     | 可接 SD 卡或作为 SDIO 从设备                |
| TWAI/CAN | 兼容 CAN 2.0 控制器                     |

官方 ESP32-WROOM-32 规格书也列出了 UART、SPI、SDIO、I2C、LED PWM、Motor PWM、I2S、IR、脉冲计数器、ADC、DAC、TWAI 等外设。([Espressif Systems][1])

## 四、N8 是什么意思？

`ESP32-WROOM-32-N8` 里可以这样理解：

```text
ESP32        ：芯片系列
WROOM        ：通用型模组系列，通常只有 Flash，没有 PSRAM
32           ：ESP32 经典系列
N8           ：常温版本，8MB Flash
```

如果是：

```text
N4   = 4MB Flash
N8   = 8MB Flash
N16  = 16MB Flash
R2   = 带 2MB PSRAM
```

例如官方 ESP32-WROOM-32E 系列里：

```text
ESP32-WROOM-32E-N4    4MB Flash，无 PSRAM
ESP32-WROOM-32E-N8    8MB Flash，无 PSRAM
ESP32-WROOM-32E-N16   16MB Flash，无 PSRAM
ESP32-WROOM-32E-N8R2  8MB Flash + 2MB PSRAM
```

这一点在乐鑫 ESP32-WROOM-32E/UE 技术规格书的型号对比表里有明确列出。([Espressif Systems][3])

## 五、和 ESP32-WROVER 的区别

这个你很容易混淆：

| 类型                   |       Flash |        PSRAM | 适合场景                 |
| -------------------- | ----------: | -----------: | -------------------- |
| ESP32-WROOM-32-N8    |         8MB |            无 | 普通联网、控制、传感器、MQTT、BLE |
| ESP32-WROVER         | 常见 4/8/16MB | 通常 8MB PSRAM | 图像、摄像头、音频、大内存缓冲      |
| ESP32-WROOM-32E-N8R2 |         8MB |    2MB PSRAM | 比普通 WROOM 稍强，能跑更大的应用 |

乐鑫也说明过，经典 ESP32 时代的 WROOM 主要是额外 Flash，WROVER 则集成 PSRAM。([Developer Portal][4])

## 六、开发时要注意的点

第一，**供电一定要稳**。ESP32 Wi-Fi 发射瞬间电流比较大，3.3V 电源最好能提供 500mA 以上余量，旁边放 10uF、100nF 去耦电容。

第二，**GPIO0 是下载模式脚**。启动时 GPIO0 拉低会进下载模式，拉高/悬空正常启动。

第三，**EN 是复位/使能脚**。EN 拉低复位，正常工作要拉高，一般接 10k 上拉。

第四，**GPIO6~GPIO11 通常不要用**。这些脚内部接 SPI Flash，用了会导致程序跑飞或无法启动。

第五，**ADC2 和 Wi-Fi 有冲突**。Wi-Fi 工作时尽量用 ADC1 通道，不要优先用 ADC2。

第六，**不要天线下铺铜**。PCB 设计时，模组天线区域要伸出板边，或者天线下方禁布铜、禁走线。

## 七、适合你的嵌入式理解方式

你可以把它看成：

```text
ESP32-WROOM-32-N8 = ESP32 MCU + Wi-Fi + 蓝牙 + 8MB Flash + 天线 + 射频电路
```

裸芯片 ESP32 你还要自己配晶振、Flash、射频、天线匹配；而 WROOM 模组已经帮你做好了这些，适合快速做产品和学习。对于一般 IoT 项目，8MB Flash 已经比较够用；但如果你要做摄像头、LVGL 大图片、音频缓存，最好选带 PSRAM 的 ESP32-WROVER 或 ESP32-S3-N16R8。

[1]: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_cn.pdf?utm_source=chatgpt.com "ESP32-WROOM-32 - 技术规格书版本3.6"
[2]: https://www.digikey.com/en/products/detail/espressif-systems/ESP32-WROOM-32-N8/9381712?utm_source=chatgpt.com "ESP32-WROOM-32-N8 Espressif Systems"
[3]: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_cn.pdf?utm_source=chatgpt.com "ESP32-WROOM-32E ESP32-WROOM-32UE"
[4]: https://developer.espressif.com/blog/2025/03/espressif-part-numbers-explained/?utm_source=chatgpt.com "Espressif part numbers explained: A complete guide"



------
-----
-----

可以，**ESP32-WROOM-32-N8 用来做 BLDC/PMSM 的 FOC 是有优势的**，尤其适合你现在这种“学习 FOC、手写 SVPWM、做开环/有感控制 Demo”的阶段。但它不是专门为高性能电机控制设计的芯片，和 STM32G4、TI C2000 这种电机控制 MCU 比，ADC 和实时性会弱一些。

## 1. 用 ESP32 做 FOC 的主要优势

### 第一，ESP32 有专门的 MCPWM 外设

FOC 驱动三相 BLDC/PMSM，本质上需要输出 **三相互补 PWM**：

```text
UH / UL
VH / VL
WH / WL
```

也就是三组半桥，共 6 路 PWM。ESP32 的 MCPWM 外设正好就是为电机、电源这种场景准备的。官方文档里说，ESP32 有两个 MCPWM 单元，每个单元有三对 PWM 输出，也就是一组 MCPWM 就够驱动一个三相逆变桥。MCPWM 还支持 fault/capture/sync 等功能，可用于过流保护、霍尔捕获、同步控制等。([Espressif Systems][1])

这点对 FOC 非常关键。普通 LEDC PWM 也能输出 PWM，但它不适合严肃三相电机控制，因为互补输出、死区、同步更新这些不好做。

### 第二，MCPWM 支持死区时间

三相桥上下 MOS 不能同时导通，否则就是直通短路。所以半桥驱动需要死区时间。ESP32 MCPWM 有 dead time 子模块，可以给 PWM 边沿插入延迟，ESP-IDF 文档也明确描述了 dead time 用于给已有 PWM 边沿增加额外延迟。([Espressif Systems][2])

也就是说你可以这样理解：

```text
FOC算法算出 Ta/Tb/Tc
        ↓
更新 MCPWM 三相占空比
        ↓
MCPWM 硬件自动输出互补 PWM
        ↓
MCPWM 硬件插入死区
        ↓
驱动三相 MOS / 驱动芯片
```

这比你手动用 GPIO 翻转安全太多。

### 第三，官方已经有 SVPWM/FOC 示例

ESP-IDF 文档里有 `mcpwm_foc_svpwm_open_loop` 示例，官方说明这个例子会用 MCPWM 产生三对 PWM 信号，用于 FOC，可以驱动 BLDC/PMSM 或三相逆变器。([Espressif Systems][2])

所以你不是从完全空白开始，可以基于这个例子改。你之前 ESP-IDF v5.5.3 目录里看到的：

```text
examples/peripherals/mcpwm/mcpwm_foc_svpwm_open_loop
```

这个就是最适合你入门手写 FOC 的例子。

### 第四，双核 + 240MHz，算力对入门 FOC 够用

ESP32 是 Xtensa LX6 架构，常见双核，最高 240MHz。官方 datasheet 也写了 ESP32 支持单核/双核 32-bit LX6，单核 240MHz 下 CoreMark 分数为 539.98。

实际做 FOC 时，你可以这样分配：

```text
Core 0：Wi-Fi / BLE / 日志 / 通信 / UI
Core 1：FOC控制环 / PWM更新 / 编码器或霍尔采样
```

这样做的好处是：电机控制任务可以尽量固定在一个核上，减少 Wi-Fi、日志打印、MQTT 这些东西对控制环的影响。

但是注意：**Wi-Fi 开启后实时性会变差**。如果你做高频闭环 FOC，最好先关闭 Wi-Fi/BLE，把 ESP32 当纯 MCU 用；等电机控制稳定后，再考虑加通信。

## 2. 数学运算方面的优势

FOC 里面主要是这些数学运算：

```text
Clarke 变换
Park 变换
反 Park 变换
SVPWM 扇区判断
sin / cos
PI 控制器
电流采样滤波
速度估算
```

ESP32 在这方面有几个优势。

### 第一，有硬件 FPU

ESP32 CPU 支持 Floating Point Unit，也支持 DSP 指令，例如 32-bit 乘法器、32-bit 除法器和 40-bit MAC。这个在官方 datasheet 里有明确说明。

所以 ESP32 做 FOC 时，**可以直接用 `float` 写算法**，不用像某些低端 MCU 那样一开始就被迫写 Q15/Q31 定点算法。

比如你可以直接写：

```c
float i_alpha;
float i_beta;
float i_d;
float i_q;
float sin_theta;
float cos_theta;
float v_d;
float v_q;
```

这对新手非常友好，代码可读性也强。

### 第二，适合用 `float`，不建议大量用 `double`

ESP32 的 FOC 算法里建议统一用：

```c
float
sinf()
cosf()
```

不要习惯性用：

```c
double
sin()
cos()
```

原因是 FOC 控制环频率比较高，比如 10kHz、20kHz，每次中断都要算一轮。如果用 `double`，运算开销更大，而且 FOC 本身也不需要 double 精度。

推荐写法：

```c
float theta = angle_elec;
float s = sinf(theta);
float c = cosf(theta);
```

而不是：

```c
double theta = angle_elec;
double s = sin(theta);
double c = cos(theta);
```

### 第三，可以用 ESP-DSP 库

乐鑫有官方 ESP-DSP 库，提供矩阵、FFT、滤波、向量运算等高性能计算函数，文档也说明这个库包含矩阵乘法、FFT、滤波、向量数学等功能。([Espressif Systems][3])

ESP-DSP 里很多函数有针对 ESP32 的 `_ae32` 优化版本，并且支持 32-bit float 和 16-bit 定点实现。([ESP组件注册表][4])

不过对基础 FOC 来说，你不一定必须用 ESP-DSP。因为 Clarke/Park/SVPWM 这些公式很短，自己手写就行。ESP-DSP 更适合你后面做：

```text
数字滤波
FFT分析
振动分析
矩阵运算
姿态解算辅助
观测器算法
```

## 3. ESP32 做 FOC 的实际优势总结

| 方面   | 优势                          |
| ---- | --------------------------- |
| PWM  | MCPWM 天然适合三相桥               |
| 互补输出 | 可以输出高低桥互补 PWM               |
| 死区   | 硬件 dead time 支持             |
| 保护   | 支持 fault/brake 类机制          |
| 位置输入 | capture 可用于霍尔/脉冲捕获          |
| 算力   | 双核 240MHz，基础 FOC 够用         |
| 浮点   | 支持 FPU，可以直接 float           |
| 开发   | ESP-IDF 有官方 SVPWM 开环 FOC 例子 |
| 通信   | Wi-Fi/BLE 方便做调参、上位机、日志      |

## 4. 但是它也有明显短板

### 第一，ADC 不算强

FOC 真正难的不是 PWM，而是：

```text
电流采样准不准
采样时刻对不对
ADC 噪声大不大
ADC 和 PWM 能不能严格同步
```

ESP32 的 ADC 做普通传感器采样还行，但做高性能电流环就比较一般。你如果要做严肃的闭环电流 FOC，ESP32 不如 STM32G4、STM32F3、TI C2000 这种带高级 ADC 触发链路的 MCU。

所以 ESP32 更适合：

```text
开环 FOC
有霍尔 BLDC 控制
低成本小功率 FOC
学习 SVPWM / Park / Clarke
不追求极致电流环性能的项目
```

不太适合一上来就做：

```text
高转速
大电流
高动态响应
高精度电流闭环
工业级伺服
```

### 第二，Wi-Fi 会影响实时性

ESP32 的 Wi-Fi 很方便，但 FOC 是实时控制。FOC 控制环最好稳定周期运行，比如：

```text
PWM频率：20kHz
控制周期：50us
```

如果 Wi-Fi、日志、Flash 写入、蓝牙任务抢占太多，就会造成控制环抖动。所以开发时建议：

```text
第一阶段：关闭 Wi-Fi，只做电机转起来
第二阶段：开环 FOC 稳定
第三阶段：加入霍尔/编码器
第四阶段：加入电流采样
第五阶段：最后再加 Wi-Fi 调参/监控
```

### 第三，ESP32 没有专用 CORDIC/电机控制加速器

有些 MCU，比如 STM32G4，带 CORDIC，可以更快算三角函数；TI C2000 也有专门面向电机控制的生态。ESP32 没有这种专用电机控制加速器，所以 `sin/cos`、Park 变换、PI 控制这些还是主要靠 CPU/FPU。

不过对于学习和中低性能 FOC，ESP32 的算力是够你玩的。

## 5. 对你现在最推荐的路线

你如果用 ESP32-WROOM-32-N8 做 BLDC FOC，我建议这样走：

```text
第一步：MCPWM 输出三相互补 PWM
第二步：固定角度 theta，验证三相波形
第三步：做开环角度递增，让电机低速转起来
第四步：加入 SVPWM
第五步：加入电角度 / 机械角度概念
第六步：加入霍尔或编码器
第七步：加入速度环 PI
第八步：最后再考虑电流采样和电流环
```

最开始别急着做完整三闭环 FOC。你先把这个链路跑通：

```text
theta += speed * dt
        ↓
反 Park 得到 V_alpha / V_beta
        ↓
SVPWM 计算 Ta/Tb/Tc
        ↓
MCPWM 更新三相占空比
        ↓
三相桥驱动电机转动
```

## 6. 结论

ESP32-WROOM-32-N8 做 FOC 的优势是：**MCPWM 很适合三相电机控制，支持三对 PWM、死区、fault/capture；CPU 有 FPU 和 DSP 指令，float 算法好写；双核 240MHz 对开环/有感/基础闭环 FOC 足够；官方还有 MCPWM SVPWM 开环 FOC 示例。**

它的短板是：**ADC 和实时性不如专业电机控制 MCU，Wi-Fi 会干扰实时控制，没有专用 CORDIC/高级电机控制外设。**

所以一句话：

> **ESP32-WROOM-32-N8 很适合拿来学习 FOC、做开环 SVPWM、有感 BLDC 控制、小功率低成本项目；但如果要做高性能电流闭环伺服，STM32G4 / TI C2000 会更合适。**

[1]: https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/peripherals/mcpwm.html "Motor Control Pulse Width Modulator (MCPWM) - ESP32 -  — ESP-IDF Programming Guide v4.4.1 documentation"
[2]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html "Motor Control Pulse Width Modulator (MCPWM) - ESP32 -  — ESP-IDF Programming Guide v6.0.1 documentation"
[3]: https://docs.espressif.com/projects/esp-dsp/en/latest/index.html?utm_source=chatgpt.com "ESP-DSP Library - ESP32"
[4]: https://components.espressif.com/components/espressif/esp-dsp/versions/1.2.0/readme?language= "
    espressif/esp-dsp • v1.2.0
•
    ESP Component Registry"

----
----
----
MCPWM 全称是 **Motor Control Pulse Width Modulator**，中文就是 **电机控制脉宽调制器**。它不是普通 PWM，而是 ESP32 里专门为 **电机控制、三相逆变、数字电源、FOC/SVPWM** 准备的一套高级 PWM 外设。乐鑫官方文档也明确把 MCPWM 用途列为有刷/无刷电机、数字电源、测量外部脉宽，以及给 FOC 生成 SVPWM 信号。([Espressif Systems][1])

你可以先这样理解：

```text
普通 PWM：只会输出一个占空比波形
MCPWM：能输出成对互补 PWM + 死区 + 同步 + 故障保护 + 捕获输入
```

对于 BLDC/FOC 来说，MCPWM 是 ESP32 上最应该用的 PWM 外设。

---

# 1. ESP32 里 MCPWM 有多少资源？

以经典 ESP32，也就是你说的 ESP32-WROOM-32-N8 内部 ESP32 芯片来说，它有 **两个 MCPWM 外设：MCPWM0 和 MCPWM1**。官方 TRM 里也写到 ESP32 contains two MCPWM peripherals: MCPWM0 and MCPWM1。

每个 MCPWM 外设大概包含：

| 资源            |  数量 | 作用                |
| ------------- | --: | ----------------- |
| PWM Timer     | 3 个 | 产生 PWM 计数基准       |
| PWM Operator  | 3 个 | 根据计数事件生成 PWM 波形   |
| PWM 输出对       | 3 对 | 每对为 PWMxA / PWMxB |
| Capture       | 1 组 | 捕获外部脉冲，例如霍尔、测速脉冲  |
| Fault / Brake |  支持 | 过流、过压、急停保护        |
| Sync          |  支持 | 多路 PWM 同步/相位对齐    |
| Dead Time     |  支持 | 上下桥臂互补输出插入死区      |

旧版 ESP-IDF 文档也直接写了：ESP32 有两个 MCPWM 单元，每个单元有三对 PWM 输出。([Espressif Systems][2])

所以一组 MCPWM 就天然适合三相桥：

```text
MCPWM0 Operator0 → U 相：UH / UL
MCPWM0 Operator1 → V 相：VH / VL
MCPWM0 Operator2 → W 相：WH / WL
```

也就是：

```text
PWM0A → U 相上桥
PWM0B → U 相下桥

PWM1A → V 相上桥
PWM1B → V 相下桥

PWM2A → W 相上桥
PWM2B → W 相下桥
```

这正好对应 BLDC/FOC 三相逆变桥的 6 路驱动信号。

---

# 2. MCPWM 的内部结构

MCPWM 内部不是一个简单的 PWM 寄存器，而是由多个子模块组合起来的。官方 ESP-IDF 文档列出的主要子模块包括：Timer、Operator、Comparator、Generator、Fault、Sync、Dead Time、Carrier、Brake、Capture。([Espressif Systems][1])

整体关系可以理解成这样：

```text
MCPWM Timer
    ↓ 产生计数值 / 周期事件
Comparator
    ↓ 比较计数值和占空比阈值
Generator
    ↓ 根据事件拉高/拉低输出
Dead Time
    ↓ 给互补输出插入死区
GPIO
    ↓
三相栅极驱动器 / MOSFET / 电机
```

更贴近 FOC 的流程是：

```text
FOC 算法算出 Ta / Tb / Tc
        ↓
把 Ta / Tb / Tc 转成 MCPWM comparator 比较值
        ↓
MCPWM timer 周期计数
        ↓
比较事件触发 generator 翻转
        ↓
dead time 产生互补高低桥 PWM
        ↓
驱动三相逆变桥
```

---

# 3. Timer：PWM 的时间基准

Timer 是 MCPWM 的“节拍器”。

它负责产生 PWM 周期，比如：

```text
PWM 频率 = 20kHz
周期 = 50us
```

在 FOC 里，一般会让三相 PWM 使用同一个 Timer，这样 U/V/W 三相的 PWM 周期完全同步。

例如：

```text
Timer0 → Operator0 → U 相
       → Operator1 → V 相
       → Operator2 → W 相
```

这样三相 PWM 在同一个周期边界更新，波形更稳定。

MCPWM Timer 通常支持不同计数方式：

```text
向上计数：0 → Period → 0
向下计数：Period → 0 → Period
上下计数：0 → Period → 0
```

FOC/SVPWM 更推荐用 **上下计数 / center-aligned PWM / 中心对齐 PWM**。原因是中心对齐 PWM 的开关噪声和谐波表现通常更好，也方便在 PWM 中间位置做 ADC 电流采样。

---

# 4. Comparator：占空比比较器

Comparator 可以理解为“占空比阈值寄存器”。

比如 PWM 周期计数范围是：

```text
0 ~ 1000
```

你想输出 30% 占空比，就可以把比较值设成：

```text
compare = 300
```

Timer 计数时，一旦计数值等于 compare，就会产生比较事件。Generator 收到这个事件后，可以把 PWM 输出拉高或者拉低。

简单理解：

```text
Timer 计数到 0       → PWM 拉高
Timer 计数到 compare → PWM 拉低
```

这样就形成 PWM 波形。

在 FOC 里，SVPWM 最后算出来的是三个相位的占空比：

```text
duty_u
duty_v
duty_w
```

然后你把它们转换成 comparator 值：

```c
cmp_u = duty_u * period_ticks;
cmp_v = duty_v * period_ticks;
cmp_w = duty_w * period_ticks;
```

然后写进 MCPWM 的三个 comparator 里。

---

# 5. Generator：真正控制 PWM 引脚电平

Generator 是“波形生成器”。

它根据 Timer 事件、Comparator 事件来决定输出引脚什么时候高、什么时候低。官方文档也说明，Generator 可以基于 Timer、Comparator 等事件生成互补或独立 PWM 波形。([Espressif Systems][1])

比如你可以配置成：

```text
计数器到周期起点 → PWM 输出高
计数器到比较值   → PWM 输出低
```

这样就是普通 PWM。

也可以配置成互补输出：

```text
PWMxA 高的时候，PWMxB 低
PWMxA 低的时候，PWMxB 高
```

这对半桥驱动非常关键。

---

# 6. Dead Time：死区时间

这是 MCPWM 对电机控制非常重要的功能。

三相逆变器的每一相都是一个半桥：

```text
     VMOT
      |
   上桥 MOS
      |
      +------ 电机相线 U/V/W
      |
   下桥 MOS
      |
     GND
```

上桥 MOS 和下桥 MOS 绝对不能同时导通。否则就是：

```text
VMOT → 上桥 MOS → 下桥 MOS → GND
```

这叫 **桥臂直通 / shoot-through**，轻则发热，重则炸 MOS。

所以需要死区：

```text
上桥关断
等待一小段时间
下桥再导通
```

MCPWM 的 Dead Time 子模块就是专门干这个的。官方 TRM 里说明 Dead Time Generator 可以基于 PWMxA/PWMxB 产生带死区的信号对，也支持 rising edge delay 和 falling edge delay，并可配置 active high complementary、active low complementary 等互补模式。

你可以理解成：

```text
算法只输出一个理想 PWM
        ↓
MCPWM Dead Time 自动生成互补高低桥
        ↓
UH / UL 之间自动错开一点时间
```

常见死区可能是：

```text
200ns
500ns
1us
2us
```

具体取决于 MOSFET、栅极驱动芯片、母线电压、开关速度。不能随便固定，要看硬件。

---

# 7. Fault / Brake：故障保护

这个也是 MCPWM 比普通 PWM 强很多的地方。

电机控制里常见保护信号有：

```text
过流
过压
欠压
驱动芯片 FAULT
急停输入
母线短路保护
```

这些信号可以接到 ESP32 的 GPIO，然后配置成 MCPWM Fault 输入。一旦 Fault 触发，MCPWM 可以强制把 PWM 输出切到安全状态。ESP-IDF 文档也说明 Fault 模块主要通过 GPIO matrix 检测外部故障条件，故障有效后 MCPWM Operator 可以把 Generator 强制到预定义状态，保护系统不被损坏。([Espressif Systems][1])

例如：

```text
过流信号触发
        ↓
MCPWM 立即关闭 UH/VH/WH/UL/VL/WL
        ↓
MOS 停止开关
        ↓
保护电机和功率板
```

对于实际电机驱动板，**Fault 保护非常重要**。不要只靠软件任务里轮询电流，因为软件响应慢，而且可能被中断、Wi-Fi、日志影响。

---

# 8. Sync：同步功能

Sync 用于让多个 PWM Timer 同步。

比如你有三相 PWM，希望三相 PWM 周期起点完全一致：

```text
U 相 PWM 周期起点
V 相 PWM 周期起点
W 相 PWM 周期起点
```

必须对齐。否则三相占空比虽然算对了，但更新时刻乱了，波形会抖。

官方文档说明 Sync 模块用于同步 MCPWM timers，使不同 Generator 产生的 PWM 信号可以有固定相位差，sync 信号可以来自 GPIO matrix 或 MCPWM Timer 事件。([Espressif Systems][1])

FOC 里一般最简单的做法是：

```text
三个 Operator 共用同一个 Timer
```

这样天然同步。

如果更复杂，比如多路电机、多相交错电源，就会用 Sync 控制相位关系。

---

# 9. Capture：捕获输入

Capture 可以捕获外部脉冲的时间点。

比如：

```text
霍尔传感器边沿
编码器脉冲
测速信号
超声波 Echo 脉冲
外部频率信号
```

官方文档说明 Capture 是独立子模块，有专用 timer 和多个通道；GPIO 上的脉冲会触发 capture timer 保存计数值，并通过中断通知软件。([Espressif Systems][1])

在 BLDC 里，Capture 可以用于：

```text
霍尔信号测速
霍尔换相信号时间测量
外部编码器边沿时间戳
```

例如霍尔 BLDC 六步换向：

```text
Hall A/B/C 变化
        ↓
Capture/中断捕获边沿
        ↓
计算当前电角度区间
        ↓
切换三相桥状态
```

不过做高精度编码器，一般还会考虑 PCNT、专用编码器接口，或者外部编码器芯片。

---

# 10. Carrier：载波调制

Carrier 这个功能在普通 BLDC/FOC 里不一定用。

它主要用于把高频载波调制到 PWM 上，某些隔离驱动、变压器耦合驱动、特殊功率控制场景可能会用。ESP-IDF 文档也把 Carrier Modulation 描述为可以把高频 carrier 信号调制到 PWM waveform。([Espressif Systems][1])

你现在做 FOC，可以先忽略 Carrier。

---

# 11. MCPWM 和 LEDC 的区别

ESP32 里还有 LEDC，也能输出 PWM。但 LEDC 更适合：

```text
LED 调光
蜂鸣器
舵机
简单风扇控制
普通占空比输出
```

MCPWM 更适合：

```text
有刷电机 H 桥
BLDC 六步换向
FOC / SVPWM
三相逆变器
数字电源
需要互补 PWM + 死区 + 故障保护的场景
```

对比一下：

| 功能         | LEDC | MCPWM |
| ---------- | ---- | ----- |
| 普通 PWM     | 支持   | 支持    |
| 三相 PWM     | 不方便  | 很适合   |
| 互补输出       | 不适合  | 支持    |
| 死区时间       | 不适合  | 支持    |
| Fault 保护   | 不适合  | 支持    |
| Capture 输入 | 不适合  | 支持    |
| Timer 同步   | 较弱   | 支持    |
| FOC/SVPWM  | 不推荐  | 推荐    |

所以你做 BLDC/FOC，应该优先用：

```c
driver/mcpwm_prelude.h
```

而不是 LEDC。

ESP-IDF 当前 MCPWM 文档里也列出头文件属于 `esp_driver_mcpwm` 组件，并给出 `driver/mcpwm_timer.h` 等头文件说明；实际新版本常用 `driver/mcpwm_prelude.h` 统一包含。([Espressif Systems][1])

---

# 12. 用 MCPWM 做 FOC 时的典型配置

对三相 BLDC/PMSM FOC 来说，建议结构如下：

```text
MCPWM Group 0

Timer0：20kHz，中心对齐
    ↓
Operator0：U 相
    ├── Generator A：UH
    └── Generator B：UL

Operator1：V 相
    ├── Generator A：VH
    └── Generator B：VL

Operator2：W 相
    ├── Generator A：WH
    └── Generator B：WL
```

然后 FOC 算法每个控制周期更新：

```text
Comparator U
Comparator V
Comparator W
```

FOC 控制流程：

```text
读取角度 / 估算角度
        ↓
Clarke / Park
        ↓
电流环 PI / 速度环 PI
        ↓
反 Park
        ↓
SVPWM
        ↓
得到 duty_u / duty_v / duty_w
        ↓
更新 MCPWM comparator
        ↓
MCPWM 自动输出三相互补 PWM
```

官方 ESP-IDF 示例里也有 `mcpwm_foc_svpwm_open_loop`，用于生成三对 PWM 信号做 FOC，可以驱动 BLDC/PMSM 或三相逆变器。([Espressif Systems][1])

---

# 13. 一个关键点：占空比更新要“同步更新”

做 FOC 时不能随便在任意时刻改 PWM 占空比。

错误方式：

```text
U 相刚更新
V 相还没更新
W 相还没更新
```

这样会导致三相在同一个 PWM 周期里数据不一致，波形可能抖动。

正确方式是：

```text
本周期计算 duty_u / duty_v / duty_w
        ↓
写到 shadow register
        ↓
等下一个 PWM 周期起点统一生效
```

在 ESP-IDF v5 的 MCPWM driver 里，Comparator 可以配置类似：

```c
.update_cmp_on_tez = true
```

意思是比较值在 Timer Equal Zero，也就是周期起点时更新。这样适合 FOC。

你可以理解为：

```text
软件提前准备下一周期占空比
硬件在周期边界统一切换
```

这对三相电机控制很重要。

---

# 14. PWM 频率怎么选？

FOC/BLDC 常用 PWM 频率：

|      PWM 频率 | 特点             |
| ----------: | -------------- |
|        8kHz | 开关损耗低，但可能有可闻噪声 |
|       16kHz | 接近听觉上限，常见      |
|       20kHz | 常用，基本避开人耳      |
| 25kHz~40kHz | 噪声更小，但开关损耗更高   |

新手做开环 FOC，我建议从：

```text
20kHz PWM
```

开始。

周期是：

```text
1 / 20kHz = 50us
```

也就是说，每 50us 一个 PWM 周期。

如果你控制环也跑 20kHz，那么每 50us 算一次 FOC。ESP32 能算，但要注意不要在控制环里打印日志、malloc、访问 Flash、跑太重的 Wi-Fi/BLE 逻辑。

---

# 15. MCPWM 在 FOC 中真正负责什么？

这个要分清楚。

MCPWM 不负责算 FOC。

它不负责：

```text
Clarke 变换
Park 变换
PI 控制器
SVPWM 扇区判断
电角度计算
速度估算
电流采样滤波
```

这些都是 CPU 软件算的。

MCPWM 负责：

```text
按照你算出来的 duty_u / duty_v / duty_w
稳定、同步、安全地输出三相 PWM
```

所以 FOC 系统可以分成两层：

```text
软件算法层：
角度、电流、PI、SVPWM

硬件输出层：
MCPWM、死区、互补输出、故障保护
```

MCPWM 是 FOC 里的“执行器输出模块”。

---

# 16. 对你来说最重要的 MCPWM 概念

你现在学 FOC，先抓住这几个就够了：

```text
Timer       ：决定 PWM 频率
Comparator  ：决定占空比
Generator   ：决定 PWM 引脚什么时候高/低
Dead Time   ：防止上下桥直通
Operator    ：一相半桥的控制单元
Fault/Brake ：过流保护
Sync        ：三相同步
Capture     ：霍尔/测速输入
```

对于三相 BLDC：

```text
一个 Operator 控制一相
三个 Operator 控制三相
每个 Operator 输出 A/B 两路互补 PWM
```

也就是：

```text
Operator0 → U 相上下桥
Operator1 → V 相上下桥
Operator2 → W 相上下桥
```

---

# 17. 最后用一句话总结

**MCPWM 就是 ESP32 里专门给电机控制用的高级 PWM 外设。**

它相比普通 PWM 的核心优势是：

```text
三相输出方便
支持互补 PWM
支持死区时间
支持故障保护
支持同步更新
支持捕获输入
适合 SVPWM / FOC
```

所以你用 ESP32-WROOM-32-N8 做 BLDC/FOC，MCPWM 基本就是必选外设。下一步真正写代码时，你重点就是配置：

```text
1 个 Timer
3 个 Operator
3 组 Comparator
6 个 Generator
3 组 Dead Time
```

然后在 FOC 控制循环里不断更新：

```text
duty_u
duty_v
duty_w
```

[1]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html "Motor Control Pulse Width Modulator (MCPWM) - ESP32 -  — ESP-IDF Programming Guide v6.0.1 documentation"
[2]: https://docs.espressif.com/projects/esp-idf/en/v4.4.1/esp32/api-reference/peripherals/mcpwm.html "Motor Control Pulse Width Modulator (MCPWM) - ESP32 -  — ESP-IDF Programming Guide v4.4.1 documentation"
