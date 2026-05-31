/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 说明：
 * 1. 这个文件基于 ESP-IDF 的 FOC/SVPWM 示例改写。
 * 2. 原示例使用 IQmath 定点数学库，例如 _IQ()、_IQmpy()、_IQdiv2()、_IQtoF()。
 * 3. 这里已经全部替换成 float 浮点计算，方便新手理解 FOC 的数学过程。
 * 4. 这里仍然保留 esp_svpwm 里的“逆变器/MCPWM 输出封装”，因为它负责配置 MCPWM、互补 PWM、死区等硬件输出。
 * 5. 这里不再调用 foc/esp_foc.h 里面的定点 FOC 计算函数，而是自己写 float 版本的 Park 反变换、Clarke 反变换和 SVPWM 零序注入。
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

/*
 * 这里只保留 svpwm/esp_svpwm.h。
 * 注意：这个头文件名叫 svpwm，但在这个示例里主要用它创建 inverter，也就是三相逆变桥 PWM 输出对象。
 * 原来的 foc/esp_foc.h 不再包含，因为里面的坐标类型和计算函数会使用 IQmath 定点数。
 */
#include "svpwm/esp_svpwm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "example_foc_float";

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////// 下面这些 GPIO 要按照你的实际硬件原理图修改 ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * 栅极驱动芯片使能脚。
 * 很多三相驱动板会有一个 EN 引脚，用来总开关 MOSFET 驱动输出。
 * enable = true  时，允许驱动 MOSFET。
 * enable = false 时，关闭驱动输出，电机不再受控输出。
 */
#define EXAMPLE_FOC_DRV_EN_GPIO          46

/*
 * 故障检测脚。
 * 原始示例定义了这个 GPIO，但示例代码里没有真正读取它。
 * 后续你可以用 gpio_get_level() 读取它，用来判断驱动芯片是否过流、过温、欠压等。
 */
#define EXAMPLE_FOC_DRV_FAULT_GPIO       10

/*
 * 三相半桥的 6 路 PWM 引脚。
 * U/V/W 是三相电机的三根相线。
 * H 表示 High Side，上桥臂 MOS 管。
 * L 表示 Low Side，下桥臂 MOS 管。
 *
 * UH/UL 控制 U 相上下桥臂。
 * VH/VL 控制 V 相上下桥臂。
 * WH/WL 控制 W 相上下桥臂。
 */
#define EXAMPLE_FOC_PWM_UH_GPIO          47
#define EXAMPLE_FOC_PWM_UL_GPIO          21
#define EXAMPLE_FOC_PWM_VH_GPIO          14
#define EXAMPLE_FOC_PWM_VL_GPIO          13
#define EXAMPLE_FOC_PWM_WH_GPIO          12
#define EXAMPLE_FOC_PWM_WL_GPIO          11

/*
 * MCPWM 定时器分辨率。
 * 10 MHz 表示定时器 1 秒钟计数 10000000 次。
 * 所以 1 个 tick = 1 / 10000000 s = 0.1 us。
 */
#define EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ 10000000

/*
 * PWM 周期计数值。
 * period_ticks = 1000，配合 10 MHz 分辨率，半个计数周期约为 100 us。
 * 原始示例按 10 kHz 理解：10 MHz / 1000 = 10 kHz。
 * 注意：这里使用 UP_DOWN 中心对齐模式，具体中断事件频率和 PWM 完整周期频率可能与事件选择有关。
 */
#define EXAMPLE_FOC_MCPWM_PERIOD              1000

/*
 * 开环输出的电角频率。
 * 50 Hz 表示这里人为生成一个 50 Hz 的旋转电压矢量。
 * 这不是闭环 FOC，只是让电压矢量按固定速度旋转。
 */
#define EXAMPLE_FOC_WAVE_FREQ                 50.0f

/*
 * 输出电压矢量幅值。
 * 这里不是实际电压 V，而是 PWM 计数里的一个幅度值。
 * 值越大，输出占空比摆动越大，电机获得的等效电压越高。
 * 新手调试时建议先从小值开始，比如 30、50、80，避免电机大电流发热。
 */
#define EXAMPLE_FOC_WAVE_AMPL                 100.0f

/*
 * 这个值表示 PWM 比较值最大范围。
 * 原始代码把三相输出转换到 0 ~ EXAMPLE_FOC_MCPWM_PERIOD / 2。
 * 比如 period = 1000，则输出比较值范围是 0 ~ 500。
 */
#define EXAMPLE_FOC_PWM_DUTY_MAX              (EXAMPLE_FOC_MCPWM_PERIOD / 2)

/*
 * 这个宏表示 FOC 计算更新频率。
 * 原始示例使用 10 MHz / 1000 = 10 kHz 计算每次角度增加量。
 */
#define EXAMPLE_FOC_UPDATE_FREQ_HZ            ((float)EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ / (float)EXAMPLE_FOC_MCPWM_PERIOD)

/*
 * dq 坐标系。
 * d 轴：直轴，通常和转子磁链方向对齐。
 * q 轴：交轴，通常用来控制转矩。
 *
 * 在真正闭环 FOC 里：
 * - Id 通常控制磁链、弱磁或者设为 0。
 * - Iq 通常控制转矩。
 *
 * 这个开环例程没有采样电流，也没有编码器角度，所以这里可以理解成“电压 dq”，不是电流 dq。
 */
typedef struct {
    float d;
    float q;
} foc_dq_float_t;

/*
 * alpha-beta 坐标系。
 * 这是一个静止二维坐标系。
 * 三相 U/V/W 可以等效成 alpha-beta 平面上的一个矢量。
 */
typedef struct {
    float alpha;
    float beta;
} foc_ab_float_t;

/*
 * 三相坐标。
 * u/v/w 分别对应电机三相。
 * 这里的值最终会转换成三路 PWM 比较值。
 */
typedef struct {
    float u;
    float v;
    float w;
} foc_uvw_float_t;

/*
 * 简单限幅函数。
 * 作用：防止计算出来的 PWM 比较值超出允许范围。
 */
static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/*
 * 把电角度限制在 0 ~ 360 度范围内。
 * 角度一直累加会越来越大，所以每转一圈就减掉 360 度。
 */
static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/*
 * Park 反变换：dq -> alpha-beta。
 *
 * 新手理解：
 * dq 坐标系是一个“跟着电机磁场一起旋转”的坐标系。
 * alpha-beta 坐标系是一个“固定不动”的二维坐标系。
 *
 * 反 Park 变换就是：
 * 把旋转坐标系里的 d/q 分量，按照当前电角度 theta，转换成静止坐标系里的 alpha/beta 分量。
 *
 * 公式：
 * alpha = d * cos(theta) - q * sin(theta)
 * beta  = d * sin(theta) + q * cos(theta)
 *
 * 原始 IQmath 写法大概是：
 * elec_theta_rad = _IQmpy(_IQ(elec_theta_deg), _IQ(M_PI / 180.f));
 * foc_inverse_park_transform(elec_theta_rad, &dq_out, &ab_out);
 *
 * 现在改成 float：
 * theta_rad = elec_theta_deg * M_PI / 180.0f;
 * 然后直接 sinf/cosf 计算。
 */
static void foc_inverse_park_transform_float(float theta_rad,
                                             const foc_dq_float_t *dq,
                                             foc_ab_float_t *ab)
{
    float sin_theta = sinf(theta_rad);
    float cos_theta = cosf(theta_rad);

    ab->alpha = dq->d * cos_theta - dq->q * sin_theta;
    ab->beta  = dq->d * sin_theta + dq->q * cos_theta;
}

/*
 * Clarke 反变换：alpha-beta -> u/v/w。
 *
 * 新手理解：
 * alpha-beta 是二维等效电压矢量。
 * 但是电机实际需要 U/V/W 三相电压。
 * 所以要把二维矢量重新分解成三相正弦波。
 *
 * 常用公式：
 * u = alpha
 * v = -0.5 * alpha + sqrt(3) / 2 * beta
 * w = -0.5 * alpha - sqrt(3) / 2 * beta
 *
 * 这个输出方式可以理解成普通 SPWM 的三相正弦调制。
 */
static void foc_inverse_clarke_transform_float(const foc_ab_float_t *ab,
                                               foc_uvw_float_t *uvw)
{
    const float sqrt3_div_2 = 0.86602540378f;

    uvw->u = ab->alpha;
    uvw->v = -0.5f * ab->alpha + sqrt3_div_2 * ab->beta;
    uvw->w = -0.5f * ab->alpha - sqrt3_div_2 * ab->beta;
}

/*
 * 简化版 float SVPWM：零序注入法。
 *
 * 新手理解：
 * 1. 先用 Clarke 反变换得到三相 u/v/w。
 * 2. 找到三相里的最大值 max 和最小值 min。
 * 3. 给三相一起加一个公共偏移量 offset。
 * 4. 这个公共偏移不会改变线电压，但可以让 PWM 利用率更高。
 *
 * 这就是 SVPWM 常见实现思路之一，也叫零序电压注入。
 * 它比纯 SPWM 更能利用母线电压。
 */
static void foc_svpwm_zero_sequence_float(const foc_ab_float_t *ab,
                                          foc_uvw_float_t *uvw)
{
    foc_inverse_clarke_transform_float(ab, uvw);

    float max_value = fmaxf(fmaxf(uvw->u, uvw->v), uvw->w);
    float min_value = fminf(fminf(uvw->u, uvw->v), uvw->w);

    /*
     * 公共零序偏移。
     * 三相都加同一个值，不影响三相之间的线电压差，
     * 但是能把波形更好地放进 PWM 可输出范围。
     */
    float zero_sequence_offset = -0.5f * (max_value + min_value);

    uvw->u += zero_sequence_offset;
    uvw->v += zero_sequence_offset;
    uvw->w += zero_sequence_offset;
}

/*
 * 把 u/v/w 的浮点计算结果转换成 MCPWM 比较值。
 *
 * 原始 IQmath 代码是：
 * uvw_duty[0] = _IQtoF(_IQdiv2(uvw_out.u)) + (EXAMPLE_FOC_MCPWM_PERIOD / 4);
 *
 * 对应的 float 写法就是：
 * duty = uvw / 2 + period / 4
 *
 * 例如：
 * period = 1000，period / 4 = 250。
 * 如果 uvw = 0，则 duty = 250，中间值。
 * 如果 uvw = 100，则 duty = 300。
 * 如果 uvw = -100，则 duty = 200。
 */
static int phase_value_to_pwm_duty(float phase_value)
{
    float duty_f = (phase_value * 0.5f) + ((float)EXAMPLE_FOC_MCPWM_PERIOD / 4.0f);

    /* 四舍五入成整数 tick。 */
    int duty = (int)(duty_f + 0.5f);

    /* 防止占空比越界。 */
    return clamp_int(duty, 0, EXAMPLE_FOC_PWM_DUTY_MAX);
}

/*
 * 初始化栅极驱动芯片 EN 引脚。
 */
void bsp_bridge_driver_init(void)
{
    gpio_config_t drv_en_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_FOC_DRV_EN_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
}

/*
 * 打开或者关闭三相驱动输出。
 */
void bsp_bridge_driver_enable(bool enable)
{
    ESP_LOGI(TAG, "%s MOSFET gate", enable ? "Enable" : "Disable");
    gpio_set_level(EXAMPLE_FOC_DRV_EN_GPIO, enable);
}

/*
 * MCPWM 定时器更新回调函数。
 *
 * 新手理解：
 * PWM 是周期性输出的。
 * 每到一个固定的 PWM 时刻，就通知主循环：该计算下一次三相占空比了。
 *
 * ISR 表示中断环境。
 * 中断里不要做复杂计算，所以这里只释放一个信号量。
 * 真正的 FOC 数学计算放在 while 循环里做。
 */
bool inverter_update_cb(mcpwm_timer_handle_t timer,
                        const mcpwm_timer_event_data_t *edata,
                        void *user_ctx)
{
    BaseType_t task_yield = pdFALSE;

    /* user_ctx 里传进来的是 update_semaphore 的地址。 */
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t *)user_ctx), &task_yield);

    /* 如果释放信号量后唤醒了更高优先级任务，就请求切换任务。 */
    return task_yield;
}


extern void vfoc_init(void);
extern void vfoc_open_loop_spwm_run(float target_rpm, float uq, float vbus, float dt_s);

void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC float version");

    /*
     * 创建一个计数信号量。
     * 作用：让 MCPWM 定时器中断和主循环同步。
     * 中断每来一次，给一次信号量；主循环拿到信号量后，计算一次新的 PWM 占空比。
     */
    SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);
    if (update_semaphore == NULL) {
        ESP_LOGE(TAG, "Create update semaphore failed");
        return;
    }

    vfoc_init();
    vfoc_open_loop_spwm_run( 0.0f,
                            0.0f,
                            0.0f,
                            0.0f
    );

//     /*
//      * dq_out：人为给定的旋转电压矢量。
//      * ab_out：Park 反变换之后的 alpha/beta 电压矢量。
//      * uvw_out：最终转换成三相 U/V/W 的电压指令。
//      */
//     foc_dq_float_t dq_out = {0.0f, 0.0f};
//     foc_ab_float_t ab_out = {0.0f, 0.0f};
//     foc_uvw_float_t uvw_out = {0.0f, 0.0f, 0.0f};

//     int uvw_duty[3] = {0, 0, 0};

//     /* 当前电角度，单位：度。 */
//     float elec_theta_deg = 0.0f;

//     /* 当前电角度，单位：弧度。sinf/cosf 使用弧度。 */
//     float elec_theta_rad = 0.0f;

//     /*
//      * 配置三相逆变器 PWM。
//      * 这个 inverter_config_t 来自 esp_svpwm 组件。
//      * 它会帮你配置 MCPWM timer、operator、comparator、generator、deadtime 等。
//      */
//     inverter_config_t cfg = {
//         .timer_config = {
//             .group_id = 0,
//             .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//             .resolution_hz = EXAMPLE_FOC_MCPWM_TIMER_RESOLUTION_HZ,

//             /*
//              * UP_DOWN 是中心对齐 PWM。
//              * 中心对齐 PWM 相比边沿对齐 PWM，通常电磁噪声和谐波表现更好，
//              * 在电机控制里很常用。
//              */
//             .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
//             .period_ticks = EXAMPLE_FOC_MCPWM_PERIOD,
//         },
//         .operator_config = {
//             .group_id = 0,
//         },
//         .compare_config = {
//             /*
//              * update_cmp_on_tez = true：
//              * 在定时器计数到 0 的事件更新比较值。
//              * 这样可以避免 PWM 周期中间突然改占空比造成毛刺。
//              */
//             .flags.update_cmp_on_tez = true,
//         },
//         .gen_gpios = {
//             {EXAMPLE_FOC_PWM_UH_GPIO, EXAMPLE_FOC_PWM_UL_GPIO},
//             {EXAMPLE_FOC_PWM_VH_GPIO, EXAMPLE_FOC_PWM_VL_GPIO},
//             {EXAMPLE_FOC_PWM_WH_GPIO, EXAMPLE_FOC_PWM_WL_GPIO},
//         },
//         .dt_config = {
//             /*
//              * 上升沿死区。
//              * 死区用于避免同一相的上下桥臂同时导通，防止直通烧 MOS。
//              */
//             .posedge_delay_ticks = 5,
//         },
//         .inv_dt_config = {
//             /*
//              * 下降沿死区，同时反相输出下桥臂 PWM。
//              * 这样可以得到互补 PWM。
//              */
//             .negedge_delay_ticks = 5,
//             .flags.invert_output = true,
//         },
//     };

//     inverter_handle_t inverter1;
//     ESP_ERROR_CHECK(svpwm_new_inverter(&cfg, &inverter1));
//     ESP_LOGI(TAG, "Inverter init OK");

//     /*
//      * 注册 MCPWM 定时器回调。
//      * on_full 表示定时器计数到峰值时触发。
//      */
//     mcpwm_timer_event_callbacks_t cbs = {
//         .on_full = inverter_update_cb,
//     };
//     ESP_ERROR_CHECK(svpwm_inverter_register_cbs(inverter1, &cbs, &update_semaphore));

//     /* 启动 MCPWM。 */
//     ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_START_NO_STOP));
//     ESP_LOGI(TAG, "Inverter start OK");

//     /* 打开栅极驱动芯片，使能三相 MOSFET 驱动输出。 */
//     bsp_bridge_driver_init();
//     bsp_bridge_driver_enable(true);

//     ESP_LOGI(TAG, "Start open-loop FOC float calculation");

//     while (true) {
//         /*
//          * 等待 MCPWM 回调释放信号量。
//          * 没有信号量时，这个任务会阻塞，不会一直空转浪费 CPU。
//          */
//         xSemaphoreTake(update_semaphore, portMAX_DELAY);

//         /*
//          * 计算每次更新电角度要增加多少度。
//          *
//          * 目标：输出 50 Hz 的旋转电压矢量。
//          * 一圈是 360 度。
//          * 如果每秒更新 10000 次，那么每次增加：
//          * 50 * 360 / 10000 = 1.8 度。
//          */
//         elec_theta_deg += (EXAMPLE_FOC_WAVE_FREQ * 360.0f) / EXAMPLE_FOC_UPDATE_FREQ_HZ;
//         elec_theta_deg = wrap_angle_deg(elec_theta_deg);

//         /* 角度转弧度：弧度 = 角度 * pi / 180。 */
//         elec_theta_rad = elec_theta_deg * ((float)M_PI / 180.0f);

//         /*
//          * 开环 FOC 电压指令。
//          *
//          * 原始示例：
//          * dq_out.d = _IQ(EXAMPLE_FOC_WAVE_AMPL);
//          * q 默认为 0。
//          *
//          * 这里改成 float：
//          * d = EXAMPLE_FOC_WAVE_AMPL;
//          * q = 0;
//          *
//          * 注意：
//          * 这不是闭环电流控制，也没有用编码器角度。
//          * 它只是生成一个固定频率旋转的电压矢量，让电机按开环方式尝试转起来。
//          */
//         dq_out.d = EXAMPLE_FOC_WAVE_AMPL;
//         dq_out.q = 0.0f;

//         /* dq -> alpha-beta。 */
//         foc_inverse_park_transform_float(elec_theta_rad, &dq_out, &ab_out);

// #if CONFIG_ESP_FOC_USE_SVPWM
//         /*
//          * 使用 float 版本 SVPWM。
//          * 这里没有调用 IQmath 版本的 foc_svpwm_duty_calculate()。
//          */
//         foc_svpwm_zero_sequence_float(&ab_out, &uvw_out);
// #else
//         /*
//          * 使用 float 版本 SPWM。
//          * 这里没有调用 IQmath 版本的 foc_inverse_clarke_transform()。
//          */
//         foc_inverse_clarke_transform_float(&ab_out, &uvw_out);
// #endif

//         /* 把三相计算值转换成 MCPWM duty tick。 */
//         uvw_duty[0] = phase_value_to_pwm_duty(uvw_out.u);
//         uvw_duty[1] = phase_value_to_pwm_duty(uvw_out.v);
//         uvw_duty[2] = phase_value_to_pwm_duty(uvw_out.w);

//         /*
//          * 输出三相 PWM 占空比。
//          * 这里的 duty 不是百分比，而是 MCPWM 比较值 tick。
//          */
//         ESP_ERROR_CHECK(svpwm_inverter_set_duty(inverter1,
//                                                 uvw_duty[0],
//                                                 uvw_duty[1],
//                                                 uvw_duty[2]));
//     }

//     /*
//      * 理论上 while(true) 不会退出。
//      * 如果以后你加了退出条件，可以走到这里关闭输出并释放资源。
//      */
//     bsp_bridge_driver_enable(false);
//     ESP_ERROR_CHECK(svpwm_inverter_start(inverter1, MCPWM_TIMER_STOP_EMPTY));
//     ESP_ERROR_CHECK(svpwm_del_inverter(inverter1));
}
