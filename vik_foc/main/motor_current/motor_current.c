/**
 * @file motor_current.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "motor_current.h"




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
#include <math.h>
#include <float.h>

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
#define CURRENT_ADC_UNIT ADC_UNIT_1

/* ESP32只使用ADC1 */
#define CURRENT_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1

/* 使用较大衰减，允许测量接近3.3V的电压 */
#define CURRENT_ADC_ATTEN ADC_ATTEN_DB_12

/* 使用芯片支持的最大ADC位宽，ESP32通常为12位 */
#define CURRENT_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

/* 一共有4路电流采样 */
#define CURRENT_ADC_CHANNEL_NUM 4

/*
 * ADC总采样率。
 *
 * ADC会依次轮询4个通道，因此：
 *
 * 80000次/秒 ÷ 4路 = 每路约20000次/秒
 */
#define CURRENT_ADC_TOTAL_SAMPLE_HZ (80 * 1000)

/* 每次从DMA读取的字节数 */
#define CURRENT_ADC_READ_LEN 256

/* ADC驱动内部缓冲区大小 */
#define CURRENT_ADC_STORE_BUFFER_SIZE 2048

/* 每路零点校准采样次数 */
#define CURRENT_ZERO_SAMPLE_NUM 2000

/* 每隔100ms打印一次 */
#define CURRENT_REPORT_PERIOD_US (100 * 1000)

/* ESP32 ADC1总共有8个通道：0～7 */
#define ESP32_ADC1_CHANNEL_MAX 8

/*----------------------------------------------------------
 * INA240A2配置
 *----------------------------------------------------------*/

/* INA240A2增益为50V/V */
#define INA240_GAIN 50.0f

/* 电流采样电阻为10mΩ */
#define CURRENT_SHUNT_RESISTOR_OHM 0.010f

/*
 * 输出灵敏度：
 *
 * 50 × 0.01Ω = 0.5V/A
 *
 * 即：
 * 500mV/A
 */
#define INA240_MV_PER_AMP \
    (INA240_GAIN * CURRENT_SHUNT_RESISTOR_OHM * 1000.0f)



/*----------------------------------------------------------
 * 电流采样任务配置
 *----------------------------------------------------------*/

/* ESP-IDF中的任务栈大小单位是字节 */
#define CURRENT_ADC_TASK_STACK_SIZE    (8 * 1024)

/* ADC读取任务优先级 */
#define CURRENT_ADC_TASK_PRIORITY      8

/* 固定运行在CPU0 */
#define CURRENT_ADC_TASK_CORE          0



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

/* 运行时ADC原始值统计 */
static uint64_t s_report_sum[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_report_count[ESP32_ADC1_CHANNEL_MAX];

/*
 * 运行时电流统计。
 *
 * 不能只对有符号电流求平均，因为电机相电流正负交替时，
 * 正负值会互相抵消，看起来会接近0A。
 */
static float s_current_latest[ESP32_ADC1_CHANNEL_MAX];      /* 最新一次瞬时电流 */
static float s_current_sum[ESP32_ADC1_CHANNEL_MAX];         /* 有符号电流累计 */
static float s_current_abs_sum[ESP32_ADC1_CHANNEL_MAX];     /* 电流绝对值累计 */
static float s_current_square_sum[ESP32_ADC1_CHANNEL_MAX];  /* 电流平方累计 */
static float s_current_min[ESP32_ADC1_CHANNEL_MAX];         /* 统计周期内最小值 */
static float s_current_max[ESP32_ADC1_CHANNEL_MAX];         /* 统计周期内最大值 */

/*----------------------------------------------------------
 * 四路ADC通道
 *----------------------------------------------------------*/

static const adc_channel_t current_adc_channels[CURRENT_ADC_CHANNEL_NUM] = {
    ADC_CHANNEL_0, /* GPIO36：M0_CS2 */
    ADC_CHANNEL_3, /* GPIO39：M0_CS1 */
    ADC_CHANNEL_6, /* GPIO34：M1_CS2 */
    ADC_CHANNEL_7, /* GPIO35：M1_CS1 */
};

/* ADC中断通知的任务句柄 */
static TaskHandle_t motor_current_adc_task_handle = NULL;

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
    switch (channel)
    {
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
    if (current_adc_cali_enable)
    {
        esp_err_t ret = adc_cali_raw_to_voltage(
            current_adc_cali_handle,
            raw,
            &voltage_mv);

        if (ret == ESP_OK)
        {
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
        &current_adc_cali_handle);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC线性拟合校准初始化成功");
        return true;
    }

    ESP_LOGW(
        TAG,
        "ADC校准初始化失败：%s，后续使用理论电压换算",
        esp_err_to_name(ret));

#else

    ESP_LOGW(TAG, "当前芯片或IDF不支持ADC线性拟合校准");

#endif

    return false;
}

/*----------------------------------------------------------
 * ADC转换完成回调
 *----------------------------------------------------------*/

static bool IRAM_ATTR current_adc_conv_done_cb( adc_continuous_handle_t adc_handle,
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
    if (motor_current_adc_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(
            motor_current_adc_task_handle,
            &must_yield
        );
    }

    return must_yield == pdTRUE;
}

/*----------------------------------------------------------
 * ADC连续模式初始化
 *----------------------------------------------------------*/

static void current_adc_continuous_init( adc_continuous_handle_t *out_handle )
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
        /*
         * 四路总采样率80kHz，
         * 平均每路约20kHz。
         */
        .sample_freq_hz = CURRENT_ADC_TOTAL_SAMPLE_HZ,

        /*
         * 只使用ADC1。
         */
        .conv_mode = CURRENT_ADC_CONV_MODE,

        /*
         * 经典ESP32使用TYPE1格式。
         */
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[CURRENT_ADC_CHANNEL_NUM] = {0};

    adc_config.pattern_num = CURRENT_ADC_CHANNEL_NUM;

    for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++)
    {
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
            current_adc_channels[i]);
    }

    adc_config.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(
        adc_continuous_config(
            adc_handle,
            &adc_config));

    *out_handle = adc_handle;
}

/*----------------------------------------------------------
 * 判断所有通道是否完成零点校准
 *----------------------------------------------------------*/

static bool current_adc_zero_calibration_finished(
    const uint32_t s_zero_count[ESP32_ADC1_CHANNEL_MAX])
{
    for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++)
    {
        adc_channel_t channel = current_adc_channels[i];

        if (s_zero_count[channel] < CURRENT_ZERO_SAMPLE_NUM)
        {
            return false;
        }
    }

    return true;
}


/*----------------------------------------------------------
 * 清空一个统计周期的电流数据
 *----------------------------------------------------------*/

static void current_adc_report_stats_reset(void)
{
    memset(s_report_sum, 0, sizeof(s_report_sum));
    memset(s_report_count, 0, sizeof(s_report_count));

    memset(s_current_latest, 0, sizeof(s_current_latest));
    memset(s_current_sum, 0, sizeof(s_current_sum));
    memset(s_current_abs_sum, 0, sizeof(s_current_abs_sum));
    memset(s_current_square_sum, 0, sizeof(s_current_square_sum));

    /*
     * 最小值初始设为很大的正数，最大值初始设为很小的负数。
     * 收到第一个样本以后，它们就会被真实电流值替换。
     */
    for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++)
    {
        adc_channel_t channel = current_adc_channels[i];

        s_current_min[channel] = FLT_MAX;
        s_current_max[channel] = -FLT_MAX;
    }
}




/**
 * @brief INA240电流采样任务
 *
 * 该任务负责：
 * 1. 初始化ADC校准；
 * 2. 初始化ADC连续采样；
 * 3. 接收ADC转换完成通知；
 * 4. 读取DMA数据；
 * 5. 完成零电流校准；
 * 6. 计算并打印四路电流。
 *
 * @param arg 任务参数，当前没有使用
 */
static void motor_current_adc_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;

    /* 标记四路电流零点是否校准完成 */
    bool zero_calibration_done = false;

    /* 上一次打印电流数据的时间 */
    int64_t last_report_time_us = esp_timer_get_time();

    /* 当前任务就是ADC中断需要通知的任务 */
    motor_current_adc_task_handle = xTaskGetCurrentTaskHandle();

    /*
     * 清空静态数据。
     *
     * 因为这些变量位于静态区，不占当前任务栈。
     */
    memset(s_adc_result, 0, sizeof(s_adc_result));

    memset(s_zero_sum, 0, sizeof(s_zero_sum));
    memset(s_zero_count, 0, sizeof(s_zero_count));
    memset(s_zero_raw, 0, sizeof(s_zero_raw));
    memset(s_zero_voltage_mv, 0, sizeof(s_zero_voltage_mv));

    current_adc_report_stats_reset();

    ESP_LOGI(
        TAG,
        "电流采样任务启动，剩余栈：%u字节",
        (unsigned int)uxTaskGetStackHighWaterMark(NULL)
    );

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
     *
     * ADC转换完成后，中断回调会通知当前任务。
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
     * 启动ADC连续采样。
     */
    ESP_ERROR_CHECK(
        adc_continuous_start(adc_handle)
    );

    ESP_LOGW(TAG, "开始四路电流零点校准");
    ESP_LOGW(TAG, "校准期间必须关闭电机PWM，保证相电流为0A");

    while (1)
    {
        /*
         * 等待ADC转换完成回调发送任务通知。
         *
         * 没有ADC数据时，任务会阻塞在这里，
         * 不会一直占用CPU。
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 一次通知到来后，尽量把ADC内部缓冲区的数据全部读完。
         */
        while (1)
        {
            ret = adc_continuous_read(
                adc_handle,
                s_adc_result,
                sizeof(s_adc_result),
                &ret_num,
                0
            );

            /*
             * 当前已经没有数据可以读取。
             */
            if (ret == ESP_ERR_TIMEOUT)
            {
                break;
            }

            /*
             * ADC读取发生其他错误。
             */
            if (ret != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "读取ADC失败：%s",
                    esp_err_to_name(ret)
                );

                break;
            }

            /*
             * ESP32连续ADC使用TYPE1格式。
             *
             * 每SOC_ADC_DIGI_RESULT_BYTES字节，
             * 表示一次ADC转换结果。
             */
            for (uint32_t offset = 0;
                 offset < ret_num;
                 offset += SOC_ADC_DIGI_RESULT_BYTES)
            {
                /*
                 * 将DMA数据转换为ESP32 ADC结果结构体。
                 */
                const adc_digi_output_data_t *adc_data =
                    (const adc_digi_output_data_t *)
                    &s_adc_result[offset];

                /*
                 * 从TYPE1格式中提取ADC通道和原始值。
                 */
                uint32_t channel =
                    adc_data->type1.channel;

                uint32_t raw =
                    adc_data->type1.data;

                /*
                 * ESP32 ADC1只有通道0～7。
                 */
                if (channel >= ESP32_ADC1_CHANNEL_MAX)
                {
                    continue;
                }

                /*
                 * 零点校准阶段。
                 */
                if (!zero_calibration_done)
                {
                    /*
                     * 每路采集CURRENT_ZERO_SAMPLE_NUM个样本。
                     */
                    if (s_zero_count[channel] <
                        CURRENT_ZERO_SAMPLE_NUM)
                    {
                        s_zero_sum[channel] += raw;
                        s_zero_count[channel]++;
                    }
                }
                else
                {
                    /*
                     * 正常运行阶段：每一个ADC样本都先换算成有符号电流，
                     * 再分别统计瞬时值、平均值、最小值、最大值、
                     * 绝对值平均值和RMS有效值。
                     */
                    int voltage_mv = current_adc_raw_to_mv(raw);

                    float current_a =
                        ((float)voltage_mv -
                         (float)s_zero_voltage_mv[channel]) /
                        INA240_MV_PER_AMP;

                    /* 保存ADC平均值所需的原始数据 */
                    s_report_sum[channel] += raw;
                    s_report_count[channel]++;

                    /* 最新一次采样值，作为“瞬时电流”显示 */
                    s_current_latest[channel] = current_a;

                    /* 有符号平均电流累计，正负值可能互相抵消 */
                    s_current_sum[channel] += current_a;

                    /* 绝对值平均，适合观察正负交替的相电流大小 */
                    s_current_abs_sum[channel] += fabsf(current_a);

                    /* 平方累计，用于计算RMS有效值 */
                    s_current_square_sum[channel] +=
                        current_a * current_a;

                    /* 记录统计周期内的最小、最大瞬时电流 */
                    if (current_a < s_current_min[channel])
                    {
                        s_current_min[channel] = current_a;
                    }

                    if (current_a > s_current_max[channel])
                    {
                        s_current_max[channel] = current_a;
                    }
                }
            }

            /*
             * 检查四路通道是否全部完成零点采样。
             */
            if (!zero_calibration_done &&
                current_adc_zero_calibration_finished(
                    s_zero_count))
            {
                for (int i = 0;
                     i < CURRENT_ADC_CHANNEL_NUM;
                     i++)
                {
                    adc_channel_t channel =
                        current_adc_channels[i];

                    /*
                     * 计算该通道零电流时的ADC平均值。
                     */
                    s_zero_raw[channel] =
                        s_zero_sum[channel] /
                        s_zero_count[channel];

                    /*
                     * 将零点ADC值转换成毫伏。
                     */
                    s_zero_voltage_mv[channel] =
                        current_adc_raw_to_mv(
                            s_zero_raw[channel]
                        );

                    ESP_LOGI(
                        TAG,
                        "%s零点：raw=%" PRIu32
                        "，voltage=%dmV",
                        current_adc_get_name(channel),
                        s_zero_raw[channel],
                        s_zero_voltage_mv[channel]
                    );
                }

                zero_calibration_done = true;

                /*
                 * 清空运行阶段的统计数据，
                 * 从零点校准完成后重新开始一个统计周期。
                 */
                current_adc_report_stats_reset();

                last_report_time_us =
                    esp_timer_get_time();

                ESP_LOGI(TAG, "四路电流零点校准完成");
            }
        }

        /*
         * 每100ms计算一次平均电流。
         */
        int64_t now_time_us =
            esp_timer_get_time();

        if (zero_calibration_done &&
            now_time_us - last_report_time_us >=
                CURRENT_REPORT_PERIOD_US)
        {
            last_report_time_us =
                now_time_us;

            for (int i = 0;
                 i < CURRENT_ADC_CHANNEL_NUM;
                 i++)
            {
                adc_channel_t channel =
                    current_adc_channels[i];

                /*
                 * 当前通道没有新数据时跳过。
                 */
                if (s_report_count[channel] == 0)
                {
                    continue;
                }

                /*
                 * 计算100ms内的ADC平均值。
                 */
                uint32_t average_raw =
                    s_report_sum[channel] /
                    s_report_count[channel];

                /*
                 * 将ADC原始值转换成毫伏。
                 */
                int voltage_mv =
                    current_adc_raw_to_mv(
                        average_raw
                    );

                uint32_t sample_count =
                    s_report_count[channel];

                /* 有符号平均值，交流相电流正负可能互相抵消 */
                float current_avg =
                    s_current_sum[channel] /
                    (float)sample_count;

                /* 电流绝对值平均，能反映正负交替电流的平均幅度 */
                float current_abs_avg =
                    s_current_abs_sum[channel] /
                    (float)sample_count;

                /* RMS有效值，能反映电流的发热和实际能量大小 */
                float current_rms =
                    sqrtf(
                        s_current_square_sum[channel] /
                        (float)sample_count
                    );

                ESP_LOGI(
                    TAG,
                    "%s: n=%" PRIu32
                    ",raw_avg=%" PRIu32
                    ",voltage_avg=%dmV"
                    ",instant=%.3fA"
                    ",avg=%.3fA"
                    ",min=%.3fA"
                    ",max=%.3fA"
                    ",abs_avg=%.3fA"
                    ",rms=%.3fA",
                    current_adc_get_name(channel),
                    sample_count,
                    average_raw,
                    voltage_mv,
                    s_current_latest[channel],
                    current_avg,
                    s_current_min[channel],
                    s_current_max[channel],
                    current_abs_avg,
                    current_rms
                );

                /*
                 * 清空当前通道统计数据，
                 * 开始下一个100ms统计周期。
                 */
                s_report_sum[channel] = 0;
                s_report_count[channel] = 0;
                s_current_latest[channel] = 0.0f;
                s_current_sum[channel] = 0.0f;
                s_current_abs_sum[channel] = 0.0f;
                s_current_square_sum[channel] = 0.0f;
                s_current_min[channel] = FLT_MAX;
                s_current_max[channel] = -FLT_MAX;
            }

            /*
             * 定期检查任务剩余栈。
             */
            ESP_LOGD(
                TAG,
                "电流任务剩余栈：%u字节",
                (unsigned int)
                uxTaskGetStackHighWaterMark(NULL)
            );
        }
    }

    /*
     * 正常情况下不会运行到这里。
     */
    ESP_ERROR_CHECK(
        adc_continuous_stop(adc_handle)
    );

    ESP_ERROR_CHECK(
        adc_continuous_deinit(adc_handle)
    );

    motor_current_adc_task_handle = NULL;

    vTaskDelete(NULL);
}

/**
 * @brief 
 * motor_get_current_main
   │
   └── 创建 motor_current_adc_task
              │
              ├── 初始化ADC校准
              ├── 初始化ADC连续模式
              ├── 启动ADC DMA
              ├── 等待中断通知
              ├── 读取四路ADC
              ├── 零电流校准
              └── 计算INA240相电流
 * 
 */
void motor_get_current_main(void)
{
    BaseType_t task_ret;

    /*
     * 创建独立的电流采样任务。
     *
     * ADC初始化、DMA读取和电流换算，
     * 全部在current_adc_task中完成。
     */
    task_ret = xTaskCreatePinnedToCore(
        motor_current_adc_task,                 /* 任务入口函数 */
        "motor_current_adc_task",               /* 任务名称 */
        CURRENT_ADC_TASK_STACK_SIZE,      /* 任务栈大小 */
        NULL,                             /* 任务参数 */
        CURRENT_ADC_TASK_PRIORITY,        /* 任务优先级 */
        &motor_current_adc_task_handle,         /* 保存任务句柄 */
        CURRENT_ADC_TASK_CORE             /* 固定CPU核心 */
    );

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建电流采样任务失败");

        /*
         * 创建失败后停止程序，
         * 防止后续代码在没有电流采样的情况下运行。
         */
        abort();
    }

    ESP_LOGI(
        TAG,
        "电流采样任务创建成功，任务句柄=%p",
        motor_current_adc_task_handle
    );

    /*
     * app_main执行结束没有问题。
     *
     * ESP-IDF的main_task会继续完成自己的退出处理，
     * 电流采样任务独立运行。
     */
}