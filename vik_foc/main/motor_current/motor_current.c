/**
 * @file motor_current.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * ESP32 + INA240A2 双电机四路相电流采样
 *
 * M0_CS2 -> GPIO36 -> ADC1_CH0
 * M0_CS1 -> GPIO39 -> ADC1_CH3
 * M1_CS2 -> GPIO34 -> ADC1_CH6
 * M1_CS1 -> GPIO35 -> ADC1_CH7
 * @version 0.1
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "app_rtos_config.h"
#include "app_rtos_resource.h"
#include "vik_foc.h"
#include "motor_power.h"



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

/* 每路零漂校准采样次数 */
#define CURRENT_ZERO_SAMPLE_NUM 500

/* 每隔100ms打印一次 */
#define CURRENT_REPORT_PERIOD_US (1000 * 1000)

/* ESP32 ADC1总共有8个通道：0～7 */
#define ESP32_ADC1_CHANNEL_MAX 8

/*----------------------------------------------------------
 * INA240A2配置
 Vout = Vref + G*Vin
 Vout:输出的放大信号，ADC检测到的值
 Vref:零点时的电压值，电路设计的是Vref:1.65V
 G:运算放大器的增益值
 Vin:被测电压相线的电压值

求电机相线的电流值就用Vin/R(电阻值)
I = Vin/Rsen
*----------------------------------------------------------*/

/* INA240A2增益为50V/V */
#define INA240_GAIN 50.0f

/* 电流采样电阻为10mΩ,0.01欧姆 */
#define CURRENT_SHUNT_RESISTOR_OHM 0.010f

/*
 * 输出灵敏度：
 *
 * 50 × 0.01Ω = 0.5V/A
 *
 * 即：
 * 500mV/A
 * 乘1000将值转化为v
 */
#define INA240_MV_PER_AMP \
    (INA240_GAIN * CURRENT_SHUNT_RESISTOR_OHM * 1000.0f)

/**
 10kHz
alpha = 0.25f

12kHz
alpha = 0.28f

15kHz
alpha = 0.32f

18kHz
alpha = 0.35f

20kHz
alpha = 0.40f
 * 
 */
#define CURRENT_FILTER_ALPHA 0.25f



/*----------------------------------------------------------
 * 电流采样任务配置
 *----------------------------------------------------------*/


/*电机0的相电流采集的ADC通道*/
#define MOTOR0_ADC_CH_IA        ADC_CHANNEL_0 /* GPIO36：M0_CS2 */
#define MOTOR0_ADC_CH_IB        ADC_CHANNEL_3 /* GPIO39：M0_CS1 */
#define MOTOR0_ADC_CH_IC        

#define MOTOR1_ADC_CH_IA        ADC_CHANNEL_6 /* GPIO34：M1_CS2 */
#define MOTOR2_ADC_CH_IB        ADC_CHANNEL_7 /* GPIO35：M1_CS1 */
#define MOTOR3_ADC_CH_IC        

/**
 * @brief 
 * 
// ADC_CHANNEL_0, // GPIO36：M0_CS2 
// ADC_CHANNEL_3, // GPIO39：M0_CS1
// ADC_CHANNEL_6, // GPIO34：M1_CS2
// ADC_CHANNEL_7, // GPIO35：M1_CS1
 * 
 */
typedef struct 
{
    float mtor_ia_curent;/*换算后的电机相线的电流值,单位A*/
    float mtor_ib_curent;
    float mtor_ic_curent;

    float mtor_zero_ua;/*电机静止时候零点的电压值Vref ,单位:mv*/
    float mtor_zero_ub;/*电机静止时候零点的电压值Vref*/
    float mtor_zero_uc;/*电机静止时候零点的电压值Vref*/

    float ia_shunt_mv;/*电机相线的电压值,单位:mv*/
    float ib_shunt_mv;
    float ic_shunt_mv;

}adc_motor_current_t;


typedef struct
{
    float ia;
    float ib;
    float ic;

    uint32_t sample_index;

    int64_t timestamp;

}motor_current_frame_t;

adc_motor_current_t adc_m0_val = {0};
adc_motor_current_t adc_m1_val = {0};

int64_t get_curent_time_stamp;


/*
 * 大数组必须放到静态存储区，
 * 不要全部压在app_main任务栈中。
 */
static uint8_t s_adc_dma_buf_result[CURRENT_ADC_READ_LEN];

/* 零飘电流校准统计 */
static uint64_t s_zero_sum[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_zero_count[ESP32_ADC1_CHANNEL_MAX];
static uint32_t s_zero_raw[ESP32_ADC1_CHANNEL_MAX];
static int s_zero_voltage_mv[ESP32_ADC1_CHANNEL_MAX];


/**
 * @brief 四路ADC通道
 * 
 */
static const adc_channel_t current_adc_channels[CURRENT_ADC_CHANNEL_NUM] = {
    MOTOR0_ADC_CH_IA, /* GPIO36：M0_CS2 */
    MOTOR0_ADC_CH_IB, /* GPIO39：M0_CS1 */
    MOTOR1_ADC_CH_IA, /* GPIO34：M1_CS2 */
    MOTOR2_ADC_CH_IB, /* GPIO35：M1_CS1 */
};

/* ADC中断通知的任务句柄 */
static TaskHandle_t motor_current_adc_task_handle = NULL;

/* ADC校准句柄 */
static adc_cali_handle_t current_adc_cali_handle = NULL;

/* ADC校准功能是否成功初始化 */
static bool current_adc_cali_enable = false;

/* 日志TAG */
static const char *TAG = "CURRENT_ADC";


/**
 * @brief 获取ADC通道对应的信号名称
 * 
 * @param channel 
 * @return const char* 
 */
static const char *get_adc_motor_ch_name(adc_channel_t channel)
{
    switch (channel)
    {
        case MOTOR0_ADC_CH_IA:
            return "M0_Ia";

        case MOTOR0_ADC_CH_IB:
            return "M0_Ib";

        case MOTOR1_ADC_CH_IA:
            return "M1_Ia";

        case MOTOR2_ADC_CH_IB:
            return "M1_Ib";

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
            &voltage_mv
        );

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
#if 1//ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = CURRENT_ADC_UNIT,
        .atten = CURRENT_ADC_ATTEN,
        .bitwidth = CURRENT_ADC_BIT_WIDTH,
    };

    esp_err_t ret = adc_cali_create_scheme_line_fitting(
        &cali_config,
        &current_adc_cali_handle
    );

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC线性拟合校准初始化成功");
        return true;
    }

    ESP_LOGW( TAG,
        "ADC校准初始化失败:%s,后续使用理论电压换算",
        esp_err_to_name(ret)
    );

#else

    ESP_LOGW(TAG, "当前芯片或IDF不支持ADC线性拟合校准");

#endif

    return false;
}



/**
 * @brief ADC转换完成回调, 20KHZ的ADC采样频率
 * 
 * @param adc_handle 
 * @param edata 
 * @param user_data 
 * @return true 
 * @return false 
 */
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
        adc_pattern[i].channel = current_adc_channels[i] & 0x07;
        adc_pattern[i].unit = CURRENT_ADC_UNIT;
        adc_pattern[i].bit_width = CURRENT_ADC_BIT_WIDTH;

        ESP_LOGI(
            TAG,
            "配置%s:ADC1_CH%d",
            get_adc_motor_ch_name(current_adc_channels[i]),
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
 * 判断所有通道是否完成零漂校准
 *----------------------------------------------------------*/

static bool current_adc_zero_calibration_finished(const uint32_t s_zero_count[ESP32_ADC1_CHANNEL_MAX])
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



/**
 * @brief 一阶低通滤波器(IIR Low Pass Filter)
 *
 * 数学公式：
 *      y(n) = y(n-1) + α * (x(n) - y(n-1))
 *
 * 等价于：
 *      y = α*x + (1-α)*y_old
 *
 * 说明：
 *      old    ：上一次滤波后的输出值
 *      input  ：本次新的采样值
 *      alpha  ：滤波系数(0~1)
 *
 * alpha越小：
 *      - 滤波越强
 *      - 输出更平滑
 *      - 响应更慢
 *
 * alpha越大：
 *      - 滤波越弱
 *      - 响应更快
 *      - 更接近原始采样值
 *
 * 特殊情况：
 *      alpha = 1.0f
 *          等于关闭滤波
 *
 *      alpha = 0.0f
 *          输出永远保持旧值
 *
 * 本滤波器推荐用于：
 *      - FOC相电流(Ia、Ib)
 *      - 母线电流
 *      - 母线电压
 *      - 编码器速度
 *
 * 不建议用于：
 *      - ADC Raw原始值
 *
 * @param old
 *      上一次滤波后的输出值
 *
 * @param in
 *      当前新的采样输入值
 *
 * @param alpha
 *      一阶低通滤波系数
 *      推荐：
 *          10kHz采样：0.25
 *          15kHz采样：0.32
 *          20kHz采样：0.40
 *
 * @return
 *      当前滤波后的输出值
 */
static inline float current_lpf(float in, float old)
{
    const float alpha = CURRENT_FILTER_ALPHA;/*0.2~0.4*/

    return old + alpha * (in - old);
}


int64_t get_adc_motor_cuent_time_stamp(void)
{
    return get_curent_time_stamp;
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

    /* 标记四路电流零漂是否校准完成 */
    bool zero_calibration_done = false;

    /* 上一次打印电流数据的时间 */
    // int64_t last_report_time_us = esp_timer_get_time();

    /* 当前任务就是ADC中断需要通知的任务 */
    motor_current_adc_task_handle = xTaskGetCurrentTaskHandle();

    /*
     * 清空静态数据。
     *
     * 因为这些变量位于静态区，不占当前任务栈。
     */
    memset(s_adc_dma_buf_result, 0, sizeof(s_adc_dma_buf_result));
    memset(s_zero_sum, 0, sizeof(s_zero_sum));
    memset(s_zero_count, 0, sizeof(s_zero_count));
    memset(s_zero_raw, 0, sizeof(s_zero_raw));
    memset(s_zero_voltage_mv, 0, sizeof(s_zero_voltage_mv));

    ESP_LOGI(
        TAG,
        "电流采样任务启动，剩余栈：%u字节",
        (unsigned int)uxTaskGetStackHighWaterMark(NULL)
    );

    /*
     * 初始化ADC电压校准。
     */
    current_adc_cali_enable = current_adc_calibration_init();

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
    ESP_ERROR_CHECK( adc_continuous_start(adc_handle) );

    ESP_LOGW(TAG, "开始四路电流零漂校准");
    ESP_LOGW(TAG, "校准期间必须关闭电机PWM,保证相电流为0A");

    float tmp_ia = 0;
    float tmp_lpf_ia = 0;
    float tmp_ib = 0;
    float tmp_lpf_ib = 0;

    int64_t curent_time_stamp_start = 0; /*时间戳,单位:us*/
    int64_t curent_time_stamp_end = 0; /*时间戳,单位:us*/


    while (1)
    {
        /*
         * 等待ADC转换完成回调发送任务通知。
         *
         * 没有ADC数据时，任务会阻塞在这里，
         * 不会一直占用CPU。
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        curent_time_stamp_start = esp_timer_get_time();

        /*
         * 一次通知到来后，尽量把ADC内部缓冲区的数据全部读完。
            零漂值读取
         */
        while (1)
        {
            ret = adc_continuous_read(
                adc_handle,
                s_adc_dma_buf_result,
                sizeof(s_adc_dma_buf_result),
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
                    "读取ADC失败:%s",
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
            for (uint32_t offset = 0;offset < ret_num; offset += SOC_ADC_DIGI_RESULT_BYTES)
            {
                /*
                 * 将DMA数据转换为ESP32 ADC结果结构体。
                 */
                const adc_digi_output_data_t *adc_data = (const adc_digi_output_data_t *)&s_adc_dma_buf_result[offset];
                /*
                 * 从TYPE1格式中提取ADC通道和原始值。
                 */
                uint32_t channel = adc_data->type1.channel;
                uint32_t raw = adc_data->type1.data;/*此ADC通道的原始数据*/

                /*
                 * ESP32 ADC1只有通道0～7。
                 */
                if (channel >= ESP32_ADC1_CHANNEL_MAX)
                {
                    continue;
                }
                /*
                 * 零漂校准阶段。
                 */
                if (!zero_calibration_done)
                {/*未进行零漂值获取*/
                    /*
                     * 每路采集CURRENT_ZERO_SAMPLE_NUM个样本。
                     */
                    if (s_zero_count[channel] < CURRENT_ZERO_SAMPLE_NUM)
                    {
                        s_zero_sum[channel] += raw;
                        s_zero_count[channel]++;
                    }

                }else{/*零漂值获取完成*/
                    
                    /*
                     * 正常运行阶段：每一个ADC样本都先换算成有符号电流，
                     * 再分别统计瞬时值、平均值、最小值、最大值、
                     * 绝对值平均值和RMS有效值。
                     */
                    int voltage_mv = current_adc_raw_to_mv(raw);

                    /*电机相线电压值*/
                    float motor_vin = ((float)voltage_mv - (float)s_zero_voltage_mv[channel] ) / 
                                      INA240_GAIN;
                    /*获取当前通道的电流值*/
                    // float current_a = ((float)voltage_mv - (float)s_zero_voltage_mv[channel] ) /
                    //                   INA240_MV_PER_AMP;
                    /*单位是A, mv/m欧姆 = A*/
                    float current_a = (motor_vin ) / (CURRENT_SHUNT_RESISTOR_OHM*1000);
                    switch (channel)
                    {
                        case MOTOR0_ADC_CH_IA:
                        {
                            adc_m0_val.mtor_ia_curent = current_a;/*单位:A*/
                            adc_m0_val.ia_shunt_mv = motor_vin;
                            break;
                        }
                        case MOTOR0_ADC_CH_IB:
                        {
                            adc_m0_val.mtor_ib_curent = current_a;
                            adc_m0_val.ib_shunt_mv = motor_vin;
                            break;
                        }
                        case MOTOR1_ADC_CH_IA:
                        {
                            adc_m1_val.mtor_ia_curent = current_a;
                            adc_m1_val.ia_shunt_mv = motor_vin;
                            break;
                        }
                        case MOTOR2_ADC_CH_IB:
                        {
                            adc_m1_val.mtor_ib_curent = current_a;
                            adc_m1_val.ib_shunt_mv = motor_vin;
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                }
            }

            /*
             * 检查四路通道是否全部完成零漂采样。
             */
            if ( (!zero_calibration_done ) && current_adc_zero_calibration_finished(s_zero_count))
            {
                for (int i = 0; i < CURRENT_ADC_CHANNEL_NUM; i++)
                {
                    adc_channel_t channel = current_adc_channels[i];
                    /*
                     * 计算该通道零电流时的ADC平均值。
                     */
                    s_zero_raw[channel] = s_zero_sum[channel] / s_zero_count[channel];
                    /*
                     * 将零漂ADC值转换成毫伏。
                     */
                    s_zero_voltage_mv[channel] = current_adc_raw_to_mv(s_zero_raw[channel]);

                    switch (channel)
                    {
                        case MOTOR0_ADC_CH_IA:
                        {
                            adc_m0_val.mtor_zero_ua = s_zero_voltage_mv[channel];
                            break;
                        }
                        case MOTOR0_ADC_CH_IB:
                        {
                            adc_m0_val.mtor_zero_ub = s_zero_voltage_mv[channel];
                            break;
                        }
                        case MOTOR1_ADC_CH_IA:
                        {
                            adc_m1_val.mtor_zero_ua = s_zero_voltage_mv[channel];
                            break;
                        }
                        case MOTOR2_ADC_CH_IB:
                        {
                            adc_m1_val.mtor_zero_ub = s_zero_voltage_mv[channel];
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                    ESP_LOGI(
                        TAG,
                        "%s ,zero_vref:%dmv\r\n",
                        get_adc_motor_ch_name(channel),
                        s_zero_voltage_mv[channel]
                    );
                }
                zero_calibration_done = true;

                // last_report_time_us  = esp_timer_get_time();
                
                ESP_LOGI(TAG, "四路电流零漂电流值获取完成! \r\n");

                esp_err_t ret = motor_power_enable(true);
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "get_current_MOS打开失败,禁止启动电机\r\n");
                }
                else{
                    ESP_LOGE(TAG, "get_current_MOS打开成功,启动电机!\r\n");
                }
            }
        }
        
        /*一阶低通滤波*/
        tmp_lpf_ia = current_lpf(
            adc_m0_val.mtor_ia_curent,
            tmp_lpf_ia
        );
        
        tmp_lpf_ib = current_lpf(
            adc_m0_val.mtor_ib_curent,
            tmp_lpf_ib
        );
        
        set_vfoc_ia_current(tmp_lpf_ia);
        set_vfoc_ib_current(tmp_lpf_ib);
        set_vfoc_ic_current(-tmp_lpf_ia - tmp_lpf_ib);

        curent_time_stamp_end = esp_timer_get_time();

        get_curent_time_stamp = esp_timer_get_time();/*记录下获取电流数据的时间戳*/

        #if 0
            static uint32_t log_cnt = 0;
            if ( (log_cnt++)>100 )
            {
                log_cnt = 0;

                ESP_LOGI(
                    TAG, 
                    "get_motor_curent,ia:%.2f,ib:%.2f,ic:%.2f,t:%lldus,dt:%lld\r\n",
                    tmp_lpf_ia,
                    tmp_lpf_ib,
                    -tmp_lpf_ia - tmp_lpf_ib,
                    get_curent_time_stamp,
                    (curent_time_stamp_end -curent_time_stamp_start)
                );
                
            }
        #endif
    }

    ESP_LOGE(TAG,
            "motor_current_adc_task_error! \r\n"
    );
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
        MOTOR_CURRENT_TASK_STACK,      /* 任务栈大小 */
        NULL,                             /* 任务参数 */
        MOTOR_CURRENT_ADC_TASK_PRIO,        /* 任务优先级 */
        &motor_current_adc_task_handle,         /* 保存任务句柄 */
        MOTOR_CURRENT_TASK_CORE             /* 固定CPU核心 */
    );

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建电流采样任务失败");

        /*
         * 创建失败后停止程序，
         * 防止后续代码在没有电流采样的情况下运行。
         */

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