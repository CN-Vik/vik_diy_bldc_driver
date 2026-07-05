/**
 * @file oneshot_read_main.c
 * @author vik (ufo281@outlook.com)
 * @brief 获取电机三相电流值
 * @version 0.1
 * @date 2026-06-19
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>
#include "vik_foc.h"
#include "app_rtos_config.h"
#include "motor_current.h"
#include "app_rtos_resource.h"
#include "motor_power.h"


const static char *TAG = "MOTOR_ADC_CURENT";

/* ADC中断通知的任务句柄 */
static TaskHandle_t motor_current_adc_task_handle = NULL;

adc_oneshot_unit_handle_t adc1_handle;



/**
 * @brief Motor0 adc curent struct 
 *
 */
adc_motor_curent_t adc_motor_data[MOTOR_NUM_MAX] = {

    [MOTOR0] ={
        .chanel = {
            [MOTOR0_ADC_IA_CH] = {
                .adc_ch = MOTOR0_ADC_CH_IA,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,
                
            },
            [MOTOR0_ADC_IB_CH] = {
                .adc_ch = MOTOR0_ADC_CH_IB,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,
                
            },
            [MOTOR0_ADC_IC_CH] = {/* 电机IC相不做电流采样,随便写个通道 */
                .adc_ch = MOTOR0_ADC_CH_IC,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,
            },
        },
        .time_stamp = 0,
        .ia_curent = 0.0f,
        .ib_curent = 0.0f,
        .ic_curent = 0.0f
    },

    [MOTOR1] ={
        .chanel = {
            [MOTOR1_ADC_IA_CH] = {/* GPIO34：M1_CS2 */
                .adc_ch = MOTOR1_ADC_CH_IA,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,
                
            },
            [MOTOR1_ADC_IB_CH] = {/* GPIO35：M1_CS1 */
                .adc_ch = MOTOR1_ADC_CH_IB,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,
                
            },
            [MOTOR1_ADC_IC_CH] = {/* 电机IC相不做电流采样,随便写个通道 */
                .adc_ch = MOTOR1_ADC_CH_IC,
                .adc_raw = 0,
                .volt_mv = 0,
                .adc_ch_t_stap = 0,

            },
        },
        .time_stamp = 0,
        .ia_curent = 0.0f,
        .ib_curent = 0.0f,
        .ic_curent = 0.0f
    },
};


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
 * @param input
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

        case MOTOR1_ADC_CH_IB:
            return "M1_Ib";

        default:
            return "UNKNOWN";
    }
}

/*---------------------------------------------------------------
ADC Calibration
---------------------------------------------------------------*/
static bool motor_adc_calib_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

/**
 *  直线拟合本质：两点校准，只修正增益 + 零点偏移，不能补偿 ADC 本身的非线性失真
    缺点：低电压段（0~0.5V，电机电流零点附近）误差大，采样电流会轻微抖动；
    优点：出厂 eFuse 自带参数，开箱即用，能把原始读数误差从 ±5% 缩小到 ±3% 以内。
 *
 */
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static void adc_cfg_init(void)
{
    /*创建ADC外设*/
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, /*默认ESP32 ADC 12bit*/
        .atten = ADC_ATTEN_DB_12,         /*ADC衰减 11~12dB,ADC 输入范围扩大。0~3.6V*/
    };

    /*当前单电机,双相电流检测,所以为2*/
    for (uint8_t i = 0; i < CURRENT_SAMPLE_PHASE_NUM; i++)
    {
        for (uint8_t n = 0; n < MOTOR_NUM; n++)
        {
            /*配置通道的衰减系数*/
            ESP_ERROR_CHECK(
                adc_oneshot_config_channel(
                    adc1_handle,
                    adc_motor_data[n].chanel[i].adc_ch,
                    &config
                )
            );

            //-------------ADC1 Calibration Init---------------//
            /* 分别对A相电流通道、B相电流通道创建独立校准句柄，进行ADC校准初始化，保证读取的值准确
            calibrated：标记当前通道是否成功启用校准
            true：校准正常，读 ADC 必须调用 adc_cali_raw_to_voltage() 换算
            false：芯片无 eFuse 校准参数，只能用原始 ADC 值，误差会变大*/
            if (!motor_adc_calib_init(
                    ADC_UNIT_1,
                    adc_motor_data[n].chanel[i].adc_ch, /*ADC通道值*/
                    EXAMPLE_ADC_ATTEN,
                    &adc_motor_data[n].chanel[i].adc1_clib_motor_handle
                ))
            {
                ESP_LOGE(TAG, "Motor_%s通道ADC校准初始化失败,系统无法正常采样电流！\r\n",
                        get_adc_motor_ch_name(adc_motor_data[n].chanel[i].adc_ch));
                abort(); // 直接崩溃重启，等同于ESP_ERROR_CHECK效果
            }
        }
    }

}

static void adc_delete(void)
{
    // 删除ADC1
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));

    /*反初始化ADC通道*/
    for (uint8_t i = 0; i < CURRENT_SAMPLE_PHASE_NUM; i++)
    {
        for (uint8_t n = 0; n < MOTOR_NUM; n++)
        {
            example_adc_calibration_deinit(
                adc_motor_data[n].chanel[i].adc1_clib_motor_handle
            );
        }
    }
}

/**
 * @brief 初始化获取电机INA240相电压的 Vref
 *  
 * INA240A2配置
 Vout = Vref + G*Vin
 Vout:输出的放大信号，ADC检测到的值
 Vref:零点时的电压值，电路设计的是Vref:1.65V
 G:运算放大器的增益值
 Vin:被测电压相线的电压值

求电机相线的电流值就用Vin/R(电阻值)
I = Vin/Rsen
 */
void motor_ina240_vref_init(void)
{
    for (uint32_t k = 0; k < CURRENT_VREF_SAMPLE_NUM; k++)
    {
        /*当前单电机,双相电流检测,所以为2*/
        for (uint8_t i = 0; i < CURRENT_SAMPLE_PHASE_NUM; i++)
        {
            for (uint8_t n = 0; n < MOTOR_NUM; n++)
            {
                /*读取ESP32-ADC原始值*/
                ESP_ERROR_CHECK(
                    adc_oneshot_read(
                        adc1_handle,
                        adc_motor_data[n].chanel[i].adc_ch, /*ADC 通道值*/
                        &adc_motor_data[n].chanel[i].adc_raw
                    )
                );
                adc_motor_data[n].chanel[i].adc_ch_t_stap = esp_timer_get_time();
        
                /*转换成ADC的电压值*/
                ESP_ERROR_CHECK(
                    adc_cali_raw_to_voltage(
                        adc_motor_data[n].chanel[i].adc1_clib_motor_handle, /*ADC handle*/
                        adc_motor_data[n].chanel[i].adc_raw,
                        &adc_motor_data[n].chanel[i].volt_mv
                    )
                );
        
                adc_motor_data[n].chanel[i].vref_sum += adc_motor_data[n].chanel[i].volt_mv;
                adc_motor_data[n].chanel[i].vref_cnt++;
                /*每次累加都求均值*/
                adc_motor_data[n].chanel[i].vref = (((float)adc_motor_data[n].chanel[i].vref_sum) / 
                                                    adc_motor_data[n].chanel[i].vref_cnt);

                if ( k == (CURRENT_VREF_SAMPLE_NUM-1) )
                {/*最后的时候把均值计算出来的vref 打印下*/
                    ESP_LOGI(
                        TAG,
                        "%s:Verf:%.2fmv\r\n",
                        get_adc_motor_ch_name((adc_channel_t)adc_motor_data[n].chanel[i].adc_ch), /*ADC 通道值*/
                        adc_motor_data[n].chanel[i].vref
                    );
                }
                             
            }
            
        }

    }

}
 
void motor_get_curent_init(void)
{
    adc_cfg_init();
    motor_ina240_vref_init();

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


void motor_get_curent(void)
{
    // adc_time_stamp_start = esp_timer_get_time();/*角度值时间戳us*/
    /*当前单电机,双相电流检测,所以为2*/
    for (uint8_t i = 0; i < CURRENT_SAMPLE_PHASE_NUM; i++)
    {
        for (uint8_t n = 0; n < MOTOR_NUM; n++)
        {
            /*读取ESP32-ADC原始值*/
            ESP_ERROR_CHECK(
                adc_oneshot_read(
                    adc1_handle,
                    adc_motor_data[n].chanel[i].adc_ch, /*ADC 通道值*/
                    &adc_motor_data[n].chanel[i].adc_raw
                )
            );
            adc_motor_data[n].chanel[i].adc_ch_t_stap = esp_timer_get_time();

            /*转换成ADC的电压值*/
            ESP_ERROR_CHECK(
                adc_cali_raw_to_voltage(
                    adc_motor_data[n].chanel[i].adc1_clib_motor_handle, /*ADC handle*/
                    adc_motor_data[n].chanel[i].adc_raw,
                    &adc_motor_data[n].chanel[i].volt_mv
                )
            );

            /*电机相线电压值*/
            adc_motor_data[n].chanel[i].vin_mv = ((float)adc_motor_data[n].chanel[i].volt_mv - 
                                            (float)adc_motor_data[n].chanel[i].vref ) / 
                                            INA240_GAIN;
            /*获取当前通道的电流值*/
            /*单位是A, mv/毫欧姆 = A*/
            float curent_now = (adc_motor_data[n].chanel[i].vin_mv ) / 
                                    (CURRENT_SHUNT_RESISTOR_OHM);


            switch (adc_motor_data[n].chanel[i].adc_ch)
            {
                case MOTOR0_ADC_CH_IA:
                {
                    curent_now = current_lpf(
                        curent_now,
                        adc_motor_data[n].ia_curent
                    );
                    set_vfoc_ia_current(curent_now);
                    adc_motor_data[n].ia_curent = curent_now;/*单位:A*/
                    break;
                }
                case MOTOR0_ADC_CH_IB:
                {
                    curent_now = current_lpf(
                        curent_now,
                        adc_motor_data[n].ib_curent
                    );
                    set_vfoc_ib_current(curent_now);
                    
                    adc_motor_data[n].ib_curent = curent_now;/*单位:A*/
                    /*采集完IB相以后立刻计算IC电流*/
                    adc_motor_data[n].ic_curent = -adc_motor_data[n].ia_curent
                                                    -adc_motor_data[n].ib_curent;/*单位:A*/
                    set_vfoc_ic_current(adc_motor_data[n].ic_curent);

                    adc_motor_data[n].time_stamp = esp_timer_get_time();/*三相电流采集完成的时间戳us*/
                    break;
                }
                case MOTOR1_ADC_CH_IA:
                {
                    adc_motor_data[n].ia_curent = curent_now;/*单位:A*/
                    break;
                }
                case MOTOR1_ADC_CH_IB:
                {
                    adc_motor_data[n].ib_curent = curent_now;/*单位:A*/
                    /*采集完IB相以后立刻计算IC电流*/
                    adc_motor_data[n].ic_curent = -adc_motor_data[n].ia_curent
                                                    -adc_motor_data[n].ib_curent;/*单位:A*/
                    adc_motor_data[n].time_stamp = esp_timer_get_time();/*三相电流采集完成的时间戳us*/
                    break;
                }

                default:
                {
                    break;
                }
            }

            // ESP_LOGI(
            //     TAG,
            //     "i:%d,%s:vref:%.2fmv,vin:%.2fmv,adc_V:%dmv,ia:%.2fA,ib:%.2fA,ic:%.2fA,adc_read_T:%lldus\r\n",
            //     i,
            //     get_adc_motor_ch_name(adc_motor_data[n].chanel[i].adc_ch), /*ADC 通道值*/
            //     adc_motor_data[n].chanel[i].vref,
            //     adc_motor_data[n].chanel[i].vin_mv,
            //     adc_motor_data[n].chanel[i].volt_mv,
            //     adc_motor_data[n].ia_curent,
            //     adc_motor_data[n].ib_curent,
            //     adc_motor_data[n].ic_curent,
            //     adc_read_T
            // );
        }
    }
    // adc_time_stamp_end = esp_timer_get_time();/*角度值时间戳us*/
    // adc_read_T = adc_time_stamp_end - adc_time_stamp_start;
    // ESP_LOGI(
    //     TAG,
    //     "6adc_read_T:%lldus\r\n",
    //     adc_read_T
    // );

}


#if 0

/**
 * @brief INA240电流采样任务
 *
 * @param arg 任务参数，当前没有使用
 */
static void motor_current_adc_task(void *arg)
{
    int64_t adc_time_stamp_start; /*时间戳,单位:us*/
    int64_t adc_time_stamp_end; /*时间戳,单位:us*/
    int64_t adc_read_T = 0; /*adc读取电流时间,单位:us*/

    adc_cfg_init();
    motor_ina240_vref_init();

    ESP_LOGI(TAG, "四路电流零漂电流值获取完成! \r\n");

    esp_err_t ret = motor_power_enable(true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "get_current_MOS打开失败,禁止启动电机\r\n");
    }
    else{
        ESP_LOGE(TAG, "get_current_MOS打开成功,启动电机!\r\n");
    }

    while (1)
    {
        adc_time_stamp_start = esp_timer_get_time();/*角度值时间戳us*/

        /*当前单电机,双相电流检测,所以为2*/
        for (uint8_t i = 0; i < CURRENT_SAMPLE_PHASE_NUM; i++)
        {
            for (uint8_t n = 0; n < MOTOR_NUM; n++)
            {
                /*读取ESP32-ADC原始值*/
                ESP_ERROR_CHECK(
                    adc_oneshot_read(
                        adc1_handle,
                        adc_motor_data[n].chanel[i].adc_ch, /*ADC 通道值*/
                        &adc_motor_data[n].chanel[i].adc_raw
                    )
                );
                adc_motor_data[n].chanel[i].adc_ch_t_stap = esp_timer_get_time();

                /*转换成ADC的电压值*/
                ESP_ERROR_CHECK(
                    adc_cali_raw_to_voltage(
                        adc_motor_data[n].chanel[i].adc1_clib_motor_handle, /*ADC handle*/
                        adc_motor_data[n].chanel[i].adc_raw,
                        &adc_motor_data[n].chanel[i].volt_mv
                    )
                );

                /*电机相线电压值*/
                adc_motor_data[n].chanel[i].vin_mv = ((float)adc_motor_data[n].chanel[i].volt_mv - 
                                                (float)adc_motor_data[n].chanel[i].vref ) / 
                                                INA240_GAIN;
                /*获取当前通道的电流值*/
                /*单位是A, mv/毫欧姆 = A*/
                float curent_now = (adc_motor_data[n].chanel[i].vin_mv ) / 
                                        (CURRENT_SHUNT_RESISTOR_OHM);


                switch (adc_motor_data[n].chanel[i].adc_ch)
                {
                    case MOTOR0_ADC_CH_IA:
                    {
                        curent_now = current_lpf(
                            curent_now,
                            adc_motor_data[n].ia_curent
                        );
                        set_vfoc_ia_current(curent_now);
                        adc_motor_data[n].ia_curent = curent_now;/*单位:A*/
                        break;
                    }
                    case MOTOR0_ADC_CH_IB:
                    {
                        curent_now = current_lpf(
                            curent_now,
                            adc_motor_data[n].ib_curent
                        );
                        set_vfoc_ib_current(curent_now);
                        
                        adc_motor_data[n].ib_curent = curent_now;/*单位:A*/
                        /*采集完IB相以后立刻计算IC电流*/
                        adc_motor_data[n].ic_curent = -adc_motor_data[n].ia_curent
                                                      -adc_motor_data[n].ib_curent;/*单位:A*/
                        set_vfoc_ic_current(adc_motor_data[n].ic_curent);

                        adc_motor_data[n].time_stamp = esp_timer_get_time();/*三相电流采集完成的时间戳us*/
                        break;
                    }
                    case MOTOR1_ADC_CH_IA:
                    {
                        adc_motor_data[n].ia_curent = curent_now;/*单位:A*/
                        break;
                    }
                    case MOTOR1_ADC_CH_IB:
                    {
                        adc_motor_data[n].ib_curent = curent_now;/*单位:A*/
                        /*采集完IB相以后立刻计算IC电流*/
                        adc_motor_data[n].ic_curent = -adc_motor_data[n].ia_curent
                                                      -adc_motor_data[n].ib_curent;/*单位:A*/
                        adc_motor_data[n].time_stamp = esp_timer_get_time();/*三相电流采集完成的时间戳us*/
                        break;
                    }

                    default:
                    {
                        break;
                    }
                }

                // ESP_LOGI(
                //     TAG,
                //     "i:%d,%s:vref:%.2fmv,vin:%.2fmv,adc_V:%dmv,ia:%.2fA,ib:%.2fA,ic:%.2fA,adc_read_T:%lldus\r\n",
                //     i,
                //     get_adc_motor_ch_name(adc_motor_data[n].chanel[i].adc_ch), /*ADC 通道值*/
                //     adc_motor_data[n].chanel[i].vref,
                //     adc_motor_data[n].chanel[i].vin_mv,
                //     adc_motor_data[n].chanel[i].volt_mv,
                //     adc_motor_data[n].ia_curent,
                //     adc_motor_data[n].ib_curent,
                //     adc_motor_data[n].ic_curent,
                //     adc_read_T
                // );
            }
        }
        adc_time_stamp_end = esp_timer_get_time();/*角度值时间戳us*/
        adc_read_T = adc_time_stamp_end - adc_time_stamp_start;
        // ESP_LOGI(
        //     TAG,
        //     "6adc_read_T:%lldus\r\n",
        //     adc_read_T
        // );

        vTaskDelay(pdMS_TO_TICKS(1));
    }

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

#endif