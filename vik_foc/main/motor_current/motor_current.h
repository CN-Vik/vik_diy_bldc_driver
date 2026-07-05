/**
 * @file motor_current.h
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef MOTOR_CURRENT
#define MOTOR_CURRENT

#include <string.h>
#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


/* 每个电机3路电流采样 ,实际只做了两相*/
/*目前两个电机四个通道*/
#define CURRENT_SAMPLE_PHASE_NUM (3-1)

#define MOTOR_NUM   1

#define CONFIG_IDF_TARGET_ESP32     1

// ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32

#define MOTOR0_ADC_CH_IA ADC_CHANNEL_3 /* GPIO39：M0_CS1 */ 
#define MOTOR0_ADC_CH_IB ADC_CHANNEL_0 /* GPIO36：M0_CS2 */
#define MOTOR0_ADC_CH_IC ADC_CHANNEL_2 /*电机IC相不做电流采样随便写个通道*/

#define MOTOR1_ADC_CH_IA ADC_CHANNEL_6
#define MOTOR1_ADC_CH_IB ADC_CHANNEL_7
#define MOTOR1_ADC_CH_IC ADC_CHANNEL_5
#else
#define MOTOR0_ADC_CH_IA ADC_CHANNEL_2
#define MOTOR0_ADC_CH_IB ADC_CHANNEL_3
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define EXAMPLE_USE_ADC2 0
#endif



#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

/* VREF采样次数 */
#define CURRENT_VREF_SAMPLE_NUM (500)

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

/* 电流采样电阻为10mΩ ,0.01欧姆,此单位是毫欧 */
#define CURRENT_SHUNT_RESISTOR_OHM 10.0f

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

//-------------ADC1 Init---------------//
typedef enum
{
    MOTOR0_ADC_IA_CH = 0,
    MOTOR0_ADC_IB_CH,
    MOTOR0_ADC_IC_CH,

    MOTOR1_ADC_IA_CH = 0,
    MOTOR1_ADC_IB_CH,
    MOTOR1_ADC_IC_CH,

} motor_adc_ch_e_t;

typedef enum
{
    MOTOR0 = 0,
    MOTOR1,
    MOTOR_NUM_MAX,
} motor_num_e_t;



/**
 * @brief ADC数据结构体
 *
 */
typedef struct
{
    /*ADC 校准句柄，后续读取 ADC 原始值时传入，API 自动换算修正后的真实电压*/
    /*motor0_ia && motor0_ib && motor0_ic*/
    adc_cali_handle_t adc1_clib_motor_handle;

    adc_channel_t adc_ch;
    int adc_raw;
    int volt_mv;  /*INA240A2Vout电压值,单位:mv*/
    uint32_t vref_sum;/*INA240的参考电压值,累加值*/
    uint32_t vref_cnt;/*INA240的参考电压值,累加次数*/
    float vref;/*单位mv,INA240的参考电压值,求均值最稳妥*/
    float vin_mv;/*电机相电压*/

    int64_t adc_ch_t_stap; /*每相ADC电流采样时间戳,单位:us*/

} adc_data_t;



typedef struct
{
    adc_data_t chanel[3];/*每个电机三相信号*/
    int64_t time_stamp; /*时间戳,单位:us*/
    float ia_curent;
    float ib_curent;
    float ic_curent;

} adc_motor_curent_t;


// void motor_get_current_main(void);
void motor_get_curent_init(void);
void motor_get_curent(void);



#endif