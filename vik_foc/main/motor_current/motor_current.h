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
#define MOTOR0_ADC_CH_IA        ADC_CHANNEL_3 /* GPIO39：M0_CS1 */ 
#define MOTOR0_ADC_CH_IB        ADC_CHANNEL_0 /* GPIO36：M0_CS2 */
#define MOTOR0_ADC_CH_IC        4

#define MOTOR1_ADC_CH_IA        5
#define MOTOR1_ADC_CH_IB        ADC_CHANNEL_6 /* GPIO34：M1_CS2 */
#define MOTOR1_ADC_CH_IC        ADC_CHANNEL_7 /* GPIO35：M1_CS1 */

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


extern adc_motor_current_t adc_m0_val;

extern void motor_get_current_main(void);
int64_t get_adc_motor_cuent_time_stamp(void);


#endif