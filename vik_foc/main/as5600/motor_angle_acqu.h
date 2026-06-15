/**
 * @file motor_angle_acqu.h
 * @author vik (ufo281@outlook.com)
 * @brief 电机的机械角度，电角度，角速度，转速获取
 * @version 0.1
 * @date 2026-06-15
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef MOTOR_ANGLE_ACQU_H
#define MOTOR_ANGLE_ACQU_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"


/*角度抖动误差*/
#define AS5600_ANGLE_DEADBAND_DEG   0.12f


#define AS5600_CHECK_CONF_FIELD_EQ(field)                          \
    do                                                             \
    {                                                              \
        if (conf_write.field != conf_read.field)                   \
        {                                                          \
            ESP_LOGE(TAG, #field " mismatch: write=%d, read=%d",   \
                     (int)conf_write.field, (int)conf_read.field); \
            return ESP_FAIL;                                       \
        }                                                          \
    } while (0)

/* ========================== I2C 硬件引脚配置 ========================== */

/*
 * I2C SCL 时钟线 GPIO。
 *
 * 注意：
 * 这里 GPIO18 / GPIO19 要和你实际硬件连线一致。
 * AS5600 的 SCL 接 ESP32 的 GPIO18。
 */
#define I2C_MASTER_SCL_IO 18

/*
 * I2C SDA 数据线 GPIO。
 *
 * AS5600 的 SDA 接 ESP32 的 GPIO19。
 */
#define I2C_MASTER_SDA_IO 19

/*
 * 使用 ESP32 的 I2C0 控制器。
 *
 * ESP32 一般有 I2C_NUM_0 和 I2C_NUM_1 两组 I2C 控制器。
 */
#define I2C_MASTER_NUM I2C_NUM_0

/*
 * I2C 通信频率。
 *
 * AS5600 支持标准 I2C 通信，这里使用 100kHz，属于比较稳妥的低速配置。
 *
 * 100 * 1000 = 100000 Hz = 100 kHz
 */
#define I2C_MASTER_FREQ_HZ 399 * 1000 /* 399KHZ */

/* ========================== 磁铁状态字符串表 ========================== */

/*
 * AS5600 会检测磁铁状态。
 *
 * 磁铁状态很重要：
 * - 磁铁太远，磁场太弱，角度可能不准
 * - 磁铁太近，磁场太强，角度也可能不准
 * - 没检测到磁铁，则角度数据基本不可用
 *
 * 这里用字符串表把枚举值转换成人能看懂的文本，方便 printf 打印。
 */

/*
 * 计算磁铁状态字符串表的元素个数。
 *
 * sizeof(数组) / sizeof(数组元素) 是 C 语言里常见的数组长度计算方式。
 */
#define AS5600_MAGNET_STATUS_STR_COUNT \
    (sizeof(s_as5600_magnet_status_str) / sizeof(s_as5600_magnet_status_str[0]))

/* ========================== 零点校准参数 ========================== */

/*
 * ZPOS 零点校准后的角度容差。
 *
 * 理论上：
 * 设置当前位置为零点后，再读取角度应该接近 0 度。
 *
 * 但是实际会有：
 * - AS5600 内部滤波延迟
 * - 磁铁安装偏差
 * - I2C 读取时刻误差
 * - 角度值在 0 / 360 度附近跳变
 *
 * 所以不能要求严格等于 0.00 度。
 * 这里允许 ±5 度范围。
 */
#define AS5600_ZERO_CAL_TOL_DEG 5.0f




/**
 * @brief 读取 电机编码器AS5600 当前角度
 *
 * @param angle_deg 输出角度，范围 0~360
 * @return esp_err_t 0 表示读取成功
 */
esp_err_t motor_encoder_get_angle(float *angle_deg);



/**
 * @brief 根据当前机械角度计算电机机械角速度 deg/s
 *
 * @details
 * 这个函数专门给 FOC 位置环的 D 项/阻尼项使用。
 *
 * 作用：
 * 1. 根据 AS5600 当前角度计算机械角速度
 * 2. 自动处理 0° / 360° 跳变
 * 3. 对角速度做一阶低通滤波，减少 AS5600 角度抖动带来的 D 项噪声
 * 4. 静止小抖动时输出 0，避免 D 项残留
 *
 * @param now_angle 当前机械角度，单位 deg，范围一般为 0~360
 * @return float 机械角速度，单位 deg/s
 */
float get_motor_omega_deg_s_by_angle(float now_angle);



/**
 * @brief 根据当前机械角度计算电机机械转速 rpm，并做自适应滤波
 *
 * @details
 * 这个函数用于 FOC 速度闭环。
 *
 * 功能：
 * 1. 根据 AS5600 当前角度计算机械转速 rpm
 * 2. 自动处理 0° / 360° 跳变
 * 3. 静止小抖动时，让速度慢慢衰减到 0
 * 4. 根据低速 / 中速 / 高速自动选择滤波强度
 *
 * 速度范围建议：
 * - 低速：0 ~ 150 rpm，滤波强一点，防止速度环抖动
 * - 中速：150 ~ 800 rpm，滤波适中
 * - 高速：800 rpm 以上，滤波弱一点，保证响应速度
 *
 * @param now_angle 当前机械角度，单位 deg，范围一般是 0 ~ 360
 * @return float 滤波后的机械转速，单位 rpm
 */
float get_motor_rpm_by_angle(float now_angle);


/**
 * @brief 电机编码器AS5600零点位置校准
 *
 * @return uint8_t 0:OK, other:failed!
 */
uint8_t motor_encoder_zero_point_calib(void);


/**
 * @brief 电机编码器AS5600初始化
 *
 * @return uint8_t 0:OK, other failed!
 */
uint8_t as5600_init(void);


/**
 * @brief 电机编码器AS5600初始化
 *
 * @return uint8_t 0:OK, other failed!
 */
void motor_encoder_init(void);


#endif