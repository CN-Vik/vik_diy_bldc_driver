/**
 * @file esp32_flas_nvs.h
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ESP32_FLASH_NVS_H
#define ESP32_FLASH_NVS_H

#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"

#define ESP32_FLASH_NVS_NAMESPACE       "blanc_v_car"
#define ESP32_FLASH_NVS_KEY_CFG         "cardev_param"

// 你的配置结构体
typedef struct
{
    uint8_t m0_zero_theta_e_calib_flag;/*M0电机零电角度校准标志*/
    float m0_mech_ofset;/*M0电机零点角度时刻的机械角度偏移值*/
    char dev_name[16];
} DevConfig_t;


extern DevConfig_t balance_vehicle_car;


// 保存结构体到NVS
esp_err_t cfg_saveto_flash(const DevConfig_t *dev);

// 从NVS读取结构体
esp_err_t cfg_readfrom_flash(DevConfig_t *dev);

// 删除保存的结构体（只删自己的参数，WiFi信息保留）
esp_err_t flash_cfg_data_erase(void);


#endif

