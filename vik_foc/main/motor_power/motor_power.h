/**
 * @file motor_power.h
 * @author vik (ufo281@outlook.com)
 * @brief 电机使能
 * @version 0.1
 * @date 2026-06-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef MOTOR_POWER_H
#define MOTOR_POWER_H

#include <stdbool.h>
#include "esp_err.h"



/*
 * 栅极驱动芯片使能脚。
 * 很多三相驱动板会有一个 EN 引脚，用来总开关 MOSFET 驱动输出。
 * enable = true  时，允许驱动 MOSFET。
 * enable = false 时，关闭驱动输出，电机不再受控输出。
 */
#define VBUS_EN_GPIO         12

/*
 * 这里改成你实际的 MOS enable 引脚
 */
#define MOTOR_MOS_EN_GPIO    VBUS_EN_GPIO

/*
 * 初始化 MOS enable GPIO
 */
void motor_power_init(void);

/*
 * 打开 MOS enable
 *
 * 注意：
 * 这个函数内部会等待电流零漂校准完成。
 */
esp_err_t motor_power_enable(bool en);

/*
 * 获取 MOS 当前是否使能
 */
bool motor_power_is_enabled(void);

#endif