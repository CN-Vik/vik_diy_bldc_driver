/**
 * @file app_rtos_config.h
 * @author vik (ufo281@outlook.com)
 * @brief 进行统一管理RTOS
 * @version 0.1
 * @date 2026-06-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef APP_RTOS_CONFIG_H
#define APP_RTOS_CONFIG_H


/*
 * 任务优先级统一管理
 *
 * 数值越大，优先级越高。
 * ESP-IDF FreeRTOS 中，普通应用任务一般不要乱设太高。
 */

#define APP_TASK_PRIO_LOW          3
#define APP_TASK_PRIO_NORMAL       5
#define APP_TASK_PRIO_HIGH         8
#define APP_TASK_PRIO_REALTIME     10


/*
 * 各个任务的具体优先级
 */
/* ADC读取任务优先级 */
#define MOTOR_CURRENT_ADC_TASK_PRIO      20
#define MO_FOC_CONTROL_TASK_PRIO   8


/*
 * 任务栈大小，ESP-IDF 里单位是 byte，不是 word。
 */
#define MOTOR_CURRENT_TASK_STACK   (8 * 1024)
#define MOTOR_CONTROL_TASK_STACK   (8 * 1024)
#define UART_PRINT_TASK_STACK      (4 * 1024)
#define WIFI_TASK_STACK            (6 * 1024)
#define MO_FOC_CONTROL_TASK_STACK  (8 * 1024) /*8K Byte*/




/*
 * 任务运行核心
 */
#define APP_TASK_CORE_0            0
#define APP_TASK_CORE_1            1

#define MOTOR_CURRENT_TASK_CORE    APP_TASK_CORE_0
#define MOTOR_CONTROL_TASK_CORE    APP_TASK_CORE_1
#define UART_PRINT_TASK_CORE       APP_TASK_CORE_0
#define MO_FOC_CONTROL_TASK_CORE   APP_TASK_CORE_0


#endif
