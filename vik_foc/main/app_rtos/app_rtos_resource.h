/**
 * @file app_rtos_resource.h
 * @author vik (ufo281@outlook.com)
 * @brief RTOS的资源 互斥锁，信号量，消息队列，邮箱，事件标志等使用
 * @version 0.1
 * @date 2026-06-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef APP_RTOS_RESOURCE_H
#define APP_RTOS_RESOURCE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/*----------------事件组 bit 定义-----------------------*/
/*
 * 系统事件位定义
 */
#define APP_EVT_CURRENT_ZERO_DONE    (1 << 0)  /* 电流零漂校准完成 */
#define APP_EVT_MOS_ENABLED          (1 << 1)  /* MOS已经使能 */

/*----------------事件组 bit 定义-----------------------*/



// /* ADC零点校准完成事件 */
// #define APP_EVT_ADC_ZERO_DONE      (1 << 0)

// /* 电机启动事件 */
// #define APP_EVT_MOTOR_START        (1 << 1)

// /* 电机停止事件 */
// #define APP_EVT_MOTOR_STOP         (1 << 2)

// /* 系统故障事件 */
// #define APP_EVT_SYSTEM_ERROR       (1 << 3)

/*----------------------------------------------------------
 * 消息队列数据结构
 *----------------------------------------------------------*/

typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_START,
    MOTOR_CMD_STOP,
    MOTOR_CMD_SET_SPEED,
    MOTOR_CMD_SET_POSITION,
} motor_cmd_type_t;

typedef struct
{
    motor_cmd_type_t cmd;
    float value;
} motor_cmd_msg_t;


/**
 * @brief 全局 FreeRTOS 资源声明
 * 
*/

/*----------------信号量---------------------*/
/* ADC数据就绪信号量 */
extern SemaphoreHandle_t g_adc_done_sem;
/*----------------信号量---------------------*/



/*----------------互斥锁---------------------*/
/*
* MOS enable 互斥锁
*
* 作用：
* 防止多个任务同时操作 电机MOS enable GPIO。
*/
extern SemaphoreHandle_t g_mos_enable_mutex;
/* I2C总线互斥锁 */
extern SemaphoreHandle_t g_i2c_mutex;
/* SPI总线互斥锁 */
extern SemaphoreHandle_t g_spi_mutex;
/*----------------互斥锁---------------------*/


/*----------------事件标志组---------------------*/
/*
 * 系统事件组
 *
 * 作用：
 * 用来通知其他任务：
 * 1. 电流零漂是否完成
 * 2. MOS是否已经使能
 */
extern EventGroupHandle_t g_app_event_group;
/*----------------事件标志组---------------------*/



/*----------------邮箱---------------------*/
/* 定义邮箱句柄，本质是QueueHandle_t */
extern QueueHandle_t g_motor0_mech_rpm_mailbox;
extern QueueHandle_t g_motor0_mech_deg_mailbox;

/*----------------邮箱---------------------*/



/*----------------队列---------------------*/
/* 电机命令队列 */
extern QueueHandle_t g_motor_cmd_queue;
/* UART打印队列 */
extern QueueHandle_t g_uart_print_queue;
extern QueueHandle_t g_motor0_mech_rpm_queue;
extern QueueHandle_t g_motor0_mech_deg_queue;

/*----------------队列---------------------*/


void app_rtos_resource_init(void);


#endif