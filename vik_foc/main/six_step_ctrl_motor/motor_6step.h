/**
 * @file motor_6step.h
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-08-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef SIX_STEP_H
#define SIX_STEP_H


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vik_foc.h"
#include "motor_power.h"
#include "app_rtos_resource.h"
#include "app_rtos_config.h"
#include "motor_current.h"
#include "motor_angle_acqu.h"
#include "motor_cfg_pwm.h"
#include "esp_timer.h"
#include "vik_foc_pid.h"
#include "esp_task_wdt.h"
#include "esp32_flas_nvs.h"


typedef enum
{
    PHASE_OFF=0,
    PHASE_PWM,
    PHASE_LOW

}phase_state_t;


typedef struct
{
    phase_state_t U;
    phase_state_t V;
    phase_state_t W;

}comm_table_t;


extern TaskHandle_t six_step_task_handle;



void six_step_task(void *arg);



#endif