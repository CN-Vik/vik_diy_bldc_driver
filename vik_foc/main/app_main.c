/**
 * @file app_main.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "motor_cfg_pwm.h"
#include "app_rtos_config.h"
#include "app_rtos_resource.h"
#include "as5600.h"
#include "bsp_cfg.h"
#include "motor_current.h"
#include "motor_power.h"
#include "vik_foc.h"
#include "vik_foc_pid.h"
#include "motor_angle_acqu.h"
#include "foc_task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "motor_6step.h"




#define USE_FOC
// #define USE_SIX_STEP

static const char *TAG = "vik_foc_example_main";

extern void gptimer_creat_main(void);
extern void wifi_smartcfg_main(void);



void motor_ctrl_task(void)
{
    /*
     *  初始化 FOC 参数。
     * 里面设置 pole_pairs = 7。
     */
    vfoc_init(&vfoc_m0_dt);

    SMO_Init(&vfoc_m0_dt.smo_val);/*滑膜观测器初始化*/
    PLL_Init(&vfoc_m0_dt.pll_val);/*锁相环PLL初始化*/

    /*初始化 MOS enable GPIO,默认必须关闭 MOS*/
    motor_power_init();

    /*上电默认设置零电角度未对齐*/
    set_zero_theta_e_calib_flag(false);

    /*电机编码器初始化获取机械角度,以及初始化零角度的位置*/
    motor_encoder_init();

    /*获取电机电流，初始化*/
    motor_get_current_main();

    /*
     * 配置PWM。
     */
    esp32_mcpwm_init();


    // 读取flash参数
    esp_err_t err = cfg_readfrom_flash(&balance_vehicle_car);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                "read_NVS_failed: %s (0x%x)",
                esp_err_to_name(err),
                err
        );
    }
    else
    {
        ESP_LOGI(TAG,"flash_read_m0_mech_offset_ok = %.3f,flag:%d,theta:%.4f\r\n",
            balance_vehicle_car.m0_e_ofset_rad,
            balance_vehicle_car.m0_zero_theta_e_calib_flag,
            vfoc_calc_theta_e_rad(balance_vehicle_car.m0_e_ofset_rad)
                    
        );
    }

    /*零电角度校准*/

    if ( !balance_vehicle_car.m0_zero_theta_e_calib_flag )
    {/*未进行电角度零点对齐*/
        
        vfoc_calibrate_m0_offset();
        
    }
        

    #ifdef USE_FOC
        /*
        * 创建 FOC 控制任务。
        *    创建一个固定运行在 CPU 核心1上的高优先级电机控制任务
        */
        xTaskCreatePinnedToCore(
            foc_task,        /* 任务函数指针：FOC 电机控制主循环函数（无限循环） */
            "foc_task",              /* 任务名称：调试时方便识别，无实际功能 */
            FOC_TASK_STACK,  /* 任务堆栈大小：分配 4096 字节栈空间（ESP32 单位是字，不是字节） */
            NULL,                       /* 任务入参：不需要传递参数，填 NULL */
            FOC_TASK_PRIO,   /* 任务优先级：20级（最高优先级，保证电机控制实时性） */
            &foc_task_handle,      /* 任务句柄：输出参数，保存创建的任务句柄，用于后续任务管理 */
            FOC_TASK_RUN_CORE    /* 绑定CPU核心：指定任务**只运行在 CPU 0** 上 */
        );
        if ( foc_task_handle==NULL )
        {
            ESP_LOGW(
                TAG,
                " foc_task_creat_failed!\r\n"
            );
            return;
            
        }else{
            ESP_LOGW(
                TAG,
                " foc_task_creat_suceful!\r\n"
            );
            ESP_LOGI(TAG, "[foc_task.c] 地址=%p, 值=%p", &foc_task_handle, foc_task_handle);
        }
        
    #elif defined (USE_SIX_STEP)
        /*
        * 创建 六步换向方波控制 控制任务。
        *    创建一个固定运行在 CPU 核心1上的高优先级电机控制任务
        */
        xTaskCreatePinnedToCore(
            six_step_task,        /* 任务函数指针：6step 电机控制主循环函数（无限循环） */
            "six_step_task",              /* 任务名称：调试时方便识别，无实际功能 */
            SIX_STEP_TASK_STACK,  /* 任务堆栈大小：分配 4096 字节栈空间（ESP32 单位是字，不是字节） */
            NULL,                       /* 任务入参：不需要传递参数，填 NULL */
            SIX_STEP_PRIO,   /* 任务优先级：20级（最高优先级，保证电机控制实时性） */
            &six_step_task_handle,      /* 任务句柄：输出参数，保存创建的任务句柄，用于后续任务管理 */
            SIX_STEP_RUN_CORE    /* 绑定CPU核心：指定任务**只运行在 CPU 0** 上 */
        );
        if ( six_step_task_handle==NULL )
        {
            ESP_LOGW(
                TAG,
                " 6step_task_creat_failed!\r\n"
            );
            return;
            
        }else{
            ESP_LOGW(
                TAG,
                " 6step_task_creat_suceful!\r\n"
            );
        }

    #endif
}




void app_main(void)
{
    ESP_LOGI(TAG, "Hello FOC float version");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 分区异常，先擦除整个nvs分区
        ESP_ERROR_CHECK(nvs_flash_erase());
        // ⭐擦除完成【必须重新初始化】
        err = nvs_flash_init();
        ESP_LOGI(TAG, "NVS_FLASH_Erase_All_data! \r\n");
    }
    ESP_ERROR_CHECK(err);
    wifi_smartcfg_main();

    /*1.rtos系统资源初始化*/
    app_rtos_resource_init();

    motor_ctrl_task();

    gptimer_creat_main();


}
