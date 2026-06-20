/**
 * @file foc_task.c
 * @author vik (ufo281@outlook.com)
 * @brief FOC任务处理
 * 
 *  * 整个工业FOC时序图
 *               PWM Timer

TEZ ------------------------------------ TEZ
│                                         │
│                                         │
│<--------- 一个PWM周期 ----------------->│
│
│  MOS导通
│
│      ↓
│
│  电流稳定
│
│      ↓
│
│  ADC Trigger（硬件）
│
│      ↓
│
│  ADC转换
│
│      ↓
│
│ DMA Done ISR
│      │
│      ▼
│ vTaskNotifyGive()
│      │
│      ▼
│ FOC Task
│      │
│      ├──Ia Ib
│      ├──Encoder
│      ├──Clark
│      ├──Park
│      ├──Id PID
│      ├──Iq PID
│      ├──InvPark
│      ├──SVPWM
│      └──Compare
│
└──────────────等待下一TEZ自动更新PWM──────────────
 * 
 * @version 0.1
 * @date 2026-06-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "motor_power.h"
#include "vik_foc.h"
#include "vik_foc_pid.h"
#include "app_rtos_resource.h"
#include "app_rtos_config.h"
#include "motor_current.h"
#include "motor_angle_acqu.h"
#include "motor_cfg_pwm.h"
#include "esp_timer.h"
#include "vik_foc.h"
#include "vik_foc_pid.h"


const static char *TAG = "FOC_TASK";


/*!< 静态全局变量：电机M0 FOC控制任务句柄
 *   作用：指向FOC电机控制任务（FreeRTOS任务），用于任务挂起、恢复、删除等管理
 *   初始值NULL表示未创建任务 */
TaskHandle_t foc_task_handle = NULL;

vfoc_time_stamp_t foc_time_stamp={0};


/**
 * @brief 
 * 
 * @param arg 
 */
static void foc_task(void *arg)
{
    clark_parm_t clark_temp={0};
    park_parm_t park_temp={0};
    
    float exp_iq = 0.40f;/*期望iq值*/
    float now_iq = 0.0f;/*当前实际的Uq值*/
    float exp_id = 0.0f;/*期望id值*/
    float now_id = 0.0f;
    
    float kp = 0.99f;
    float ki = 0.0f;/*0.001f ~ 0.005f;*/
    float kd = 0.0f;/*0.001f ~ 0.005f;*/
    
    float pid_out_uq = 0.0f;/*pid运算后的uq*/
    float pid_out_ud = 0.0f;
    
    static uint32_t log_cnt = 0;

    int64_t curent_t_stamp = 0;
    int64_t angle_t_stamp = 0;
    int64_t pwm_isr_t_stamp = 0;

    uint16_t t_index = 0;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        foc_time_stamp.time[t_index].strat_t = esp_timer_get_time();/*角度值时间戳us*/
        
        curent_t_stamp = get_adc_motor_cuent_time_stamp();/*获取电流值的时间戳*/
        angle_t_stamp = get_motor_angle_time_stamp();/*获取角度值数据的时间戳*/
        pwm_isr_t_stamp = get_motor_pwm_isr_time_stamp();

        /*2. clark变换,
        输入三相电流值 ia、ib、ic，计算出 I_alpha、I_beta*/
        clark_temp = clark_tansform(
            get_vfoc_ia_current(),
            get_vfoc_ib_current(),
            get_vfoc_ic_current()
        );

        /* 4. park变换
        * 输入两相静止坐标系电流 I_alpha、I_beta，
        * 结合当前电角度 theta_e_rad，
        * 计算旋转坐标系下的 Id、Iq。
        * */
        park_temp = park_tansform(
            clark_temp.I_alpha,
            clark_temp.I_beta,
            get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())
        );

        
        /*当前电源每V电压支持0.071A， 0.071A/V，
        12V 是母线总电压（VBUS），在 SVPWM 调制下，d/q 轴电压的理论最大幅值只有约 6.93V 
        6.93*0.071A=0.49A
        或者直接uq=6.93V,测试堵转电流值*/
        exp_iq = 0.35f;/*期望iq值*/
        now_iq = park_temp.Uq;/*当前实际的Uq值*/

        /*uq pid 参数*/
        kp = 21.2f;
        ki = 0.0f;/*0.001f ~ 0.005f;*/
        kd = 0.0f;/*0.001f ~ 0.005f;*/

        /* FOC电流环_Iq_PI控制 */
        pid_out_uq = vfoc_pid_calt_curent_iq(
            kp,
            ki,
            -CURENT_I_OUT_LIMIT,
            +CURENT_I_OUT_LIMIT,
            kd,
            exp_iq,/*expect:0.8A*/
            now_iq
        );

        exp_id = 0.0f;/*期望id值*/
        now_id = park_temp.Ud;/*当前实际的Uq值*/
        kp = 0.3f;
        ki = 0.01f;
        kd = 0.0f;

        /* FOC电流环_Id_PI控制 */
        pid_out_ud = vfoc_pid_calt_curent_id(
            kp,
            ki,
            -CURENT_I_OUT_LIMIT,
            +CURENT_I_OUT_LIMIT,
            kd,
            exp_id,/*expect:0.8A*/
            now_id
        );

        /*输出uq限幅*/
        pid_out_uq = limit_float(pid_out_uq, -UQ_LIMIT, +UQ_LIMIT);
        pid_out_uq *= MOTOR0_FORWARD_IQ_DIR;
        pid_out_uq = 1.5;
        pid_out_ud = 0;

        /*设置iq,id*/
        vfoc_set_svpwm(
            pid_out_uq,
            pid_out_ud,
            MOTOR_DRV_VBUS
        );

        motor_set_pwm_duty(
            vfoc_get_pwm_duty().duty_Ua,
            vfoc_get_pwm_duty().duty_Ub,
            vfoc_get_pwm_duty().duty_Uc
        );

        foc_time_stamp.time[t_index].end_t = esp_timer_get_time();/*角度值时间戳us*/

        foc_time_stamp.time[t_index].dt = foc_time_stamp.time[t_index].end_t - 
                                          foc_time_stamp.time[t_index].strat_t;/*角度值时间戳us*/
                                          
        #if 0
            // if ( (t_index==6) && ((log_cnt++)>1000) )
            if ( (log_cnt++)>100 ) 
            {

                ESP_LOGI(
                    TAG,
                    "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2fdeg,e_rad:%.2frad/s,mech_rpm:%.2f\r\n",
                    get_vfoc_ia_current(),
                    get_vfoc_ib_current(),
                    get_vfoc_ic_current(),
                    get_vfoc_theta_m_deg(),
                    get_vfoc_theta_e_rad(get_vfoc_theta_m_deg()),
                    get_vfoc_mech_rpm()
                );
                // ESP_LOGI(
                //     TAG,
                //     "foc_task_T:%lldus,foc_time_start:%lld,last_foc_t:%lld ,pwm_isr_t:%lld,curent_t:%lld,angle_t:%lld\r\n",
                //     foc_time_stamp.time[t_index-2].dt,
                //     foc_time_stamp.time[t_index-2].strat_t,
                //     foc_time_stamp.time[t_index-3].strat_t,
                //     pwm_isr_t_stamp,
                //     curent_t_stamp,
                //     angle_t_stamp
                // );

                // ESP_LOGI(
                //     TAG,
                //     "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2f,e_rad:%.2f, duty_ua:%.2f,duty_ub:%.2f,duty_uc:%.2f, \r\n",
                //     get_vfoc_ia_current(),
                //     get_vfoc_ib_current(),
                //     get_vfoc_ic_current(),
                //     get_vfoc_theta_m_deg(),
                //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg()),

                //     vfoc_get_pwm_duty().duty_Ua,
                //     vfoc_get_pwm_duty().duty_Ub,
                //     vfoc_get_pwm_duty().duty_Uc


                // );
                

                t_index = 0;
                log_cnt = 0;

            }
            
        #endif
        
        ++t_index;
        t_index%=10;

    }

}


void foc_task_creat(void)
{
    /*
     *  初始化 FOC 参数。
     * 里面设置 pole_pairs = 7。
     */
    vfoc_init();

    /*初始化 MOS enable GPIO,默认必须关闭 MOS*/
    motor_power_init();

    /*电机编码器初始化获取机械角度,以及初始化零角度的位置*/
    motor_encoder_init();

    /*获取电机电流，初始化*/
    motor_get_current_main();

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
        FOC_TASK_RUN_CORE    /* 绑定CPU核心：指定任务**只运行在 CPU 1** 上 */
    );

    /*
     * 配置PWM。
     */
    esp32_mcpwm_init();
}

