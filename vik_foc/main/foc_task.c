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
#include "foc_task.h"
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

/*浮点数专用的绝对值宏*/
#define FABS(x) (((x) >= 0.0f ) ? (x) : -(x))


/*!< 静态全局变量：电机M0 FOC控制任务句柄
 *   作用：指向FOC电机控制任务（FreeRTOS任务），用于任务挂起、恢复、删除等管理
 *   初始值NULL表示未创建任务 */
TaskHandle_t foc_task_handle = NULL;

vfoc_time_stamp_t foc_time_stamp={0};
vfoc_time_stamp_t curent_loop_time_stamp={0};

vfoc_pid_t curent_loop_iq_pid = {0};
vfoc_pid_t curent_loop_id_pid = {0};

park_parm_t curent_loop_park = {
    .Uq =0.0f,
    .Ud =0.0f,
    .id=0.0f,
    .iq=0.0f
};



/**
 * @brief 一阶低通滤波器(IIR Low Pass Filter)
 *
 * 数学公式：
 *      y(n) = y(n-1) + α * (x(n) - y(n-1))
 *
 * 等价于：
 *      y = α*x + (1-α)*y_old
 *
 * 说明：
 *      old    ：上一次滤波后的输出值
 *      input  ：本次新的采样值
 *      alpha  ：滤波系数(0~1)
 *
 * alpha越小：
 *      - 滤波越强
 *      - 输出更平滑
 *      - 响应更慢
 *
 * alpha越大：
 *      - 滤波越弱
 *      - 响应更快
 *      - 更接近原始采样值
 *
 * 特殊情况：
 *      alpha = 1.0f
 *          等于关闭滤波
 *
 *      alpha = 0.0f
 *          输出永远保持旧值
 *
 * 本滤波器推荐用于：
 *      - FOC相电流(Ia、Ib)
 *      - 母线电流
 *      - 母线电压
 *      - 编码器速度
 *
 * 不建议用于：
 *      - ADC Raw原始值
 *
 * @param old
 *      上一次滤波后的输出值
 *
 * @param in
 *      当前新的采样输入值
 *
 * @param alpha
 *      一阶低通滤波系数
 *      FOC中断频率推荐：
 *          10kHz采样：0.25
 *          15kHz采样：0.28 ~ 0.35
 *          20kHz采样：0.35 ~ 0.45
 *
 * @return
 *      当前滤波后的输出值
 */
static inline float current_lpf(float in, float old)
{
    /*alpha ↑：响应快、滤波弱、iq 纹波大（和你图里正常运转高频毛刺一致）
    alpha ↓：滤波强、纹波小、动态响应变慢（堵转 / 加减速时 iq 跟踪滞后）*/
    const float alpha = 0.40;

    return old + alpha * (in - old);
}


/**
 * @brief 电流环
 * 
 */
void vfoc_curent_loop(void)
{
    clark_parm_t clark_temp={0};
    park_parm_t park_temp={0};

    static uint32_t log_cnt = 0;

    static uint16_t t_index = 0;

    curent_loop_time_stamp.time[t_index].strat_t = esp_timer_get_time();/*角度值时间戳us*/

    /* clark变换,
    输入三相电流值 ia、ib、ic，计算出 I_alpha、I_beta*/
    clark_temp = clark_tansform(
        get_vfoc_ia_current(),
        get_vfoc_ib_current(),
        get_vfoc_ic_current()
    );

    /* park变换
    * 输入两相静止坐标系电流 I_alpha、I_beta，
    * 结合当前电角度 theta_e_rad，
    * 计算旋转坐标系下的 Id、Iq。
    * */
    park_temp = park_tansform(
        clark_temp.I_alpha,
        clark_temp.I_beta,
        get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())
        // 0.0f
    );

/*---------------------FOC-iq-PI-控制---------------------------*/
    curent_loop_iq_pid.pid_dt = CURRENT_LOOP_DT;
    /*当前电源每V电压支持0.071A， 0.071A/V，
    12V 是母线总电压（VBUS），在 SVPWM 调制下，d/q 轴电压的理论最大幅值只有约 6.93V 
    6.93*0.071A=0.49A 或者直接uq=6.93V,测试堵转电流值*/
    curent_loop_iq_pid.exp_v = 0.20f;//0.30f;/*期望iq值*/
    // 正确滤波Park变换后的Iq反馈电流
    curent_loop_iq_pid.now_v = current_lpf(park_temp.iq, curent_loop_iq_pid.now_v);

    /* 误差值 = 期望值-实际值 */
    curent_loop_iq_pid.err_v = curent_loop_iq_pid.exp_v - curent_loop_iq_pid.now_v;

    /*iq pid 参数,纯PI控制器，kd=0 */
    curent_loop_iq_pid.kp = 50.0f;
    curent_loop_iq_pid.ki = 0.8f;//100.5f;/*  */
    curent_loop_iq_pid.kd = 0.0f;/*  */

    curent_loop_iq_pid.ki_integral_min = -CURENT_I_OUT_LIMIT;
    curent_loop_iq_pid.ki_integral_max = +CURENT_I_OUT_LIMIT;

    /*PID输出结果限幅*/
    curent_loop_iq_pid.pid_out_max = +UQ_LIMIT;
    curent_loop_iq_pid.pid_out_min = -UQ_LIMIT;

    /* FOC电流环_Iq_PI控制 */
    vfoc_pid_calt(&curent_loop_iq_pid);

/*---------------------FOC-iq-PI-控制---------------------------*/


/*---------------------FOC-id-PI-控制---------------------------*/
    curent_loop_id_pid.pid_dt = CURRENT_LOOP_DT;

    curent_loop_id_pid.exp_v = 0.0f;/*期望id值*/
    /*当前实际的Uq值*/
    curent_loop_id_pid.now_v = current_lpf(park_temp.id, curent_loop_id_pid.now_v);

    /* 误差值 = 期望值-实际值 */
    curent_loop_id_pid.err_v = curent_loop_id_pid.exp_v - curent_loop_id_pid.now_v;

    /*id pid 参数,纯PI控制器，kd=0 */
    curent_loop_id_pid.kp = 50.0f;
    curent_loop_id_pid.ki = 0.9f;/*  */
    curent_loop_id_pid.kd = 0.0f;/*  */

    curent_loop_id_pid.ki_integral_min = -CURENT_I_OUT_LIMIT;
    curent_loop_id_pid.ki_integral_max = +CURENT_I_OUT_LIMIT;
    
    curent_loop_id_pid.pid_out_max = +UQ_LIMIT;
    curent_loop_id_pid.pid_out_min = -UQ_LIMIT;

    /* FOC电流环_Id_PI控制 */
    vfoc_pid_calt(&curent_loop_id_pid);
/*---------------------FOC-id-PI-控制---------------------------*/


    #if 1
        // curent_loop_park.Uq = (curent_loop_iq_pid.pid_out*MOTOR0_FORWARD_IQ_DIR);
        curent_loop_park.Uq = curent_loop_iq_pid.pid_out + get_q_cross_couple(&vfoc_m0_dt);
        // curent_loop_park.Uq = 0.0f;
        // curent_loop_park.Ud = curent_loop_id_pid.pid_out + get_d_cross_couple(&vfoc_m0_dt);
        curent_loop_park.Ud = 0.0f;

        curent_loop_park.Uq = limit_float(curent_loop_park.Uq, +UQ_LIMIT, -UQ_LIMIT);
    #else
            curent_loop_park.Uq = UQ_LIMIT;
            // curent_loop_park.Uq = 0.0f;
            // curent_loop_park.Ud = 3.5f;
            curent_loop_park.Ud = 0.0f;
    #endif
    
    /*更新误差值*/
    curent_loop_iq_pid.last_err_v = curent_loop_iq_pid.err_v;
    curent_loop_id_pid.last_err_v = curent_loop_id_pid.err_v;

    curent_loop_time_stamp.time[t_index].end_t = esp_timer_get_time();/*角度值时间戳us*/

    curent_loop_time_stamp.time[t_index].dt = foc_time_stamp.time[t_index].end_t - 
                                        foc_time_stamp.time[t_index].strat_t;/*角度值时间戳us*/

    ++t_index;
    t_index%=10;


    #if 0
        // if ( (t_index==6) && ((log_cnt++)>1000) )
        if ( (log_cnt++)>1000 ) 
        {
            ESP_LOGI(
                TAG,
                // "iq: %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f, %.2f\r\n",
                "iq: %.2f,%.2f,%.2f, %.2f,%.2f\r\n",
                curent_loop_iq_pid.exp_v,//0
                curent_loop_iq_pid.now_v,//1
                // park_temp.Uq,
                curent_loop_park.Uq,//2  

                // get_vfoc_theta_m_deg(),
                // get_vfoc_theta_e_rad(get_vfoc_theta_m_deg()),
                
                // curent_loop_id_pid.exp_v,//3
                // // park_temp.Ud,
                // curent_loop_id_pid.now_v, //4
                // curent_loop_park.Ud,
                get_vfoc_theta_e_w(&vfoc_m0_dt),
                vfoc_m0_dt.motor_drv_val.w_e
            );

            // ESP_LOGI(
            //     TAG,
            //     "iq: %.2f,%.2f,%.2f ,%.2f,%.2f,%.2f, %.2f,%.2f, %.2f,%.2f, %.2f,%.2f,%.2f ,%.2f \r\n",
            //     exp_iq,//0
            //     now_iq,//1  
            //     (exp_iq-now_iq),

            //     exp_id,
            //     now_id,
            //     (exp_id - now_id),

            //     pid_out_uq,
            //     pid_out_ud,

            //     clark_temp.I_alpha,
            //     clark_temp.I_beta,

            //     get_vfoc_ia_current(),
            //     get_vfoc_ib_current(),
            //     get_vfoc_ic_current(),

            //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())
            // );


            /*分析IA IB IC 和Ialpha Ibeta值，iq,id, theta_e电角度值*/
            // ESP_LOGI(
            //     TAG,
            //     "ia:%.2f,ib:%.2f,ic:%.2f, Ialpha:%.2f, Ibeta:%.2f, iq:%.2f, id:%.2f, theta_e:%.2f \r\n",

            //     get_vfoc_ia_current(),
            //     get_vfoc_ib_current(),
            //     get_vfoc_ic_current(),

            //     clark_temp.I_alpha,
            //     clark_temp.I_beta,

            //     now_iq,
            //     now_id,
            //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())

            // );

            // ESP_LOGI(
            //     TAG,
            //     "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2fdeg,e_rad:%.2frad/s,mech_rpm:%.2f\r\n",
            //     get_vfoc_ia_current(),
            //     get_vfoc_ib_current(),
            //     get_vfoc_ic_current(),
            //     get_vfoc_theta_m_deg(),
            //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg()),
            //     get_vfoc_mech_rpm()
            // );
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

            log_cnt = 0;
        }

    #endif
}


/**
 * @brief 获取clark-park-位置/速度/电流环PID运算后的uq,ud值
 * 
 * @return park_parm_t 
 */
park_parm_t vfoc_get_uqd(void)
{
    return curent_loop_park;
}

/**
 * @brief 
 * 
 * @param arg 
 */
static void foc_task(void *arg)
{
    static uint32_t log_cnt = 0;

    int64_t curent_t_stamp = 0;/*电流值时间戳*/
    int64_t angle_t_stamp = 0;/*角度值时间戳*/
    int64_t pwm_isr_t_stamp = 0;/*PWM ISR产生时间戳*/

    uint16_t t_index = 0;

    uint32_t zero_e_cnt = 0;
    float zero_e_mech_sum = 0.0f;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        foc_time_stamp.time[t_index].strat_t = esp_timer_get_time();/*角度值时间戳us*/

        // ESP_LOGI(
        //     TAG,
        //     "foc_task_start!22,%lld\r\n",
        //     (foc_time_stamp.time[t_index].strat_t - get_motor_pwm_isr_time_stamp())
        // );
        
        curent_t_stamp = get_adc_motor_cuent_time_stamp();/*获取电流值的时间戳*/
        angle_t_stamp = get_motor_angle_time_stamp();/*获取角度值数据的时间戳*/
        pwm_isr_t_stamp = get_motor_pwm_isr_time_stamp();

        if ( get_zero_theta_e_calib_flag() )
        {/*已经进行了电角度零点对齐*/

            vfoc_curent_loop();
        
        }else{/*未进行电角度零点对齐*/

            curent_loop_park.Uq = 0.0f;
            curent_loop_park.Ud = 4.5f;

            ++zero_e_cnt;
            zero_e_mech_sum += get_vfoc_theta_m_deg();
            if ( zero_e_cnt >=(20*3000) )/*50us一个周期，20次=1ms,需要100ms*/
            {
                // 显式强清零，确保 SVPWM 此时注入的电压向量在绝对的物理 0 度
                set_vfoc_theta_e_rad(0.0f);
                /*设置零电角度时候的，机械角度偏移值*/
                set_theta_e_offset_mech( zero_e_mech_sum / zero_e_cnt );
                set_zero_theta_e_calib_flag(true);
                zero_e_cnt = 0;

                ESP_LOGI(
                    TAG,
                    "zero_e_mech:%.2f,theta_e(rad/s):%.2f \r\n",
                    get_theta_e_offset_mech(),
                    get_vfoc_theta_e_rad(get_theta_e_offset_mech())
                );

            }
            
        }
        
        #ifdef USE_FOC_SPWM
            /*设置Uq,Ud*/
            vfoc_set_spwm(
                vfoc_get_uqd().Uq,
                vfoc_get_uqd().Ud,
                MOTOR_DRV_VBUS
            );
        #elif defined(USE_FOC_SVPWM)
            /*设置Uq,Ud*/
            vfoc_set_svpwm(
                vfoc_get_uqd().Uq,
                vfoc_get_uqd().Ud,
                MOTOR_DRV_VBUS
            );
        #endif

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
            if ( (log_cnt++)>1000 ) 
            {
                ESP_LOGI(
                    TAG,
                    "iq: %.2f,%.2f,%.2f ,%.2f,%.2f \r\n",
                    exp_iq,//0
                    now_iq,//1  
                    pid_out_uq,//3

                    exp_id,
                    now_id
                );

                // ESP_LOGI(
                //     TAG,
                //     "iq: %.2f,%.2f,%.2f ,%.2f,%.2f,%.2f, %.2f,%.2f, %.2f,%.2f, %.2f,%.2f,%.2f ,%.2f \r\n",
                //     exp_iq,//0
                //     now_iq,//1  
                //     (exp_iq-now_iq),

                //     exp_id,
                //     now_id,
                //     (exp_id - now_id),

                //     pid_out_uq,
                //     pid_out_ud,

                //     clark_temp.I_alpha,
                //     clark_temp.I_beta,

                //     get_vfoc_ia_current(),
                //     get_vfoc_ib_current(),
                //     get_vfoc_ic_current(),

                //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())
                // );


                /*分析IA IB IC 和Ialpha Ibeta值，iq,id, theta_e电角度值*/
                // ESP_LOGI(
                //     TAG,
                //     "ia:%.2f,ib:%.2f,ic:%.2f, Ialpha:%.2f, Ibeta:%.2f, iq:%.2f, id:%.2f, theta_e:%.2f \r\n",
                    
                //     get_vfoc_ia_current(),
                //     get_vfoc_ib_current(),
                //     get_vfoc_ic_current(),

                //     clark_temp.I_alpha,
                //     clark_temp.I_beta,

                //     now_iq,
                //     now_id,
                //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg())

                // );

                // ESP_LOGI(
                //     TAG,
                //     "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2fdeg,e_rad:%.2frad/s,mech_rpm:%.2f\r\n",
                //     get_vfoc_ia_current(),
                //     get_vfoc_ib_current(),
                //     get_vfoc_ic_current(),
                //     get_vfoc_theta_m_deg(),
                //     get_vfoc_theta_e_rad(get_vfoc_theta_m_deg()),
                //     get_vfoc_mech_rpm()
                // );
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
    vfoc_init(&vfoc_m0_dt);

    /*初始化 MOS enable GPIO,默认必须关闭 MOS*/
    motor_power_init();

    /*上电默认设置零电角度未对齐*/
    set_zero_theta_e_calib_flag(false);

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
    

    /*
     * 配置PWM。
     */
    esp32_mcpwm_init();
}

