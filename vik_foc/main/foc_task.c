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
#include "vik_foc_pid.h"
#include "esp_task_wdt.h"
#include "esp32_flas_nvs.h"


const static char *TAG = "FOC_TASK";

/*开启电流环*/
#define VFOC_CURENT_LOOP_EN    0

#define FOC_SENSOR_LESS_EN     1

/*浮点数专用的绝对值宏*/
#define FABS(x) (((x) >= 0.0f ) ? (x) : -(x))


/*!< 静态全局变量：电机M0 FOC控制任务句柄
 *   作用：指向FOC电机控制任务（FreeRTOS任务），用于任务挂起、恢复、删除等管理
 *   初始值NULL表示未创建任务 */
TaskHandle_t foc_task_handle = NULL;

#if (TASK_RUNTIME_STATIS==1)
    vfoc_time_stamp_t foc_time_stamp={0};
    vfoc_time_stamp_t curent_loop_time_stamp={0};
    vfoc_time_stamp_t sped_loop_time_stamp={0};
    vfoc_time_stamp_t sped_postion_time_stamp={0};
#endif

vfoc_pid_t curent_loop_iq_pid = {0};
vfoc_pid_t curent_loop_id_pid = {0};
vfoc_pid_t speed_loop_pid = {0};
vfoc_pid_t postion_loop_pid = {0};




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
 * @brief 位置环PD控制(不基于电流环，输出结果直接作用于Uq)
 * 
 * @param exp_postion_deg 
 * @param now_postion_deg 
 * @return park_parm_t 
 */
park_parm_t vfoc_postion_loop(float exp_postion_deg, float now_postion_deg)
{
    park_parm_t l_postion_lop_park = {0.0f};

    #if (TASK_RUNTIME_STATIS==1)
        ++sped_postion_time_stamp.index;
        sped_postion_time_stamp.index %= TIME_STAMP_SIZE;
        sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % (TIME_STAMP_SIZE)].strat_t = esp_timer_get_time();
        #if 0 /*任务运行频率统计*/
            if ( sped_postion_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_speed_loop_t:%lld,%lld,%lld us\r\n",
                    sped_postion_time_stamp.time[20-1].strat_t,
                    sped_postion_time_stamp.time[20-2].strat_t,
                    sped_postion_time_stamp.time[20-2].strat_t - sped_postion_time_stamp.time[20-1].strat_t

                );

                sped_postion_time_stamp.index = 0;
            }
        #endif
    #endif

    /*设置速度环周期值*/
    postion_loop_pid.pid_dt = POSTION_LOOP_DT;

    /*设置转速期望值*/
    LIMIT_EXP_MECH_360(exp_postion_deg);
    postion_loop_pid.exp_v = (exp_postion_deg);
    
    /*获取当前角度位置*/
    postion_loop_pid.now_v = now_postion_deg;
    // postion_loop_pid.now_v = get_vfoc_theta_m_deg();

    /*角度误差 = 期望值-实际值*/
    postion_loop_pid.err_v = angle_error_deg(postion_loop_pid.exp_v,postion_loop_pid.now_v);

    postion_loop_pid.kp = 0.15f;/**/
    postion_loop_pid.ki = 0.05f;
    postion_loop_pid.kd = 0.0035f;

    postion_loop_pid.ki_out_max = +UQ_LIMIT;
    postion_loop_pid.ki_out_min = -UQ_LIMIT;

    postion_loop_pid.pid_out_max = +UQ_LIMIT;
    postion_loop_pid.pid_out_min = -UQ_LIMIT;

    vfoc_pid_calt(&postion_loop_pid);

    postion_loop_pid.last_err_v = postion_loop_pid.err_v;

    #if 1
        // l_postion_lop_park.Uq = (curent_loop_iq_pid.pid_out*MOTOR0_FORWARD_IQ_DIR);
        // l_postion_lop_park.Uq = curent_loop_iq_pid.pid_out + get_q_cross_couple(&vfoc_m0_dt);
        l_postion_lop_park.Uq = postion_loop_pid.pid_out;
        l_postion_lop_park.Ud = 0.0f;

        // l_postion_lop_park.Ud = curent_loop_id_pid.pid_out + get_d_cross_couple(&vfoc_m0_dt);
    #else
        static float temp_uq = 0.0f;
        temp_uq+=0.001f;
        if ( temp_uq>=6.5f )
        {
            temp_uq=0.0f;
        }
        l_postion_lop_park.Uq = temp_uq;
        // l_postion_lop_park.Uq = UQ_LIMIT;
        l_postion_lop_park.Uq = +3.0f;
        l_postion_lop_park.Ud = 0.0f;
        
    #endif
    l_postion_lop_park.Uq = limit_float(l_postion_lop_park.Uq, -UQ_LIMIT, +UQ_LIMIT);

    #if (TASK_RUNTIME_STATIS==1)
        sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % TIME_STAMP_SIZE].end_t = esp_timer_get_time();
        sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % TIME_STAMP_SIZE].dt = 
            sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % TIME_STAMP_SIZE].end_t -
            sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % TIME_STAMP_SIZE].strat_t;
        #if 0 /*任务运行时长统计*/
            if ( sped_postion_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_speed_loop_DT:%lldus\r\n",
                    sped_postion_time_stamp.time[(sped_postion_time_stamp.index) % TIME_STAMP_SIZE].dt
                );

                sped_postion_time_stamp.index = 0;
            }
        #endif
    #endif

    
    #if 0
        static uint32_t log_cnt = 0;
        // if ( (t_index==6) && ((log_cnt++)>1000) )
        if ( (log_cnt++)>10 ) 
        {
            log_cnt = 0;

            ESP_LOGI(
                TAG,
                "vfoc_postion: %.2f,%.2f,%.2f ,%.4f,%.4f,%.4f,%.4f \r\n",
                postion_loop_pid.exp_v,//0
                postion_loop_pid.now_v,//1
                postion_loop_pid.err_v,//2

                postion_loop_pid.kp_out,//3
                postion_loop_pid.kd_out,//4
                postion_loop_pid.pid_out,
                l_postion_lop_park.Uq
            );

            // set_theta_e_offset_mech(get_theta_e_offset_mech()+10.0f);
            // ESP_LOGI(
                
            //     TAG,
            //     "vfoc_sped: %.2f,%.2f,%.2f,%.2f,%.2f,  %.2f,%.2f\r\n",
            //     get_vfoc_theta_m_deg(),
            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),
            //     // get_theta_e_offset_mech(),
            //     balance_vehicle_car.m0_e_ofset_rad,
            //     m0_mch_rpm,
            //     l_postion_lop_park.Uq,

            //     park_temp.iq,
            //     park_temp.id
                
            // );

        }
    #endif

    return l_postion_lop_park;

}



/**
 * @brief 速度环PI控制(不基于电流环，输出结果直接作用于Uq)
 * 
 * @param exp_sped_rpm 
 * @param now_sped_rpm 
 * @return park_parm_t 
 */
park_parm_t vfoc_speed_loop(float exp_sped_rpm, float now_sped_rpm)
{
    park_parm_t l_sped_lop_park = {0.0f};

    #if (TASK_RUNTIME_STATIS==1)
        ++sped_loop_time_stamp.index;
        sped_loop_time_stamp.index %= TIME_STAMP_SIZE;
        sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % (TIME_STAMP_SIZE)].strat_t = esp_timer_get_time();
        #if 0 /*任务运行频率统计*/
            if ( sped_loop_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_speed_loop_t:%lld,%lld,%lld us\r\n",
                    sped_loop_time_stamp.time[20-1].strat_t,
                    sped_loop_time_stamp.time[20-2].strat_t,
                    sped_loop_time_stamp.time[20-2].strat_t - sped_loop_time_stamp.time[20-1].strat_t

                );

                sped_loop_time_stamp.index = 0;
            }
        #endif
    #endif

    /*设置速度环周期值*/
    speed_loop_pid.pid_dt = SPEED_LOOP_DT;

    /*设置转速期望值*/
    speed_loop_pid.exp_v = exp_sped_rpm;

    /*获取当前转速实际值*/
    speed_loop_pid.now_v = now_sped_rpm;

    /*计算转速误差 = 期望值-实际值*/
    speed_loop_pid.err_v = speed_loop_pid.exp_v - speed_loop_pid.now_v;

    speed_loop_pid.kp = 0.0040f;/*0.0155f*/
    speed_loop_pid.ki = 0.05f;
    // speed_loop_pid.ki = 0.0f;
    speed_loop_pid.kd = 0.0f;

    speed_loop_pid.ki_out_max = +UQ_LIMIT;
    speed_loop_pid.ki_out_min = -UQ_LIMIT;
    // speed_loop_pid.ki_sep_err_thr = 200.0f;
    speed_loop_pid.pid_out_max = +UQ_LIMIT;
    speed_loop_pid.pid_out_min = -UQ_LIMIT;

    vfoc_pid_calt(&speed_loop_pid);

    #if 1
        // l_sped_lop_park.Uq = (curent_loop_iq_pid.pid_out*MOTOR0_FORWARD_IQ_DIR);
        // l_sped_lop_park.Uq = curent_loop_iq_pid.pid_out + get_q_cross_couple(&vfoc_m0_dt);
        l_sped_lop_park.Uq = speed_loop_pid.pid_out;
        l_sped_lop_park.Ud = 0.0f;

        // l_sped_lop_park.Ud = curent_loop_id_pid.pid_out + get_d_cross_couple(&vfoc_m0_dt);
    #else
        static float temp_uq = -6.5f;
        temp_uq+=0.0001f;
        if ( temp_uq>=0.0f )
        {
            temp_uq=-6.5f;
        }
        l_sped_lop_park.Uq = temp_uq;
        // l_sped_lop_park.Uq = UQ_LIMIT;
        l_sped_lop_park.Uq = +3.0f;
        l_sped_lop_park.Ud = 0.0f;
        
    #endif
    l_sped_lop_park.Uq = limit_float(l_sped_lop_park.Uq, -UQ_LIMIT, +UQ_LIMIT);
    // l_sped_lop_park.Uq *= (MOTOR0_UQ_DIR);
    #if (TASK_RUNTIME_STATIS==1)
        sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % TIME_STAMP_SIZE].end_t = esp_timer_get_time();
        sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % TIME_STAMP_SIZE].dt = 
            sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % TIME_STAMP_SIZE].end_t -
            sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % TIME_STAMP_SIZE].strat_t;
        #if 0 /*任务运行时长统计*/
            if ( sped_loop_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_speed_loop_DT:%lldus\r\n",
                    sped_loop_time_stamp.time[(sped_loop_time_stamp.index) % TIME_STAMP_SIZE].dt
                );

                sped_loop_time_stamp.index = 0;
            }
        #endif
    #endif

    
    #if 0
        static uint32_t log_cnt = 0;
        // if ( (t_index==6) && ((log_cnt++)>1000) )
        if ( (log_cnt++)>100 ) 
        {
            log_cnt = 0;
            
            // ESP_LOGI(
            //     TAG,
            //     "vfoc_sped:%.4f,%.2f\r\n",
            //     l_sped_lop_park.Uq,
            //     speed_loop_pid.now_v
                
            // );

            ESP_LOGI(
                TAG,
                "vfoc_sped: %.2f,%.2f,%.2f ,%.4f,%.4f,%.4f,%.4f,%.4f, %.4f,%.4f \r\n",
                speed_loop_pid.exp_v,//0
                speed_loop_pid.now_v,//1
                speed_loop_pid.err_v,
                // speed_loop_pid.kp_out,
                speed_loop_pid.ki,//
                speed_loop_pid.ki_integral,//3
                speed_loop_pid.pid_dt,//4
                speed_loop_pid.ki_out,//5
                speed_loop_pid.pid_out,/*speed_pid_out*/

                l_sped_lop_park.Uq,
                l_sped_lop_park.Ud
                // park_temp.iq
            );

            // set_theta_e_offset_mech(get_theta_e_offset_mech()+10.0f);
            // ESP_LOGI(
                
            //     TAG,
            //     "vfoc_sped: %.2f,%.2f,%.2f,%.2f,%.2f,  %.2f,%.2f\r\n",
            //     get_vfoc_theta_m_deg(),
            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),
            //     // get_theta_e_offset_mech(),
            //     balance_vehicle_car.m0_e_ofset_rad,
            //     m0_mch_rpm,
            //     l_sped_lop_park.Uq,

            //     park_temp.iq,
            //     park_temp.id
                
            // );

        }
    #endif

    return l_sped_lop_park;
}


#if (VFOC_CURENT_LOOP_EN == 1)


/**
 * @brief 电流环(PI控制)
 * 
 * @param exp_iq 
 * @param exp_id 
 * @param now_iq 
 * @param now_id 
 */
park_parm_t vfoc_curent_loop(float exp_iq, float exp_id,float now_iq,float now_id)
{
    park_parm_t l_curent_loop_park = {0.0f};

    #if (TASK_RUNTIME_STATIS==1)
        ++curent_loop_time_stamp.index;
        curent_loop_time_stamp.index %= TIME_STAMP_SIZE;
        curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%(TIME_STAMP_SIZE)].strat_t = esp_timer_get_time();/*角度值时间戳us*/
        
        #if 0
            if ( curent_loop_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_curent_loop_t:%lld,%lld,%lld us\r\n",
                    curent_loop_time_stamp.time[20-1].strat_t,
                    curent_loop_time_stamp.time[20-2].strat_t,
                    curent_loop_time_stamp.time[20-2].strat_t - curent_loop_time_stamp.time[20-1].strat_t

                );
                curent_loop_time_stamp.index = 0;
            }
        #endif
    #endif


/*---------------------FOC-iq-PI-控制---------------------------*/
    curent_loop_iq_pid.pid_dt = CURRENT_LOOP_DT;
    /*当前电源每V电压支持0.071A， 0.071A/V，
    12V 是母线总电压（VBUS），在 SVPWM 调制下，d/q 轴电压的理论最大幅值只有约 6.93V 
    6.93*0.071A=0.49A 或者直接uq=6.93V,测试堵转电流值*/
    curent_loop_iq_pid.exp_v = exp_iq;//0.30f;/*期望iq值*/
    // 正确滤波Park变换后的Iq反馈电流
    curent_loop_iq_pid.now_v = now_iq;/*这个不能滤波，这个iq值是当前最真实的数据反馈*/

    /* 误差值 = 期望值-实际值 */
    curent_loop_iq_pid.err_v = curent_loop_iq_pid.exp_v - curent_loop_iq_pid.now_v;

    /*iq pid 参数,纯PI控制器，kd=0 */
    curent_loop_iq_pid.kp = 13.35f;
    curent_loop_iq_pid.ki = 25918.0f;/*ki = R*2pi*fc，fc:电流环频率500HZ*/
    curent_loop_iq_pid.kd = 0.0f;

    /*PID输出结果限幅*/
    curent_loop_iq_pid.pid_out_max = +UQ_LIMIT;
    curent_loop_iq_pid.pid_out_min = -UQ_LIMIT;

    /* FOC电流环_Iq_PI控制 */
    vfoc_pid_calt(&curent_loop_iq_pid);

/*---------------------FOC-iq-PI-控制---------------------------*/


/*---------------------FOC-id-PI-控制---------------------------*/
    curent_loop_id_pid.pid_dt = CURRENT_LOOP_DT;

    curent_loop_id_pid.exp_v = exp_id;/*期望id值*/
    /*当前实际的Uq值*/
    curent_loop_id_pid.now_v = now_id;/*这个不能滤波，这个id值是当前最真实的数据反馈*/

    /* 误差值 = 期望值-实际值 */
    curent_loop_id_pid.err_v = curent_loop_id_pid.exp_v - curent_loop_id_pid.now_v;

    /*id pid 参数,纯PI控制器，kd=0 */
    curent_loop_id_pid.kp = 13.35f;
    curent_loop_id_pid.ki = 25918.0f;
    curent_loop_id_pid.kd = 0.0f;
    
    curent_loop_id_pid.pid_out_max = +UQ_LIMIT;
    curent_loop_id_pid.pid_out_min = -UQ_LIMIT;

    /* FOC电流环_Id_PI控制 */
    vfoc_pid_calt(&curent_loop_id_pid);
/*---------------------FOC-id-PI-控制---------------------------*/
    #if 0
        // l_curent_loop_park.Uq = (curent_loop_iq_pid.pid_out*MOTOR0_FORWARD_IQ_DIR);
        // l_curent_loop_park.Uq = curent_loop_iq_pid.pid_out + get_q_cross_couple(&vfoc_m0_dt);
        l_curent_loop_park.Uq = curent_loop_iq_pid.pid_out;
        l_curent_loop_park.Ud = 0.0f;

        // l_curent_loop_park.Ud = curent_loop_id_pid.pid_out + get_d_cross_couple(&vfoc_m0_dt);
    #else
        // l_curent_loop_park.Uq = UQ_LIMIT;
        l_curent_loop_park.Uq = +6.0f;
        l_curent_loop_park.Ud = 0.0f;
    #endif
    l_curent_loop_park.Uq = limit_float(l_curent_loop_park.Uq, -UQ_LIMIT, +UQ_LIMIT);
    
    #if (TASK_RUNTIME_STATIS==1)
        /*更新误差值*/
        curent_loop_iq_pid.last_err_v = curent_loop_iq_pid.err_v;
        curent_loop_id_pid.last_err_v = curent_loop_id_pid.err_v;

        curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].end_t = esp_timer_get_time();/*角度值时间戳us*/

        curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].dt = 
            curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].end_t - 
            curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].strat_t;/*角度值时间戳us*/
        #if 0 /*任务运行时长统计*/
            if ( curent_loop_time_stamp.index == 20 )
            {
                ESP_LOGW(
                    TAG,
                    "vfoc_curt_lop_DT: %lld\r\n",
                    // "vfoc_curt_lop_DT:%lldus,%lldus,%lldus\r\n",
                    curent_loop_time_stamp.time[(curent_loop_time_stamp.index) % TIME_STAMP_SIZE].dt
                    // curent_loop_time_stamp.time[(curent_loop_time_stamp.index) % TIME_STAMP_SIZE].dt,
                    // curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].end_t,
                    // curent_loop_time_stamp.time[(curent_loop_time_stamp.index)%TIME_STAMP_SIZE].strat_t

                );
                curent_loop_time_stamp.index = 0;
            }
        #endif
    #endif

    #if 1
        static uint32_t log_cnt = 0;
        // if ( (t_index==6) && ((log_cnt++)>1000) )
        if ( (log_cnt++)>1000 ) 
        {
            // ESP_LOGI(
            //     TAG,
            //     // "vfoc_sped: %.2f,%.2f ,%.2f,%.2f, ,%.2f,%.2f, %.2f \r\n",
            //     "vfoc_curent: %.2f,%.2f ,%.2f,%.2f ,%.2f,%.2f,%.2f,%.2f\r\n",
            //     speed_loop_pid.exp_v,
            //     speed_loop_pid.now_v,

            //     speed_loop_pid.err_v,
            //     speed_loop_pid.kp_out,

            //     speed_loop_pid.pid_out,/*iqref*/
            //     curent_loop_iq_pid.exp_v,/*iqref*/
            //     curent_loop_iq_pid.now_v,/*iq*/
            //     l_curent_loop_park.Uq /*uq*/
            //     // curent_loop_iq_pid.now_v,/*iq*/
            //     // l_curent_loop_park.Uq
            //     // vfoc_get_uqd().Uq
                
            // );

            ESP_LOGI(
                TAG,
                // "iq: %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f, %.2f\r\n",
                // "iq: %.2f,%.2f,%.2f, %.2f, %.2f,%.2f\r\n",
                "iq: %.2f,%.2f,%.2f, %.2f\r\n",
                curent_loop_iq_pid.exp_v,//0
                curent_loop_iq_pid.now_v,//1
                now_iq,
                l_curent_loop_park.Uq//2  

                // (curent_loop_iq_pid.kp_out + curent_loop_iq_pid.ki_out + curent_loop_iq_pid.kd_out)
                // park_temp.Uq,

                // get_vfoc_theta_m_deg(),
                // vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),
                
                // curent_loop_id_pid.exp_v,//3
                // // park_temp.Ud,
                // curent_loop_id_pid.now_v, //4
                // l_curent_loop_park.Ud,
                // get_vfoc_theta_e_w(&vfoc_m0_dt),
                // vfoc_m0_dt.motor_drv_val.w_e
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

            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg())
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
            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg())

            // );

            // ESP_LOGI(
            //     TAG,
            //     "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2fdeg,e_rad:%.2frad/s,mech_rpm:%.2f\r\n",
            //     get_vfoc_ia_current(),
            //     get_vfoc_ib_current(),
            //     get_vfoc_ic_current(),
            //     get_vfoc_theta_m_deg(),
            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),
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
            //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),

            //     vfoc_get_pwm_duty().duty_Ua,
            //     vfoc_get_pwm_duty().duty_Ub,
            //     vfoc_get_pwm_duty().duty_Uc


            // );

            log_cnt = 0;
        }

    #endif

    return l_curent_loop_park;
}

#endif






/**
 * @brief 校准专用：设置指定 Alpha/Beta 电压并更新硬件 PWM
 * 
 * @param u_alpha Alpha 轴电压 (V)
 * @param u_beta  Beta 轴电压 (V)
 */
static void align_set_voltage_ab(float u_alpha, float u_beta)
{
    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_e_t svpwm_status;

    /* 1. 填充电压值到你的 clark 变量结构体中 */
    l_temp_clark_v.I_alpha = u_alpha;
    l_temp_clark_v.I_beta  = u_beta;

    /* 2. 反 Clark 变换 (仅供 Ua/Ub/Uc 调试打印用) */
    vfoc_m0_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

    /* 3. 计算 SVPWM 占空比 */
    svpwm_status = vfoc_svpwm_calc_duty_uab( 
        &l_temp_clark_v,
        MOTOR_DRV_VBUS,
        &vfoc_m0_dt.motor_drv_val.pwm_duty_val
    );

    /* 4. 异常检测 */
    if ((svpwm_status != VFOC_STATUS_OK) && (svpwm_status != VFOC_STATUS_SATURATED))
    {
        ESP_LOGE(TAG, "SVPWM error, status=%d", (int)svpwm_status);
    }


    /*设置PWM占空比*/
    motor_set_pwm_duty(
        vfoc_get_pwm_duty().duty_Ua,
        vfoc_get_pwm_duty().duty_Ub,
        vfoc_get_pwm_duty().duty_Uc
    );

}



/**
 * @brief 校准 M0 电角度 Offset
 * 
 * @return float 校准出的 Offset 弧度 [0, 2π)
 */
float vfoc_calibrate_m0_offset(void)
{   
    uint16_t pole_pairs = vfoc_m0_dt.motor_par.pole_pairs; // 7 极对数
    float mech_theta_deg = 0.0f;

    ESP_LOGI(TAG, "===开始_M0_电角度双向零偏校准===\r\n");

    // =======================================================
    // Step 1: 0° 电角度定位 (Ualpha = +ALIGN_VOLTAGE, Ubeta = 0)
    // =======================================================
    set_vfoc_theta_e_rad(0.0f);// 设置电角度为 0
    align_set_voltage_ab(+0.5f, 0.0f); 
    
    vTaskDelay(pdMS_TO_TICKS(2000));   

    motor_encoder_get_angle(&mech_theta_deg);/*获取最新的机械角度*/

    float theta_e_rad_ofset = (FOC_DEG_TO_RAD(mech_theta_deg)*pole_pairs);

    theta_e_rad_ofset = electricalAngleWrap(theta_e_rad_ofset);/*归一化电角度*/

    align_set_voltage_ab(0.0f, 0.0f);/*安全切断电机电压输出*/

    
    /* 更新到你的平衡车全局结构体 */
    balance_vehicle_car.m0_e_ofset_rad = -theta_e_rad_ofset;
    balance_vehicle_car.m0_zero_theta_e_calib_flag = 1;
    // 保存flash配置
    cfg_saveto_flash(&balance_vehicle_car);

    ESP_LOGI(TAG, "=== 校准成功! M0_e_Ofset: %.4f rad (%.2f°) ===\r\n", 
        balance_vehicle_car.m0_e_ofset_rad, mech_theta_deg
    );

    return -theta_e_rad_ofset;
}


/**
 * @brief 20KHZ
 * 
 * 位置环  << 速度环  <<  电流环
 * 
 * 
位置环：100Hz
    ↓
速度环：500Hz~1kHz
    ↓
电流环：10kHz~30kHz
 * 
 * @param arg 
 */
void foc_task(void *arg)
{
    static uint32_t log_cnt = 0;

    int64_t curent_t_stamp = 0;/*电流值时间戳*/
    int64_t angle_t_stamp = 0;/*角度值时间戳*/
    int64_t pwm_isr_t_stamp = 0;/*PWM ISR产生时间戳*/

    uint32_t zero_e_cnt = 0;
    uint32_t sample_cnt = 0;
    float zero_e_mech_sum = 0.0f;

    float motor_exp_rpm = 0.0;
    float motor_exp_pos_deg = 0.0;
    float iq_ref = 0.0;

    uint16_t freq_cnt = 0;
    uint16_t freq_1KHZ_cnt = 0;
    uint16_t freq_200HZ_cnt = 0;

    clark_parm_t l_temp_clark_v = {0};
    vfoc_status_e_t svpwm_status;

    clark_parm_t clark_temp={0};
    park_parm_t park_temp={0};

    float smo_theta_e = 0.0f;/*无感滑膜计算出来的电角度*/
    float smo_pll_theta_e = 0.0f;/*无感滑膜计算出来的电角度*/
    float sensor_theta_e = 0.0f;/*编码器传感器计算的电角度*/
    float open_lop_theta_e = 0.0f;/*编码器传感器计算的电角度*/

    uint8_t sensor_les_flag = 0;

    park_parm_t foc_lop_out = {0.0f};/*FOC三环输出的uq/ud*/

    float m0_mch_rpm = 0.0f;
    float m0_mch_postion_deg = 0.0f;

    foc_state_t foc_work_state = FOC_STATE_ALIGN;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        #if (TASK_RUNTIME_STATIS==1)

            ++foc_time_stamp.index;
            foc_time_stamp.index %= TIME_STAMP_SIZE;
            foc_time_stamp.time[(foc_time_stamp.index)%(TIME_STAMP_SIZE)].strat_t = esp_timer_get_time();/*角度值时间戳us*/
            #if 0
                if ( foc_time_stamp.index == 20 )
                {
                    ESP_LOGW(
                        TAG,
                        "foc_task_t:%lld,%lld,%lld us\r\n",
                        foc_time_stamp.time[20-1].strat_t,
                        foc_time_stamp.time[20-2].strat_t,
                        foc_time_stamp.time[20-2].strat_t - foc_time_stamp.time[20-1].strat_t

                    );
                    foc_time_stamp.index = 0;
                }
            #endif
            
            curent_t_stamp = get_adc_motor_cuent_time_stamp();/*获取电流值的时间戳*/
            angle_t_stamp = get_motor_angle_time_stamp();/*获取角度值数据的时间戳*/
            pwm_isr_t_stamp = get_motor_pwm_isr_time_stamp();

        #endif

    
        /* clark变换,
        输入三相电流值 ia、ib、ic，计算出 I_alpha、I_beta*/
        clark_temp = clark_tansform(
            get_vfoc_ia_current(),
            get_vfoc_ib_current(),
            get_vfoc_ic_current()
        );

        switch (foc_work_state)
        {
            case FOC_STATE_ALIGN:
            {
                static uint32_t time_cnt_50us = 0;

                // 1. 预定位阶段：给一个固定的角度和一定的 D 轴电流，把转子吸过去
                set_vfoc_theta_e_rad(-FOC_PI/2.0f);
                foc_lop_out.Uq = 1.5f;
                foc_lop_out.Ud = 0.0f;
                
                time_cnt_50us++;
                // 延时一段时间（如 0.5秒）后，进入开环阶段
                if ((time_cnt_50us) > (2000)) // 约500ms（20KHz下）
                {
                    foc_work_state = FOC_STATE_OPEN_LOOP;
                }

                #if 1
                    static uint32_t log_cnt = 0;
                    if ( (log_cnt++)>10 ) 
                    {
                        
                        ESP_LOGI(
                            TAG,
                            "align: %.3f,%.3f, %.3f,%.3f,%.3f \r\n",
                            get_vfoc_theta_e_rad(),
                            sensor_theta_e,
        
                            smo_theta_e,
                            // (sensor_theta_e-smo_theta_e)
                            smo_pll_theta_e,
                            vfoc_m0_dt.pll_val.Ed
        
                        );
        
                        log_cnt = 0;
                    }
                #endif
                
                break;
            }

            case FOC_STATE_OPEN_LOOP:
            {
                // 1. 速度从 0 开始平滑爬坡
                static float current_rpm = 0.0f;

                const float START_RPM  = 0.0f;
                const float TARGET_RPM = 350.0f; 
                const float ACCEL_RATE = 150.0f; // 加速度 (每秒增加 150 RPM)
                
                // 2. 斜坡加速逻辑 (Ramp Up)
                if (current_rpm < TARGET_RPM) 
                {
                    current_rpm += ACCEL_RATE * M0_PWM_TASK_T_S;
                }
                else 
                {
                    current_rpm = TARGET_RPM;
                }

                // 3. RPM 转电角速度 we
                float W_m = current_rpm * FOC_2PI / 60.0f;
                float w_e = W_m * MOTOR_POLR;

                // 4. 累加电角度 (从 -PI/2 连续往上累加，零相位突变！)
                open_lop_theta_e += w_e * M0_PWM_TASK_T_S;
                
                // 5. 归一化到 [0, 2π)
                while (open_lop_theta_e >= FOC_2PI)
                {
                    open_lop_theta_e -= FOC_2PI;
                }
                while (open_lop_theta_e < 0.0f)
                {
                    open_lop_theta_e += FOC_2PI;
                }

                set_vfoc_theta_e_rad(open_lop_theta_e);

                // 6. ★★★ 真正的线性 V/F 曲线 ★★★
                // 起步基底电压 U_min (克服电阻压降)，高速拉到 U_max (对抗反电动势)
                // 如果空载起步转不动，微调 1.0f 到 1.3f；如果发烫严重，调低到 0.8f
                const float U_MIN = 1.0f; 
                const float U_MAX = 2.5f; 
                
                float vf_ratio = current_rpm / TARGET_RPM;
                if (vf_ratio > 1.0f) vf_ratio = 1.0f;

                foc_lop_out.Ud = 0.0f;
                foc_lop_out.Uq = U_MIN + vf_ratio * (U_MAX - U_MIN); // 彻底删除原代码中强行写死 3.0f 的行！

                // 7. 后台运行 SMO 观测器
                smo_theta_e = SMO_Update(
                    &vfoc_m0_dt.smo_val,
                    l_temp_clark_v.I_alpha,
                    l_temp_clark_v.I_beta,
                    clark_temp.I_alpha,
                    clark_temp.I_beta
                );

                w_e = vfoc_calc_we(&vfoc_we_calc_v, smo_theta_e);
                
                // 计算反电动势幅值
                float Emag = sqrtf((vfoc_m0_dt.smo_val.ebmf_alpha * vfoc_m0_dt.smo_val.ebmf_alpha) +
                                   (vfoc_m0_dt.smo_val.ebmf_beta * vfoc_m0_dt.smo_val.ebmf_beta));

                // 8. 切闭环判断条件
                if (w_e > 80.0f && Emag > 0.3f && current_rpm > 120.0f)
                {
                    vfoc_m0_dt.pll_val.theta_e = smo_theta_e;
                    vfoc_m0_dt.pll_val.we = w_e;
                    vfoc_m0_dt.pll_val.ki_integral = w_e;
                    
                    // 需要复位 open loop 内部静态变量，方便下次重启
                    current_rpm = 0.0f; 
                    
                    // foc_work_state = FOC_STATE_CLOSED_LOOP; // 确认开环平稳后放开
                }


                #if 1
                    static uint32_t log_cnt = 0;
                    if ( (log_cnt++)>1000 ) 
                    {
                        
                        ESP_LOGI(
                            TAG,
                            "open_lop: %.3f,%.3f, %.3f,%.3f,%.3f \r\n",
                            get_vfoc_theta_e_rad(),
                            sensor_theta_e,
        
                            smo_theta_e,
                            // (sensor_theta_e-smo_theta_e)
                            smo_pll_theta_e,
                            vfoc_m0_dt.pll_val.Ed
        
                        );
        
                        log_cnt = 0;
                    }
                #endif

                break;
            }
            
            case FOC_STATE_CLOSED_LOOP:
            {
                // 3. 闭环无感阶段：完全由 SMO 和 PLL 接管
                smo_theta_e = SMO_Update(
                    &vfoc_m0_dt.smo_val,
                    l_temp_clark_v.I_alpha,
                    l_temp_clark_v.I_beta,
                    clark_temp.I_alpha,
                    clark_temp.I_beta
                );

                smo_pll_theta_e = PLL_Update(
                    &vfoc_m0_dt.pll_val,
                    vfoc_m0_dt.smo_val.ebmf_alpha,
                    vfoc_m0_dt.smo_val.ebmf_beta
                );

                set_vfoc_theta_e_rad(smo_pll_theta_e);/*更新电角度值*/
                
                /*接收机械角度数据*/
                if ( xQueueReceive(g_motor0_mech_deg_mailbox, &m0_mch_postion_deg, 5)!= pdPASS )
                {
                    ESP_LOGW(
                        TAG,
                        "g_motor0_mech_deg_mailbox recive failed! ,remi:%d,use:%d\r\n",
                        uxQueueSpacesAvailable(g_motor0_mech_deg_mailbox),
                        uxQueueMessagesWaiting(g_motor0_mech_deg_mailbox)
                    );
                }
                
                sensor_theta_e = vfoc_calc_theta_e_rad(m0_mch_postion_deg);/*更新电角度值*/
        
                #if 1
                    static uint32_t log_cnt = 0;
                    if ( (log_cnt++)>10 ) 
                    {
                        
                        ESP_LOGI(
                            TAG,
                            "close_lop: %.3f,%.3f, %.3f,%.3f,%.3f \r\n",
                            get_vfoc_theta_e_rad(),
                            sensor_theta_e,
        
                            smo_theta_e,
                            // (sensor_theta_e-smo_theta_e)
                            smo_pll_theta_e,
                            vfoc_m0_dt.pll_val.Ed
        
                        );
        
                        log_cnt = 0;
                    }
                #endif
        
                #if ( (FOC_SENSOR_LESS_EN == 1) && (1) )
                    /* park变换
                    * 输入两相静止坐标系电流 I_alpha、I_beta，
                    * 结合当前电角度 theta_e_rad，
                    * 计算旋转坐标系下的 Id、Iq。
                    * */
                    park_temp = park_tansform(
                        clark_temp.I_alpha,
                        clark_temp.I_beta,
                        sensor_theta_e
                    );
                #endif
        
                // if ( 0 )
                if ( balance_vehicle_car.m0_zero_theta_e_calib_flag )
                {/*已经进行了电角度零点对齐*/
        
                    static uint32_t run_cnt;
                    if ((run_cnt++)>=(20*100*1))
                    {
                        run_cnt = 0;
                        motor_exp_rpm+=1.0f;
                        if (motor_exp_rpm>=(650.0f))
                        {
                            motor_exp_rpm=1.0f;
                        }
                        
                    }
                    
                    switch ((++freq_1KHZ_cnt))
                    {
                        case 20:{/* 20KHZ/2= 10KHZ*/
        
                            #if (VFOC_CURENT_LOOP_EN == 1)
        
                            #else
                                #if 1
                                    /*接收机械转速*/
                                    if ( xQueueReceive(g_motor0_mech_rpm_queue, &m0_mch_rpm, 5)!= pdPASS )
                                    {
                                        ESP_LOGW(
                                            TAG,
                                            "g_motor0_mech_rpm_queue recive failed! ,remi:%d,use:%d\r\n",
                                            uxQueueSpacesAvailable(g_motor0_mech_rpm_queue),
                                            uxQueueMessagesWaiting(g_motor0_mech_rpm_queue)
                                        );
                                    }
                                    motor_exp_rpm=400.0f;
                                    foc_lop_out = vfoc_speed_loop(motor_exp_rpm,m0_mch_rpm);
                                #endif
                            #endif
                            freq_1KHZ_cnt = 0;
                            break;
                        }
                            
                        default:{
                            break;
                        }
                    }
        
                    switch ((++freq_200HZ_cnt))
                    {
                        case 100:{/* 20KHZ/100= 200HZ*/
        
                            #if 0
                                /*接收机械角度数据*/
                                if ( xQueueReceive(g_motor0_mech_deg_mailbox, &m0_mch_postion_deg, 5)!= pdPASS )
                                {
                                    ESP_LOGW(
                                        TAG,
                                        "g_motor0_mech_deg_mailbox recive failed! ,remi:%d,use:%d\r\n",
                                        uxQueueSpacesAvailable(g_motor0_mech_deg_mailbox),
                                        uxQueueMessagesWaiting(g_motor0_mech_deg_mailbox)
                                    );
                                }
                                motor_exp_pos_deg =150.0f;
                                foc_lop_out = vfoc_postion_loop(motor_exp_pos_deg, m0_mch_postion_deg);
                            #endif
                            freq_200HZ_cnt = 0;
                            break;
                        }
                            
                        default:{
                            break;
                        }
                    }
        
                    #if (VFOC_CURENT_LOOP_EN == 1)/*20KHZ运行*/
                        iq_ref = 0.3f;
                        // foc_lop_out.Uq = 3.0f;
                        // foc_lop_out.Ud = 0.0f;
                        foc_lop_out = vfoc_curent_loop(
                            iq_ref,
                            0.0f,
                            park_temp.iq,
                            park_temp.id
                        );
                    #else
                        void;
                    #endif
                    
                
                }else{/*未进行电角度零点对齐*/
        
                    ESP_LOGW(
                        TAG,
                        "zero_theta_e_calib_failed! (%d)\r\n",
                        balance_vehicle_car.m0_zero_theta_e_calib_flag
                    );
        
                }

                break;
            }

            default:
                break;
        }

        #ifdef USE_FOC_SPWM
            /*设置Uq,Ud*/
            vfoc_set_spwm(
                vfoc_get_uqd().Uq,
                vfoc_get_uqd().Ud,
                MOTOR_DRV_VBUS
            );
        #elif defined(USE_FOC_SVPWM)

            /*
            * 2. 设置 dq 电压。
            *
            * 开环阶段：
            *      Ud = 0
            *      Uq = 给定测试电压
            */
            vfoc_m0_dt.park_val.Uq = foc_lop_out.Uq;
            vfoc_m0_dt.park_val.Ud = foc_lop_out.Ud;

            /*
            * 3. 逆 Park：
            *      输入Ud/Uq + theta_e，输出Ualpha/Ubeta
            */
            l_temp_clark_v = park_inv_transform(&vfoc_m0_dt);

            /*
            * 4. 可选：保存 Ua/Ub/Uc，方便你打印调试。
            *
            * 注意：
            * 真正 SVPWM duty 不依赖这里的 Ua/Ub/Uc。
            * SVPWM 是直接用 alpha/beta 算 duty。
            */
            vfoc_m0_dt.motor_drv_val = clark_inv_transform(&l_temp_clark_v);

            /*
            * 5. SVPWM：
            *      Ualpha/Ubeta -> duty_Ua/duty_Ub/duty_Uc
            */

            #if 0
                svpwm_status = vfoc_svpwm_calc_duty_uab(
                    &l_temp_clark_v,
                    vbus,
                    &vfoc_m0_dt.motor_drv_val.pwm_duty_val
                );
            #else 
            
                #if 1
                    svpwm_status = vfoc_7segment_svpwm_calc(
                        &l_temp_clark_v,
                        MOTOR_DRV_VBUS,
                        &vfoc_m0_dt.motor_drv_val.pwm_duty_val
                    );
                #else        
                    svpwm_status = vfoc_5segment_svpwm_calc(
                        &l_temp_clark_v,
                        vbus,
                        &vfoc_m0_dt.motor_drv_val.pwm_duty_val
                    );
                #endif

            #endif


            #if 0
                static uint32_t log_cnt = 0;
                if ( (log_cnt++)>10 ) 
                {
                    /*
                    * 这里打印的是 SVPWM 注入零序之后的三相电压。
                    * 注意：这个 sum 不一定等于 0。
                    */
                    // ESP_LOGI(TAG,"SVPWM_UVW: %.4f,%.4f,%.4f \r\n",
                    //         vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Ua,
                    //         vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Ub,
                    //         vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Uc
                    // );
                    
                    ESP_LOGI(
                        TAG,
                        "foc_task: %.3f,%.3f \r\n",
                        sensor_theta_e,
                        smo_theta_e
                    );

                    log_cnt = 0;
                }

            #endif
            
            #if 0
                static uint32_t log_cnt = 0; 
                if ( (log_cnt++)>100 ) 
                {
                    ESP_LOGI(
                        TAG,
                        "vfoc_svpwm: %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f \r\n",
                        vfoc_m0_dt.park_val.Uq,
                        l_temp_clark_v.I_alpha,
                        l_temp_clark_v.I_beta,

                        vfoc_m0_dt.motor_drv_val.Ua,
                        vfoc_m0_dt.motor_drv_val.Ub,
                        vfoc_m0_dt.motor_drv_val.Uc,

                        vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Ua,
                        vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Ub,
                        vfoc_m0_dt.motor_drv_val.pwm_duty_val.duty_Uc
                        
                    );
                    
                    log_cnt = 0;
                }
            #endif

            /*
            * 量产代码里，不建议 1ms 打一次日志。
            * 这里只在异常时打。
            */
            if ((svpwm_status != VFOC_STATUS_OK) &&
                (svpwm_status != VFOC_STATUS_SATURATED))
            {
                ESP_LOGE(TAG, "SVPWM error, status=%d", (int)svpwm_status);
            }
        #endif

        
        /*设置PWM占空比*/
        motor_set_pwm_duty(
            vfoc_get_pwm_duty().duty_Ua,
            vfoc_get_pwm_duty().duty_Ub,
            vfoc_get_pwm_duty().duty_Uc
        );
                                          
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

                //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg())
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
                //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg())

                // );

                // ESP_LOGI(
                //     TAG,
                //     "ia:%.2fA,ib:%.2fA,ic:%.2fA,m_deg:%.2fdeg,e_rad:%.2frad/s,mech_rpm:%.2f\r\n",
                //     get_vfoc_ia_current(),
                //     get_vfoc_ib_current(),
                //     get_vfoc_ic_current(),
                //     get_vfoc_theta_m_deg(),
                //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),
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
                //     vfoc_calc_theta_e_rad(get_vfoc_theta_m_deg()),

                //     vfoc_get_pwm_duty().duty_Ua,
                //     vfoc_get_pwm_duty().duty_Ub,
                //     vfoc_get_pwm_duty().duty_Uc


                // );
                

                t_index = 0;
                log_cnt = 0;

            }
            
        #endif

        
        #if (TASK_RUNTIME_STATIS==1)
        
            foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].end_t = esp_timer_get_time();/*角度值时间戳us*/
            foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].dt = 
                foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].end_t - 
                foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].strat_t;/*角度值时间戳us*/

            if ( foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].dt > (M0_PWM_TASK_T_US) )
            {
                ESP_LOGW(
                    TAG,
                    "foc_over_time(%.4f):%lldus,index:%lld\r\n",
                    // "(%.1f)%lldus,%lld\r\n",
                    (float)(M0_PWM_TASK_T_US),
                    foc_time_stamp.time[(foc_time_stamp.index)%TIME_STAMP_SIZE].dt,
                    foc_time_stamp.index
                );
            }
            
            #if 0 /*任务运行时长统计*/
                if ( foc_time_stamp.index == 20 )
                {
                    ESP_LOGW(
                        TAG,
                        "foc_task_DT: %lld\r\n",
                        foc_time_stamp.time[(foc_time_stamp.index) % TIME_STAMP_SIZE].dt
                        // foc_time_stamp.time[(foc_time_stamp.index) % TIME_STAMP_SIZE].dt,
                        // curent_loop_time_stamp.time[(curent_loop_time_stamp.index-2)%TIME_STAMP_SIZE].dt,
                        // sped_loop_time_stamp.time[(sped_loop_time_stamp.index-3)%TIME_STAMP_SIZE].dt

                    );
                    foc_time_stamp.index = 0;
                }
            #endif
        #endif
        
        // vTaskDelay( pdMS_TO_TICKS(1) );
    }

}


