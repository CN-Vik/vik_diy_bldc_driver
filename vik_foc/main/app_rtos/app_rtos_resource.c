/**
 * @file app_rtos_resource.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-13
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "app_rtos_resource.h"
#include "esp_log.h"

static const char *TAG = "APP_RTOS";



/**
 * @brief 全局 FreeRTOS 资源定义
 * 
 */
/*------信号量--------*/
SemaphoreHandle_t g_adc_done_sem = NULL;
/*------信号量--------*/


/*------互斥锁--------*/
SemaphoreHandle_t g_i2c_mutex = NULL;
SemaphoreHandle_t g_spi_mutex = NULL;
SemaphoreHandle_t g_mos_enable_mutex;

/*------互斥锁--------*/


/*------邮箱-------*/

/* 定义邮箱句柄，本质是QueueHandle_t */
QueueHandle_t g_motor0_mech_rpm_mailbox = NULL;
QueueHandle_t g_motor0_mech_deg_mailbox = NULL;

/*------邮箱-------*/



/*------队列--------*/
QueueHandle_t g_motor_cmd_queue = NULL;
QueueHandle_t g_motor0_mech_rpm_queue = NULL;
QueueHandle_t g_motor0_mech_deg_queue = NULL;
QueueHandle_t g_uart_print_queue = NULL;
/*------队列--------*/

/*---------事件标志组-----------*/
EventGroupHandle_t g_app_event_group = NULL;
/*---------事件标志组-----------*/



/**
 * @brief rtos邮箱创建
 * 
 */
void rtos_email_creat(void)
{
    // 创建邮箱：队列深度=1，每个元素大小 = float
    g_motor0_mech_rpm_mailbox = xQueueCreate( 1, sizeof(float) );
    if(g_motor0_mech_rpm_mailbox == NULL)
    {
        ESP_LOGE(TAG, "创建 g_motor0_mech_rpm_mailbox 失败 \r\n");
    }


    // 创建邮箱：队列深度=1，每个元素大小 = float
    g_motor0_mech_deg_mailbox = xQueueCreate( 1, sizeof(float) );
    if(g_motor0_mech_deg_mailbox == NULL)
    {
        ESP_LOGE(TAG, "创建 g_motor0_mech_rpm_mailbox 失败 \r\n");
    }

}


/**
 * @brief rtos二值信号量创建
 * 
 */
void rtos_binary_semaphore_creat(void)
{
    /*
     * 二值信号量：
     * 适合 ISR 通知任务、任务间简单同步。
    */
    // g_adc_done_sem = xSemaphoreCreateBinary();
    // if (g_adc_done_sem == NULL)
    // {
    //     ESP_LOGE(TAG, "创建 g_adc_done_sem 失败");
        
    // }


}


/**
 * @brief rtos信号量创建
 * 
 */
void rtos_semaphore_creat(void)
{


}


/**
 * @brief rtos互斥锁创建
 * 
 */
void rtos_mutex_creat(void)
{
    // /*
    //  * 互斥锁：
    //  * 适合保护 I2C、SPI、Flash、共享变量等临界资源。
    //  */
    // g_i2c_mutex = xSemaphoreCreateMutex();
    // if (g_i2c_mutex == NULL)
    // {
    //     ESP_LOGE(TAG, "创建 g_i2c_mutex 失败");
        
    // }

    // g_spi_mutex = xSemaphoreCreateMutex();
    // if (g_spi_mutex == NULL)
    // {
    //     ESP_LOGE(TAG, "创建 g_spi_mutex 失败");
        
    // }

    /*
     * 创建 MOS enable 互斥锁
     */
    g_mos_enable_mutex = xSemaphoreCreateMutex();
    if (g_mos_enable_mutex == NULL)
    {
        ESP_LOGE(TAG, "创建 g_mos_enable_mutex 失败");
    }

}


/**
 * @brief rtos队列创建
 * 
 */
void rtos_queue_creat(void)
{
    g_motor0_mech_rpm_queue = xQueueCreate(10, sizeof(float));
    if (g_motor0_mech_rpm_queue == NULL)
    {
        ESP_LOGE(TAG, "创建 g_motor0_mech_rpm_queue 失败");
    }

    // g_motor0_mech_deg_queue = xQueueCreate(10, sizeof(float));
    // if (g_motor0_mech_deg_queue == NULL)
    // {
    //     ESP_LOGE(TAG, "创建 g_motor0_mech_deg_queue 失败");
    // }

    // /*
    //  * UART打印队列：
    //  * 多任务打印时，可以统一丢到打印任务里处理。
    //  */
    // g_uart_print_queue = xQueueCreate(32, sizeof(char *));
    // if (g_uart_print_queue == NULL)
    // {
    //     ESP_LOGE(TAG, "创建 g_uart_print_queue 失败");
        
    // }

}


/**
 * @brief rtos事件标志组创建
 * 
 */
void rtos_event_group_creat(void)
{
    /*
     * 事件组：
     * 适合表示系统状态，比如 ADC校准完成、电机运行、系统错误等。
     */
    g_app_event_group = xEventGroupCreate();
    if (g_app_event_group == NULL)
    {
        ESP_LOGE(TAG, "创建 g_app_event_group 失败");
    }


    /*
     * 上电默认清除事件：
     * 1. 默认认为电流零漂未完成
     * 2. 默认认为 MOS 未使能
     */
    xEventGroupClearBits(
        g_app_event_group,
        APP_EVT_CURRENT_ZERO_DONE | APP_EVT_MOS_ENABLED
    );

}



/**
 * @brief 初始化所有系统级 FreeRTOS 资源
 * 
 */
void app_rtos_resource_init(void)
{
    rtos_email_creat();
    rtos_binary_semaphore_creat();
    rtos_semaphore_creat();
    rtos_mutex_creat();
    rtos_queue_creat();
    rtos_event_group_creat();

    ESP_LOGI(TAG, "FreeRTOS公共资源初始化完成");
}