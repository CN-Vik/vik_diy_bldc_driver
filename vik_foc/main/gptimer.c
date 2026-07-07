/**
 * @file gptimer.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-31
 * 
 * @copyright Copyright (c) 2026
 * 
 */
/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * 说明：
 * 这是 ESP-IDF 官方 GPTimer（通用硬件定时器）的示例代码。
 *
 * 这个例子主要演示 3 种 GPTimer 使用方式：
 *
 * 1. 定时器到达 alarm_count 后，在中断里停止定时器
 *    对应回调函数：example_timer_on_alarm_cb_v1()
 *
 * 2. 定时器到达 alarm_count 后，自动重装 reload_count，形成周期性定时
 *    对应回调函数：example_timer_on_alarm_cb_v2()
 *
 * 3. 定时器到达 alarm_count 后，在中断里动态修改下一次 alarm_count
 *    对应回调函数：example_timer_on_alarm_cb_v3()
 *
 * 对新手来说，可以把 GPTimer 理解成：
 * MCU 内部有一个硬件计数器，它按照固定频率一直数数。
 * 当计数值等于你设置的 alarm_count 时，就触发一次中断。
 */

#include <stdio.h>

/* FreeRTOS 基础头文件
 * ESP-IDF 默认运行在 FreeRTOS 上。
 */
#include "freertos/FreeRTOS.h"

/* FreeRTOS 任务相关接口
 * 本例中用到了 vTaskDelay 相关宏的基础定义，虽然没有直接创建任务。
 */
#include "freertos/task.h"

/* FreeRTOS 队列
 * 本例中 ISR 中断回调函数不会直接 printf，
 * 而是把事件通过队列发送给 app_main 所在任务处理。
 */
#include "freertos/queue.h"

/* ESP-IDF GPTimer 驱动头文件
 * GPTimer 是 ESP32 系列中的通用硬件定时器驱动。
 */
#include "driver/gptimer.h"

/* ESP-IDF 日志头文件
 * ESP_LOGI / ESP_LOGW / ESP_LOGE 等日志宏都在这里。
 */
#include "esp_log.h"
#include <inttypes.h>

#include "esp_timer.h"

// int64_t time_us = esp_timer_get_time();   // us 级，单位：微秒
// int64_t time_ms = time_us / 1000;         // ms 级
// int64_t time_s  = time_us / 1000000;      // s 级

#define GPTIMER_US(us)          (us)
#define GPTIMER_MS(ms)      (ms * GPTIMER_US(1000) )
#define GPTIMER_S(s)        (s * GPTIMER_MS(1000)  )


/* 日志 TAG
 * 打印日志时会显示这个 TAG，方便区分是哪个模块输出的日志。
 * 例如：I (1234) example: Create timer handle
 */
static const char *TAG = "gptimer";


/* 队列元素结构体
 *
 * 中断里触发定时器 alarm 后，会把当前计数值发送到队列。
 * app_main 再从队列里接收这个结构体。
 */
typedef struct {
    /* 定时器触发 alarm 时的计数值
     *
     * 类型是 uint64_t，因为硬件定时器计数值可能很大。
     */
    uint64_t event_count;
} example_queue_element_t;


extern TaskHandle_t motor_get_angle_task_handle;



/**
 * @brief 定时器 alarm 回调函数版本 2
 *
 * 特点：
 * 这里只把 alarm 事件发送到队列，不主动停止定时器。
 *
 * 配合后面的：
 * .flags.auto_reload_on_alarm = true
 *
 * 可以实现周期性定时。
 *
 * 应用场景：
 * 每 1ms / 10ms / 1s 周期执行一次任务。
 *
 * 注意：
 * 周期性定时最好不要在 ISR 里做复杂工作，
 * ISR 里只发通知，真正的业务逻辑放到任务里执行。
 */
static bool IRAM_ATTR gptimer_1ms_cb( gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_data)
{
    static int32_t cnt = 0;
#if 0
    /* 当前没有直接使用 timer 参数
     * 有些编译器可能会提示未使用参数，不过 ESP-IDF 示例里这样写没问题。
     */
    BaseType_t high_task_awoken = pdFALSE;

    /* 获取队列句柄 */
    QueueHandle_t gptimer_queue = (QueueHandle_t)user_data;

    /* 保存本次 alarm 触发时的计数值 */
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };

    /* ISR 中发送队列 */
    xQueueSendFromISR(gptimer_queue, &ele, &high_task_awoken);

    /* 返回是否需要任务切换 */
    return (high_task_awoken == pdTRUE);
#endif

    BaseType_t hp = pdFALSE;

    vTaskNotifyGiveFromISR(
        motor_get_angle_task_handle,
        &hp
    );

    return hp == pdTRUE;

}



/**
 * @brief ESP-IDF 应用入口函数
 *
 * ESP-IDF 中没有传统裸机 main()。
 * 用户应用从 app_main() 开始执行。
 */
void gptimer_creat_main(void)
{
    /* 用于从队列中接收定时器事件 */
    // example_queue_element_t ele;

    /* 创建一个 FreeRTOS 队列
     *
     * 参数 1：队列长度，最多可以缓存 10 个元素
     * 参数 2：每个元素大小，这里是 example_queue_element_t
     *
     * 为什么用队列？
     * 因为定时器 alarm 回调在 ISR 中运行。
     * ISR 中不建议直接做复杂逻辑或 printf。
     * 正确做法是 ISR 发事件，任务里处理事件。
     */
    // QueueHandle_t gptimer_queue = xQueueCreate(10, sizeof(example_queue_element_t));

    /* 判断队列是否创建成功 */
    // if (!gptimer_queue) {
    //     ESP_LOGE(TAG, "Creating gptimer_queue failed");
    //     return;
    // }

    ESP_LOGI(TAG, "Create gptimer handle");

    /* GPTimer 句柄
     *
     * 句柄可以理解为“定时器对象”的指针。
     * 后面对这个定时器的所有操作，都通过 gptimer 这个句柄完成。
     */
    gptimer_handle_t gptimer = NULL;

    /* GPTimer 基础配置 */
    gptimer_config_t timer_config = {
        /* 时钟源
         *
         * GPTIMER_CLK_SRC_DEFAULT：
         * 使用 ESP-IDF 默认推荐的 GPTimer 时钟源。
         */
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,

        /* 计数方向
         *
         * GPTIMER_COUNT_UP：
         * 向上计数，也就是 0、1、2、3... 这样递增。
         */
        .direction = GPTIMER_COUNT_UP,

        /* 定时器频率
         *
         * 1*1000*1000 Hz = 1 MHz
         *
         * 意味着定时器每秒计数 1000000 次。
         * 所以：
         * 1 tick = 1 / 1000000 秒 = 1 us
         *
         * 这也是后面 alarm_count = 1000000 表示 1 秒的原因。
         */
        .resolution_hz = 1*1000*1000, // 1MHz，1 tick = 1us
    };

    /* 创建一个新的 GPTimer
     *
     * 参数 1：定时器配置
     * 参数 2：返回创建好的定时器句柄
     *
     * ESP_ERROR_CHECK：
     * 如果函数返回错误，会打印错误并终止程序。
     */
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    /* 配置 GPTimer 事件回调函数
     *
     * on_alarm：
     * 当定时器计数到 alarm_count 时，会调用这个函数。
     *
     * 这里第一次使用 v1 回调：
     * alarm 后立即停止定时器。
     */
    gptimer_event_callbacks_t gptimer_cb = {
        .on_alarm = gptimer_1ms_cb,
    };

    /* 注册 GPTimer 回调函数
     *
     * 参数 1：定时器句柄
     * 参数 2：回调函数结构体
     * 参数 3：用户自定义数据 user_data
     *
     * 这里把 gptimer_queue 传进去，
     * 后面中断回调函数里就可以通过 user_data 拿到 queue。
     */
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &gptimer_cb, NULL));
    // ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &gptimer_cb, gptimer_queue));

    ESP_LOGI(TAG, "Enable timer");

    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG, "Start timer, auto-reload at alarm event");

    /* alarm 配置 2：自动重装模式
     *
     * reload_count = 0：
     * 每次 alarm 后，计数器自动回到 0。
     *
     * alarm_count = 1000000：
     * 从 0 数到 1000000，刚好 1 秒。
     *
     * auto_reload_on_alarm = true：
     * 到达 alarm 后自动 reload，然后继续计数。
     *
     * 结果：
     * 每 1 秒触发一次 alarm。
     */
    gptimer_alarm_config_t alarm_config2 = {
        .reload_count = 0,
        .alarm_count = GPTIMER_MS(1), // 周期 = 100*(1000us) = 100ms
        .flags.auto_reload_on_alarm = true,
    };

    /* 设置自动重装 alarm */
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config2));

    /* 启动定时器 */
    ESP_ERROR_CHECK(gptimer_start(gptimer));

#if 0
    /* 记录 4 次 alarm 事件 */
    int record = 4;
    int64_t time_us;

    /* 循环等待 4 次定时器触发 */
    while (record) {
    // while (1) {
        if (xQueueReceive(gptimer_queue, &ele, pdMS_TO_TICKS(2000))) {

            time_us = esp_timer_get_time();
            ESP_LOGI(TAG, "gpTimer_reloaded, count[%llu],time_stamp:%llu",
                ele.event_count,
                time_us
            );
            /* 收到一次事件，次数减 1 */
            record--;
        } else {
            ESP_LOGW(TAG, "Missed one count event");
        }
    }

    ESP_LOGI(TAG, "Stop gptimer");

    /* 停止定时器计数 */
    ESP_ERROR_CHECK(gptimer_stop(gptimer));

    ESP_LOGI(TAG, "Disable gptimer");

    /* disable 后才方便再次修改回调 */
    ESP_ERROR_CHECK(gptimer_disable(gptimer));

    ESP_LOGI(TAG, "Delete gptimer");

    /* 删除定时器，释放驱动资源 */
    ESP_ERROR_CHECK(gptimer_del_timer(gptimer));

    /* 删除 FreeRTOS 队列，释放队列资源 */
    vQueueDelete(gptimer_queue);
#endif
}

/*
 * 新手重点总结：
 *
 * 1. GPTimer 使用基本流程：
 *
 *    gptimer_new_timer()
 *        创建定时器
 *
 *    gptimer_register_event_callbacks()
 *        注册 alarm 中断回调
 *
 *    gptimer_enable()
 *        使能定时器
 *
 *    gptimer_set_alarm_action()
 *        设置 alarm_count
 *
 *    gptimer_start()
 *        启动计数
 *
 *    gptimer_stop()
 *        停止计数
 *
 *    gptimer_disable()
 *        禁用定时器
 *
 *    gptimer_del_timer()
 *        删除定时器
 *
 * 2. resolution_hz = 1000000 的含义：
 *
 *    定时器 1 秒数 1000000 次。
 *    所以 1 个 tick = 1us。
 *
 *    alarm_count = 1000      表示 1ms
 *    alarm_count = 10000     表示 10ms
 *    alarm_count = 1000000   表示 1s
 *
 * 3. ISR 中断函数里不要做复杂事情：
 *
 *    不建议在 ISR 中 printf、malloc、长时间计算。
 *    推荐 ISR 中只发队列、发信号量、置标志位。
 *
 * 4. 如果你后面做 FOC：
 *
 *    GPTimer 可以用来产生固定控制周期，
 *    比如每 100us / 200us / 1ms 执行一次控制算法。
 *
 *    但是真正输出三相 PWM，应该主要用 MCPWM 外设。
 *    GPTimer 更适合做控制循环调度、采样周期调度等。
 */
