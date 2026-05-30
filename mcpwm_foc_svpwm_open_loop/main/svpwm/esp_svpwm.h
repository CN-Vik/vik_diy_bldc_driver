/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/mcpwm_prelude.h"

/**
 * @brief SVPWM 三相逆变器配置结构体
 *
 * 通俗理解：
 * 这个结构体就是把“三相桥 PWM 输出”需要用到的配置集中放在一起。
 *
 * 三相 BLDC / PMSM 通常需要 6 路 PWM：
 *
 *      U 相：UH / UL
 *      V 相：VH / VL
 *      W 相：WH / WL
 *
 * 每一相都有上桥臂和下桥臂，所以总共是 3 组互补 PWM。
 */
typedef struct inverter_config {
    /*
     * MCPWM 定时器配置
     *
     * 作用：
     * 决定 PWM 的计数频率、计数模式、PWM 周期等。
     *
     * 举例：
     * resolution_hz = 10MHz，表示 MCPWM 计数器 1 秒计数 10000000 次。
     * period_ticks = 1000，表示一个 PWM 周期是 1000 个 tick。
     */
    mcpwm_timer_config_t timer_config;

    /*
     * MCPWM operator 配置
     *
     * 通俗理解：
     * operator 可以理解为 MCPWM 的一个“PWM 生成单元”。
     * 一般一相电机用一个 operator，所以三相电机通常用 3 个 operator。
     */
    mcpwm_operator_config_t operator_config;

    /*
     * MCPWM comparator 比较器配置
     *
     * 通俗理解：
     * comparator 用来决定 PWM 什么时候翻转。
     *
     * 举例：
     * 如果 PWM 周期是 1000，比较值是 250，
     * 那么计数器数到 250 的时候，就可以触发 PWM 电平变化。
     *
     * 所以我们平时说“设置占空比”，本质上就是在改 comparator 的比较值。
     */
    mcpwm_comparator_config_t compare_config;

    /*
     * 三相六路 PWM 输出 GPIO
     *
     * gen_gpios[0][0]：U 相上桥臂 UH
     * gen_gpios[0][1]：U 相下桥臂 UL
     *
     * gen_gpios[1][0]：V 相上桥臂 VH
     * gen_gpios[1][1]：V 相下桥臂 VL
     *
     * gen_gpios[2][0]：W 相上桥臂 WH
     * gen_gpios[2][1]：W 相下桥臂 WL
     */
    int gen_gpios[3][2];

    /*
     * 正向 PWM 死区配置
     *
     * 死区的作用：
     * 防止同一相的上桥臂 MOS 和下桥臂 MOS 同时导通。
     *
     * 如果上下桥臂同时导通，就相当于电源正极直接短到地，
     * 这叫“桥臂直通”，严重时会烧 MOS 管。
     */
    mcpwm_dead_time_config_t dt_config;

    /*
     * 反向 / 互补 PWM 死区配置
     *
     * 通俗理解：
     * 上桥臂和下桥臂通常是一对互补 PWM。
     *
     * 上桥臂导通时，下桥臂应该关闭；
     * 下桥臂导通时，上桥臂应该关闭；
     * 中间还要插入一点死区时间，避免同时导通。
     */
    mcpwm_dead_time_config_t inv_dt_config;
} inverter_config_t;

/*
 * 逆变器句柄类型
 *
 * 通俗理解：
 * inverter_handle_t 就像一个“对象指针”。
 *
 * svpwm_new_inverter() 创建成功后，会返回这个句柄。
 * 后面启动 PWM、设置占空比、删除逆变器，都要靠这个句柄找到对应的 MCPWM 资源。
 *
 * struct mcpwm_svpwm_ctx 的具体内容在 .c 文件里面实现，
 * 这里不暴露细节，只给外部一个句柄使用。
 */
typedef struct mcpwm_svpwm_ctx *inverter_handle_t;

/**
 * @brief 创建一个基于 MCPWM 的三相逆变器 PWM 输出对象
 *
 * 通俗理解：
 * 这个函数会根据 config 里面的配置，完成 MCPWM 的初始化。
 *
 * 它通常会做这些事情：
 * 1. 创建 MCPWM 定时器
 * 2. 创建 3 个 operator，对应 U/V/W 三相
 * 3. 创建 3 个 comparator，用来控制三相占空比
 * 4. 创建 6 个 generator，对应 UH/UL、VH/VL、WH/WL 六路 PWM
 * 5. 配置互补输出和死区
 *
 * @param[in]  config        输入：MCPWM 和三相 PWM 的配置参数
 * @param[out] ret_inverter  输出：创建成功后返回的逆变器句柄
 *
 * @return
 *      - ESP_OK：创建成功
 *      - ESP_ERR_INVALID_ARG：参数错误，例如传入了 NULL 指针
 *      - ESP_ERR_NO_MEM：内存不足，创建失败
 */
esp_err_t svpwm_new_inverter(const inverter_config_t *config, inverter_handle_t *ret_inverter);

/**
 * @brief 注册 MCPWM 定时器事件回调函数
 *
 * 通俗理解：
 * MCPWM 定时器运行过程中，会产生一些事件。
 *
 * 例如：
 * - 计数到周期顶部
 * - 计数到周期底部
 * - PWM 周期更新
 *
 * FOC 示例里通常会在 PWM 周期更新时通知任务：
 * “可以计算下一次 FOC / SVPWM 占空比了”。
 *
 * @param[in] handle    逆变器句柄
 * @param[in] event     MCPWM 定时器事件回调配置
 * @param[in] user_ctx  用户自定义参数，会原样传给回调函数
 *
 * @return
 *      - ESP_OK：注册成功
 *      - ESP_ERR_INVALID_ARG：参数错误，例如传入了 NULL 指针
 */
esp_err_t svpwm_inverter_register_cbs(inverter_handle_t handle, const mcpwm_timer_event_callbacks_t *event, void *user_ctx);

/**
 * @brief 启动或停止 SVPWM 逆变器 PWM 输出
 *
 * 通俗理解：
 * 这个函数本质上是在控制 MCPWM 定时器启动或停止。
 *
 * MCPWM 定时器启动后：
 * 计数器开始计数，比较器开始工作，PWM 引脚开始输出波形。
 *
 * command 参数可以使用 ESP-IDF 里的 mcpwm_timer_start_stop_cmd_t 枚举值。
 *
 * 常见用法：
 * - MCPWM_TIMER_START_NO_STOP：启动后一直运行
 * - MCPWM_TIMER_STOP_EMPTY：计数到空点时停止
 * - MCPWM_TIMER_STOP_FULL：计数到满点时停止
 *
 * @param[in] handle   逆变器句柄
 * @param[in] command  启动 / 停止命令
 *
 * @return
 *      - ESP_OK：启动或停止成功
 *      - ESP_ERR_INVALID_ARG：参数错误，例如句柄为空
 */
esp_err_t svpwm_inverter_start(inverter_handle_t handle, mcpwm_timer_start_stop_cmd_t command);

/**
 * @brief 设置三相 PWM 的比较值，也就是设置 U/V/W 三相占空比
 *
 * 通俗理解：
 * 这里的 u、v、w 不是百分比形式的 0%~100%，
 * 而是 MCPWM 计数器里的比较值。
 *
 * 举例：
 * 如果 PWM 周期 period_ticks = 1000：
 *
 *      u = 250  大约表示 U 相 25% 附近的比较位置
 *      v = 500  大约表示 V 相 50% 附近的比较位置
 *      w = 750  大约表示 W 相 75% 附近的比较位置
 *
 * 具体占空比关系还要结合 MCPWM 的计数模式和 generator action 设置来看。
 *
 * 在这个 FOC 示例里：
 * FOC / SVPWM 算法最终会得到 U/V/W 三相输出值，
 * 然后通过这个函数写入 MCPWM comparator，
 * 最后 MCPWM 硬件自动输出三相 PWM 波形。
 *
 * @param[in] handle  逆变器句柄
 * @param[in] u       U 相比较值，对应 UH / UL 这一路互补 PWM
 * @param[in] v       V 相比较值，对应 VH / VL 这一路互补 PWM
 * @param[in] w       W 相比较值，对应 WH / WL 这一路互补 PWM
 *
 * @return
 *      - ESP_OK：设置成功
 *      - ESP_ERR_INVALID_ARG：参数错误，例如句柄为空
 */
esp_err_t svpwm_inverter_set_duty(inverter_handle_t handle, uint16_t u, uint16_t v, uint16_t w);

/**
 * @brief 删除 SVPWM 逆变器对象，释放 MCPWM 相关资源
 *
 * 通俗理解：
 * 如果后面不再使用这个三相 PWM 输出，就调用这个函数释放资源。
 *
 * 它通常会释放：
 * 1. PWM generator
 * 2. PWM comparator
 * 3. MCPWM operator
 * 4. MCPWM timer
 * 5. 逆变器上下文结构体内存
 *
 * @param[in] handle  逆变器句柄
 *
 * @return
 *      - ESP_OK：释放成功
 *      - ESP_ERR_INVALID_ARG：参数错误，例如句柄为空
 */
esp_err_t svpwm_del_inverter(inverter_handle_t handle);
