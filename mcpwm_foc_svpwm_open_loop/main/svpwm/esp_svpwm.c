/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 说明：
 * 这个文件是 ESP32 MCPWM 三相逆变器封装代码。
 *
 * 注意：
 * 本文件没有使用 IQmathLib.h，也没有使用 _IQ、_iq、_IQmpy 等定点数学库。
 * 它主要负责 MCPWM 硬件资源的创建、配置、启动、设置占空比和释放。
 *
 * FOC 数学计算部分一般在 esp_foc.c 里；
 * 这里更多是“把算出来的 U/V/W 三相占空比真正输出到 GPIO 引脚上”。
 */

#include <stdlib.h> // 使用 calloc/free 需要包含这个头文件

#include "esp_log.h"   // ESP_LOGI / ESP_LOGE 等日志打印
#include "esp_check.h" // ESP_RETURN_ON_FALSE / ESP_GOTO_ON_ERROR 等错误检查宏
#include "esp_svpwm.h" // 本模块对应的头文件，里面声明了 inverter_config_t 等类型

/*
 * TAG 用于 ESP-IDF 日志输出。
 *
 * 例如：
 * ESP_LOGE(TAG, "no memory");
 *
 * 串口日志里就会显示类似：
 * E (xxx) esp_svpwm: no memory
 */
static const char *TAG = "esp_svpwm";

/*
 * MCPWM SVPWM 上下文结构体
 *
 * 通俗理解：
 * 这个结构体用来保存一整套三相 PWM 输出需要用到的 MCPWM 资源。
 *
 * 三相电机需要 U/V/W 三相输出：
 *
 *      U 相：UH / UL
 *      V 相：VH / VL
 *      W 相：WH / WL
 *
 * 每一相都有：
 * 1. operator    ：PWM 运算单元
 * 2. comparator  ：比较器，用来决定占空比
 * 3. generator   ：真正控制 GPIO 输出 PWM 的发生器
 *
 * timer 是三相共用的 PWM 定时器。
 * 三相共用一个 timer 可以保证 U/V/W 三相 PWM 同步。
 */
typedef struct mcpwm_svpwm_ctx
{
    mcpwm_timer_handle_t timer;          // MCPWM 定时器句柄，三相 PWM 共用同一个定时器
    mcpwm_oper_handle_t operators[3];    // 3 个 operator，分别对应 U/V/W 三相
    mcpwm_cmpr_handle_t comparators[3];  // 3 个 comparator，分别控制 U/V/W 三相占空比
    mcpwm_gen_handle_t generators[3][2]; // 3 相 x 每相 2 路输出，上桥臂和下桥臂
} mcpwm_svpwm_ctx_t;

/**
 * @brief 创建一个三相 SVPWM 逆变器对象
 *
 * 通俗理解：
 * 这个函数就是把 ESP32 的 MCPWM 外设配置成“三相六路 PWM 输出”。
 *
 * 它做的事情大概是：
 *
 * 1. 申请一个 mcpwm_svpwm_ctx_t 结构体，用来保存 MCPWM 资源句柄
 * 2. 创建一个 MCPWM timer，作为 PWM 的时间基准
 * 3. 创建 3 个 MCPWM operator，分别对应 U/V/W 三相
 * 4. 把 3 个 operator 都连接到同一个 timer，保证三相同步
 * 5. 创建 3 个 comparator，用来设置 U/V/W 三相占空比
 * 6. 创建 6 个 generator，对应 UH/UL、VH/VL、WH/WL 六路 PWM 引脚
 * 7. 配置 generator 的动作规则，让它根据 comparator 输出 PWM
 * 8. 配置死区，避免上下桥臂同时导通
 *
 * @param[in]  config        输入配置，包括 timer/operator/comparator/GPIO/死区等
 * @param[out] ret_inverter  输出创建好的逆变器句柄
 *
 * @return
 *      - ESP_OK：创建成功
 *      - ESP_ERR_INVALID_ARG：参数错误，比如 config 或 ret_inverter 是 NULL
 *      - ESP_ERR_NO_MEM：内存不足
 */
esp_err_t svpwm_new_inverter(const inverter_config_t *config, inverter_handle_t *ret_inverter)
{
    esp_err_t ret;

    /*
     * 检查参数是否合法。
     *
     * config 是用户传进来的配置；
     * ret_inverter 用来返回创建好的逆变器句柄。
     *
     * 如果这两个指针有任何一个是 NULL，就直接返回参数错误。
     */
    ESP_RETURN_ON_FALSE(config && ret_inverter, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    /*
     * 申请一块内存保存 MCPWM 相关资源句柄。
     *
     * calloc 会把申请到的内存清零。
     * 这样 timer、operators、comparators、generators 这些句柄初始都是 NULL。
     */
    mcpwm_svpwm_ctx_t *svpwm_dev = calloc(1, sizeof(mcpwm_svpwm_ctx_t));
    if (!svpwm_dev)
    {
        ESP_LOGE(TAG, "no memory");
        return ESP_ERR_NO_MEM;
    }

    /*
     * 创建 MCPWM timer。
     *
     * timer 决定 PWM 的基础周期和计数方式。
     *
     * 例如：
     * - resolution_hz 决定计数频率
     * - period_ticks 决定 PWM 周期
     * - count_mode 决定向上计数、向下计数还是上下计数
     *
     * FOC/SVPWM 通常喜欢使用上下计数模式，
     * 因为它可以生成中心对齐 PWM，波形更对称。
     */
    ESP_GOTO_ON_ERROR(mcpwm_new_timer(&config->timer_config, &svpwm_dev->timer),
                      err, TAG, "Create MCPWM timer failed");

    /*
     * 创建 3 个 MCPWM operator。
     *
     * 通俗理解：
     * 一个 operator 可以管理一路 PWM 逻辑。
     * 三相电机有 U/V/W 三相，所以这里创建 3 个 operator。
     *
     * 每个 operator 都连接到同一个 timer。
     * 这样三相 PWM 会使用同一个时间基准，保证同步。
     */
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(mcpwm_new_operator(&config->operator_config, &svpwm_dev->operators[i]),
                          err, TAG, "Create MCPWM operator failed");

        ESP_GOTO_ON_ERROR(mcpwm_operator_connect_timer(svpwm_dev->operators[i], svpwm_dev->timer),
                          err, TAG, "Connect operators to the same timer failed");
    }

    /*
     * 创建 3 个 comparator。
     *
     * comparator 的作用：
     * 当 timer 计数到 comparator 的比较值时，触发 PWM 输出电平变化。
     *
     * 所以设置 comparator 的值，本质上就是设置 PWM 的占空比。
     *
     * 这里先把比较值设置为 0，避免刚初始化时输出未知占空比。
     */
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(mcpwm_new_comparator(svpwm_dev->operators[i],
                                               &config->compare_config,
                                               &svpwm_dev->comparators[i]),
                          err, TAG, "Create comparators failed");

        ESP_GOTO_ON_ERROR(mcpwm_comparator_set_compare_value(svpwm_dev->comparators[i], 0),
                          err, TAG, "Set comparators failed");
    }

    /*
     * 创建 6 个 PWM generator。
     *
     * generator 是真正绑定 GPIO 引脚并输出 PWM 波形的模块。
     *
     * gen_gpios[3][2] 的含义：
     *
     *      gen_gpios[0][0] -> U 相上桥臂 UH
     *      gen_gpios[0][1] -> U 相下桥臂 UL
     *
     *      gen_gpios[1][0] -> V 相上桥臂 VH
     *      gen_gpios[1][1] -> V 相下桥臂 VL
     *
     *      gen_gpios[2][0] -> W 相上桥臂 WH
     *      gen_gpios[2][1] -> W 相下桥臂 WL
     */
    mcpwm_generator_config_t gen_config = {};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            gen_config.gen_gpio_num = config->gen_gpios[i][j];

            ESP_GOTO_ON_ERROR(mcpwm_new_generator(svpwm_dev->operators[i],
                                                  &gen_config,
                                                  &svpwm_dev->generators[i][j]),
                              err, TAG, "Create PWM generator pin %d failed", gen_config.gen_gpio_num);
        }
    }

    /*
     * 配置每一相上桥臂 generator 的动作规则。
     *
     * 这里使用的是上下计数模式。
     *
     * 当 timer 向上计数，并且计数值等于 comparator 值时：
     *      generator 输出低电平
     *
     * 当 timer 向下计数，并且计数值等于 comparator 值时：
     *      generator 输出高电平
     *
     * 这样可以生成中心对齐 PWM。
     *
     * 中心对齐 PWM 的优点：
     * - 波形更对称
     * - 谐波特性更好
     * - 电机控制里比较常用
     */
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(
                              svpwm_dev->generators[i][0],
                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                             svpwm_dev->comparators[i],
                                                             MCPWM_GEN_ACTION_LOW),
                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                                             svpwm_dev->comparators[i],
                                                             MCPWM_GEN_ACTION_HIGH),
                              MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
                          err, TAG, "Set generator actions failed");
    }

    /*
     * 配置死区 dead time。
     *
     * 为什么需要死区？
     *
     * 一相半桥有上 MOS 和下 MOS。
     * 如果上下 MOS 同时导通，就会造成电源正极直接短路到地，
     * 这叫“桥臂直通”，非常危险，可能直接烧 MOS 管或驱动芯片。
     *
     * 所以在上 MOS 关断到下 MOS 导通之间，
     * 或者下 MOS 关断到上 MOS 导通之间，
     * 需要插入一小段延迟时间，这个时间就叫死区。
     *
     * 这里的逻辑是：
     * - generators[i][0] 是原始 PWM / 上桥臂 PWM
     * - generators[i][1] 是通过死区模块生成的互补 PWM / 下桥臂 PWM
     */
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(mcpwm_generator_set_dead_time(svpwm_dev->generators[i][0],
                                                        svpwm_dev->generators[i][0],
                                                        &config->dt_config),
                          err, TAG, "Setup deadtime failed");

        ESP_GOTO_ON_ERROR(mcpwm_generator_set_dead_time(svpwm_dev->generators[i][0],
                                                        svpwm_dev->generators[i][1],
                                                        &config->inv_dt_config),
                          err, TAG, "Setup inv deadtime failed");
    }

    /*
     * 所有 MCPWM 资源创建成功后，把句柄返回给调用者。
     *
     * 后面启动 PWM、设置占空比、释放资源时，
     * 都要通过这个 inverter_handle_t 找到对应的 MCPWM 资源。
     */
    *ret_inverter = svpwm_dev;
    return ESP_OK;

err:
    /*
     * 如果中间任何一步创建失败，就跳到这里。
     *
     * 注意：
     * 这里原始示例只 free 了 svpwm_dev 结构体，
     * 没有逐个释放已经创建成功的 MCPWM 资源。
     *
     * 对于示例工程一般问题不大；
     * 如果是正式产品代码，建议在这里补充更完整的资源释放逻辑。
     */
    free(svpwm_dev);
    return ret;
}

/**
 * @brief 注册 MCPWM 定时器事件回调
 *
 * 通俗理解：
 * MCPWM 运行时会产生一些事件。
 *
 * 例如：
 * - 计数到顶部
 * - 计数到底部
 * - 一个 PWM 周期结束
 *
 * FOC 示例里通常会利用这些事件来同步控制算法：
 * 每到一个固定 PWM 更新点，就通知任务计算下一组三相占空比。
 *
 * @param[in] handle    逆变器句柄
 * @param[in] event     回调函数配置
 * @param[in] user_ctx  用户自定义参数，会传给回调函数
 *
 * @return
 *      - ESP_OK：注册成功
 *      - ESP_ERR_INVALID_ARG：参数错误
 */
esp_err_t svpwm_inverter_register_cbs(inverter_handle_t handle,
                                      const mcpwm_timer_event_callbacks_t *event,
                                      void *user_ctx)
{
    /*
     * handle 不能为空，因为里面保存了 MCPWM timer。
     * event 不能为空，因为里面保存了要注册的回调函数。
     */
    ESP_RETURN_ON_FALSE(handle && event, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    /*
     * 把用户配置的回调函数注册到 MCPWM timer 上。
     *
     * user_ctx 是用户自定义上下文。
     * 示例代码里通常会传一个信号量指针，
     * 在中断回调里释放信号量，通知主任务进行下一次 FOC 计算。
     */
    ESP_RETURN_ON_ERROR(mcpwm_timer_register_event_callbacks(handle->timer, event, user_ctx),
                        TAG, "register callbacks failed");

    return ESP_OK;
}

/**
 * @brief 启动或停止三相 SVPWM 逆变器
 *
 * 通俗理解：
 * 这个函数实际控制的是 MCPWM timer。
 *
 * timer 启动后：
 * - 计数器开始计数
 * - comparator 开始比较
 * - generator 开始按规则输出 PWM
 * - 三相 PWM 引脚开始有波形
 *
 * @param[in] handle   逆变器句柄
 * @param[in] command  启动或停止命令，例如 MCPWM_TIMER_START_NO_STOP
 *
 * @return
 *      - ESP_OK：操作成功
 *      - ESP_ERR_INVALID_ARG：参数错误
 */
esp_err_t svpwm_inverter_start(inverter_handle_t handle, mcpwm_timer_start_stop_cmd_t command)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    /*
     * 如果 command 不是停止命令，就先使能 MCPWM timer。
     *
     * MCPWM 定时器通常需要先 enable，再 start。
     */
    if ((command != MCPWM_TIMER_STOP_EMPTY) && (command != MCPWM_TIMER_STOP_FULL))
    {
        ESP_RETURN_ON_ERROR(mcpwm_timer_enable(handle->timer),
                            TAG, "mcpwm timer enable failed");
    }

    /*
     * 根据 command 启动或停止 MCPWM timer。
     *
     * 常见 command：
     * - MCPWM_TIMER_START_NO_STOP：启动后一直运行
     * - MCPWM_TIMER_STOP_EMPTY：计数到 empty 点停止
     * - MCPWM_TIMER_STOP_FULL：计数到 full 点停止
     */
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(handle->timer, command),
                        TAG, "mcpwm timer start failed");

    return ESP_OK;
}

/**
 * @brief 设置 U/V/W 三相 PWM 比较值
 *
 * 通俗理解：
 * 这个函数就是把 FOC/SVPWM 算出来的三相占空比写进 MCPWM 硬件。
 *
 * 注意：
 * 这里传入的 u、v、w 不是 0.0~1.0 的小数占空比，
 * 而是 MCPWM comparator 的比较值。
 *
 * 举例：
 * 如果 PWM 周期 period_ticks = 1000，
 * 那么比较值大概会在 0~1000 范围内变化。
 *
 * 但因为示例用的是上下计数中心对齐 PWM，
 * 实际 duty 映射方式要结合 generator action 一起看。
 *
 * @param[in] handle  逆变器句柄
 * @param[in] u       U 相 PWM 比较值
 * @param[in] v       V 相 PWM 比较值
 * @param[in] w       W 相 PWM 比较值
 *
 * @return
 *      - ESP_OK：设置成功
 *      - ESP_ERR_INVALID_ARG：参数错误
 */
esp_err_t svpwm_inverter_set_duty(inverter_handle_t handle, uint16_t u, uint16_t v, uint16_t w)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    /*
     * 设置 U 相比较值。
     *
     * comparators[0] 对应 U 相。
     * 更新它之后，U 相 PWM 占空比会随之变化。
     */
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[0], u),
                        TAG, "set duty failed");

    /*
     * 设置 V 相比较值。
     */
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[1], v),
                        TAG, "set duty failed");

    /*
     * 设置 W 相比较值。
     */
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(handle->comparators[2], w),
                        TAG, "set duty failed");

    return ESP_OK;
}

/**
 * @brief 删除三相 SVPWM 逆变器，释放 MCPWM 资源
 *
 * 通俗理解：
 * 如果后面不再使用三相 PWM 输出，就调用这个函数清理资源。
 *
 * 它会释放：
 * 1. generator
 * 2. comparator
 * 3. operator
 * 4. timer
 * 5. mcpwm_svpwm_ctx_t 内存
 *
 * @param[in] handle  逆变器句柄
 *
 * @return
 *      - ESP_OK：释放成功
 *      - ESP_ERR_INVALID_ARG：参数错误
 */
esp_err_t svpwm_del_inverter(inverter_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    /*
     * 删除 MCPWM timer 之前，先 disable。
     *
     * 这样可以避免定时器还在运行时就删除资源。
     */
    ESP_RETURN_ON_ERROR(mcpwm_timer_disable(handle->timer),
                        TAG, "mcpwm timer disable failed");

    /*
     * 逐相释放 MCPWM 资源。
     *
     * i = 0：U 相
     * i = 1：V 相
     * i = 2：W 相
     */
    for (int i = 0; i < 3; i++)
    {
        /*
         * 删除上桥臂 generator。
         */
        ESP_RETURN_ON_ERROR(mcpwm_del_generator(handle->generators[i][0]),
                            TAG, "free mcpwm positive generator failed");

        /*
         * 删除下桥臂 generator。
         */
        ESP_RETURN_ON_ERROR(mcpwm_del_generator(handle->generators[i][1]),
                            TAG, "free mcpwm negative generator failed");

        /*
         * 删除这一相的 comparator。
         */
        ESP_RETURN_ON_ERROR(mcpwm_del_comparator(handle->comparators[i]),
                            TAG, "free mcpwm comparator failed");

        /*
         * 删除这一相的 operator。
         */
        ESP_RETURN_ON_ERROR(mcpwm_del_operator(handle->operators[i]),
                            TAG, "free mcpwm operator failed");
    }

    /*
     * 删除三相共用的 MCPWM timer。
     */
    ESP_RETURN_ON_ERROR(mcpwm_del_timer(handle->timer),
                        TAG, "free mcpwm timer failed");

    /*
     * 最后释放保存这些句柄的结构体内存。
     */
    free(handle);

    return ESP_OK;
}
