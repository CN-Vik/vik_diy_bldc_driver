/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 说明：
 * 这个文件是 AS5600 磁编码器的测试代码。
 *
 * AS5600 是一个 12bit 磁编码器：
 * - 通过磁铁旋转角度，输出 0 ~ 4095 的原始角度值
 * - 可以换算成 0 ~ 360 度
 * - 常用于 FOC 电机位置检测、旋钮、角度传感器等场景
 *
 * 这里主要测试三个功能：
 * 1. 读取当前角度
 * 2. 设置当前位置为 0 度，也就是 ZPOS 零点校准
 * 3. 写入并读回 AS5600 的配置寄存器 CONF
 *
 *
 * 初始化 I2C
    ↓
创建 AS5600 设备
    ↓
执行测试：
    1. 读角度
    2. 设置当前位置为 0 度
    3. 写配置再读回来校验
    ↓
删除 AS5600 设备
    ↓
删除 I2C 总线


as5600_test_get_angle();          // 读当前角度
as5600_test_zero_calibration();   // 设置当前位置为 0 度
as5600_test_conf_readback();      // 测试配置寄存器读写

另外有个小建议：正式写 FOC 时，不建议频繁调用 as5600_set_zero_position()。
零点一般是在电机安装、校准阶段设置一次，运行过程中主要是高速读取角度。
 */

#include "motor_angle_acqu.h"
#include "as5600.h"
#include "driver/i2c_master.h"
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "app_rtos_config.h"
#include "vik_foc.h"
#include "app_rtos_resource.h"


extern void set_vfoc_theta_m_deg(float parm_angle);
extern float get_vfoc_theta_m_deg(void);
extern void set_vfoc_mech_w(float w_mech);
void set_vfoc_mech_rpm(float mech_rm);


static const char *TAG = "AS5600";

typedef struct
{
    float angle_last;      // 上一次AS5600角度
    float angle_total;     // 展开后的连续角度

} AngleUnwrap_t;

AngleUnwrap_t unwrap={0.0f};

typedef struct
{
    float theta;       // 估计角度

    float omega;       // 估计角速度 deg/s

    float kp;

    float ki;

    float dt;
} PLL_t;

PLL_t pll;


/* ========================== 磁铁状态字符串表 ========================== */

/*
 * AS5600 会检测磁铁状态。
 *
 * 磁铁状态很重要：
 * - 磁铁太远，磁场太弱，角度可能不准
 * - 磁铁太近，磁场太强，角度也可能不准
 * - 没检测到磁铁，则角度数据基本不可用
 *
 * 这里用字符串表把枚举值转换成人能看懂的文本，方便 printf 打印。
 */
static const char *const s_as5600_magnet_status_str[] = {
    [AS5600_MAGNET_OK] = "OK",
    [AS5600_MAGNET_TOO_WEAK] = "too weak",
    [AS5600_MAGNET_TOO_STRONG] = "too strong",
    [AS5600_MAGNET_NOT_DETECTED] = "not detected",
};

TaskHandle_t motor_get_angle_task_handle = NULL;





/* ========================== 全局句柄 ========================== */

/*
 * AS5600 设备句柄。
 *
 * 驱动初始化成功后，会得到一个 as5600_handle_t。
 * 后面读角度、读状态、写配置，都需要传入这个句柄。
 */
static as5600_handle_t as5600 = NULL;

/*
 * I2C 总线句柄。
 *
 * ESP-IDF 新版 I2C 驱动中，先创建 I2C bus，
 * 再把具体 I2C 设备挂到这个 bus 上。
 */
static i2c_master_bus_handle_t bus_handle = NULL;

int64_t angle_time_stamp; /*时间戳,单位:us*/



void angle_unwrap_init(AngleUnwrap_t *obj, float angle)
{
    obj->angle_last = angle;
    obj->angle_total = angle;
}



float angle_unwrap_update(AngleUnwrap_t *obj, float angle)
{
    float delta;

    delta = angle - obj->angle_last;

    // 跨越0度
    if(delta > 180.0f)
    {
        delta -= 360.0f;
    }

    else if(delta < -180.0f)
    {
        delta += 360.0f;
    }

    obj->angle_total += delta;

    obj->angle_last = angle;

    return obj->angle_total;
}


void pll_init(PLL_t *pll,float theta)
{
    pll->theta = theta;

    pll->omega = 0;

    pll->kp = 30.0f;

    pll->ki = 500.0f;

    pll->dt = 0.001f;

}



float limit_angle_error(float error)
{

    while(error > 180)
        error -= 360;


    while(error < -180)
        error += 360;


    return error;
}




void pll_update(
    PLL_t *pll,
    float angle_measure
)
{

    float error;

    error =
        angle_measure - pll->theta;

    pll->omega +=
        pll->ki *
        error *
        pll->dt;

    pll->theta +=
        pll->omega *
        pll->dt
        +
        pll->kp *
        error *
        pll->dt;

}



/*
 * 判断角度是否接近 0 度，或者接近 360 度。
 *
 * 为什么要判断接近 360 度？
 *
 * 因为角度是环形的：
 * - 0 度
 * - 359.9 度
 *
 * 在物理意义上其实很接近。
 *
 * 比如设置零点后，读到 358 度，也可能只是因为角度在 0 度附近发生了回绕。
 */
static bool as5600_degrees_near_zero_or_wrap_360(float deg, float tol_deg)
{
    /*
     * deg <= tol_deg：
     * 例如 deg = 1.2，tol_deg = 5，说明角度接近 0 度。
     *
     * deg >= 360.0f - tol_deg：
     * 例如 deg = 358，tol_deg = 5，说明角度接近 360 度。
     * 360 度和 0 度本质上是同一个方向。
     */
    return (deg <= tol_deg) || (deg >= 360.0f - tol_deg);
}


/* ========================== AS5600 初始化函数 ========================== */

/*
 * 初始化 I2C 总线，并创建 AS5600 传感器对象。
 *
 * 这个函数做两件事：
 * 1. 初始化 ESP32 的 I2C Master 总线
 * 2. 在这个 I2C 总线上添加 AS5600 设备
 */
static int as5600_test_init(void)
{
    esp_err_t ret;

    /*
     * I2C Master 总线配置结构体。
     *
     * 这里配置的是 ESP32 作为 I2C 主机：
     * - 使用哪个 I2C 控制器
     * - SDA 引脚
     * - SCL 引脚
     * - 时钟源
     * - 毛刺过滤
     * - 是否开启内部上拉
     */
    i2c_master_bus_config_t bus_config = {
        /*
         * 使用 I2C_NUM_0 控制器。
         */
        .i2c_port = I2C_MASTER_NUM,

        /*
         * SDA 数据线 GPIO。
         */
        .sda_io_num = I2C_MASTER_SDA_IO,

        /*
         * SCL 时钟线 GPIO。
         */
        .scl_io_num = I2C_MASTER_SCL_IO,

        /*
         * I2C 时钟源。
         *
         * I2C_CLK_SRC_DEFAULT 表示使用 ESP-IDF 默认推荐的时钟源。
         */
        .clk_source = I2C_CLK_SRC_DEFAULT,

        /*
         * 毛刺过滤参数。
         *
         * I2C 信号线上可能会有很短的干扰毛刺。
         * glitch_ignore_cnt = 7 表示过滤掉非常短的脉冲干扰。
         */
        .glitch_ignore_cnt = 7,

        /*
         * 使能 ESP32 内部上拉电阻。
         *
         * I2C 总线是开漏结构，SDA/SCL 必须有上拉。
         *
         * 注意：
         * 内部上拉通常比较弱，正式硬件建议外接 4.7k 左右上拉电阻。
         */
        .flags.enable_internal_pullup = true,
    };

    /*
     * 创建 I2C Master 总线。
     *
     * 成功后，bus_handle 会被赋值。
     */
    ret = i2c_new_master_bus(&bus_config, &bus_handle);

    /*
     * Unity 断言：
     * 判断 i2c_new_master_bus() 的返回值是否等于 ESP_OK。
     *
     * 如果不等于 ESP_OK，测试直接失败。
     */
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * AS5600 I2C 设备配置。
     *
     * 这里只配置了 I2C 速度。
     * 设备地址通常在驱动里固定为 AS5600 默认地址 0x36。
     */
    as5600_i2c_config_t i2c_conf = {
        .scl_speed_hz = I2C_MASTER_FREQ_HZ};

    /*
     * 在 I2C bus 上创建 AS5600 传感器对象。
     *
     * 参数说明：
     * - bus_handle：前面创建好的 I2C 总线
     * - &i2c_conf：AS5600 的 I2C 配置
     * - &as5600：输出参数，用来保存 AS5600 设备句柄
     */
    ret = as5600_new_sensor(bus_handle, &i2c_conf, &as5600);

    /*
     * 检查 AS5600 设备创建是否成功。
     */
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_ERROR_CHECK(ret);

    /*
     * 检查 as5600 句柄不为空。
     * 如果为空，说明 AS5600 设备对象没有创建成功。
     */
    if (as5600 == NULL)
    {
        ESP_LOGE(TAG, "as5600 handle is NULL");
        return ESP_FAIL;
    }

    return 0;
}

/* ========================== AS5600 反初始化函数 ========================== */

/*
 * 释放 AS5600 设备对象，并删除 I2C 总线。
 *
 * 每个 TEST_CASE 结束后都调用这个函数，
 * 避免资源泄漏，也避免影响下一个测试用例。
 */
static int as5600_test_deinit(void)
{
    /*
     * 如果 AS5600 句柄不为空，说明之前初始化成功过。
     */
    if (as5600 != NULL)
    {
        /*
         * 删除 AS5600 设备对象。
         */
        // TEST_ASSERT_EQUAL(ESP_OK, as5600_del_sensor(as5600));

        esp_err_t ret = as5600_del_sensor(as5600);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "as5600_del_sensor failed: %s", esp_err_to_name(ret));
        }

        /*
         * 删除后把句柄清空，避免野指针。
         */
        as5600 = NULL;
    }

    /*
     * 如果 I2C bus 句柄不为空，说明 I2C 总线创建成功过。
     */
    if (bus_handle != NULL)
    {
        /*
         * 删除 I2C Master 总线。
         */
        i2c_del_master_bus(bus_handle);

        /*
         * 删除后清空句柄。
         */
        bus_handle = NULL;
    }

    return 0;
}

/* ========================== 测试 1：读取角度 ========================== */

/*
 * 测试读取 AS5600 当前角度。
 *
 * 读取内容包括：
 * 1. raw 原始角度值，范围一般是 0 ~ 4095
 * 2. degrees 换算后的角度值，范围一般是 0 ~ 360 度
 * 3. magnet status 磁铁状态
 */
static int as5600_test_get_angle(void)
{
    /*
     * raw 用来保存 AS5600 的 12bit 原始角度值。
     */
    uint16_t raw;

    /*
     * degrees 用来保存换算后的角度值。
     */
    float degrees;

    /*
     * status 用来保存磁铁状态。
     */
    as5600_magnet_status_t status;

    esp_err_t ret;

    while (1)
    {

        /*
         * 读取 AS5600 原始角度值。
         *
         * 如果这里失败，可能原因：
         * - AS5600 没接好
         * - I2C 地址不对
         * - SDA/SCL 接反
         * - 没有上拉电阻
         * - AS5600 没供电
         */
        ret = as5600_get_angle_raw(as5600, &raw);

        /*
         * 如果读原始角度失败，说明设备可能没有 ACK。
         *
         * 这里没有用 TEST_ASSERT，而是直接打印后 return。
         * 这样做的效果是：
         * - 设备没连接时，不让整个测试硬崩
         * - 打印提示方便排查硬件连接
         */
        if (ret != ESP_OK)
        {
            printf("AS5600 not connected or no ack\n");
            return 2;
        }

        /*
         * 读取角度，并换算成角度单位 degree。
         *
         * 驱动内部一般会做类似换算：
         * degrees = raw * 360.0 / 4096.0
         */
        ret = as5600_get_angle_degrees(as5600, &degrees);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
            return ret;
        }

        /*
         * 读取磁铁状态。
         *
         * 角度值是否可靠，很大程度取决于磁铁状态是否 OK。
         */
        ret = as5600_get_magnet_status(as5600, &status);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
            return ret;
        }

        /*
         * 根据 status 枚举值，从字符串表中取出对应文本。
         *
         * 这里做了边界判断：
         * 如果 status 超过字符串表范围，就显示 unknown，避免数组越界。
         */
        const char *status_str = ((unsigned)status < AS5600_MAGNET_STATUS_STR_COUNT)
                                     ? s_as5600_magnet_status_str[status]
                                     : "unknown";

        /*
         * 打印当前 AS5600 状态：
         *
         * raw：
         * - 原始角度值
         * - 0 ~ 4095
         *
         * degrees：
         * - 换算后的角度
         * - 0 ~ 360 度
         *
         * magnet status：
         * - 磁铁状态
         */

        ESP_LOGI(TAG, "AS5600_raw: %u, degrees: %.2f, magnet status: %s\n",
                 (unsigned)raw,
                 (float)degrees,
                 status_str);

        vTaskDelay(pdTICKS_TO_MS(1));
    }

    return 0;
}

/* ========================== 测试 2：ZPOS 零点校准 ========================== */

/*
 * 测试 AS5600 的零点校准功能。
 *
 * 所谓零点校准：
 * 当前磁铁停在某个位置时，把这个位置设置为 0 度。
 *
 * 例如：
 * 校准前当前位置读出来是 123 度。
 * 调用 as5600_set_zero_position() 后，
 * 再读当前位置，理论上应该接近 0 度。
 */
static int as5600_test_zero_calibration(void)
{
    esp_err_t ret;
    as5600_magnet_status_t status;

    /*
     * 先读取磁铁状态。
     *
     * 零点校准必须在磁铁状态正常时才有意义。
     */
    ret = as5600_get_magnet_status(as5600, &status);

    /*
     * 如果磁铁状态读取失败，说明 I2C 通信或者设备状态异常。
     * 这里直接跳过零点校准测试。
     */
    if (ret != ESP_OK)
    {
        printf("AS5600 magnet status read failed, skip zero calibration check\n");
        return -1;
    }

    /*
     * 如果磁铁状态不是 OK，也跳过测试。
     *
     * 因为磁场太弱、太强、没检测到磁铁时，
     * 读出来的角度不可靠，做零点校准没有意义。
     */
    if (status != AS5600_MAGNET_OK)
    {
        printf("AS5600 magnet not OK (status=%d), skip zero calibration check\n", (int)status);
        return -1;
    }

    /*
     * degrees_before 保存校准前角度。
     */
    float degrees_before;

    /*
     * 读取校准前的角度。
     */
    ret = as5600_get_angle_degrees(as5600, &degrees_before);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 打印校准前角度，方便观察校准效果。
     */
    printf("AS5600 before ZPOS cal: degrees=%.2f\n", (double)degrees_before);

    /*
     * 设置当前位置为零点。
     *
     * 注意：
     * 这个函数具体是写 AS5600 的 ZPOS 寄存器，
     * 还是只在驱动内部做软件零点偏移，
     * 要看 as5600_set_zero_position() 的驱动实现。
     */
    ret = as5600_set_zero_position(as5600);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 设置零点后等待一小段时间。
     *
     * 原因：
     * AS5600 内部可能有滤波器，
     * 写入 ZPOS 后 ANGLE 输出不一定立刻稳定。
     *
     * 这里延时 20ms，让滤波后的角度输出稳定一下。
     */
    vTaskDelay(pdMS_TO_TICKS(20));

    /*
     * degrees 保存校准后的角度。
     */
    float degrees;

    /*
     * 读取校准后的角度。
     */
    ret = as5600_get_angle_degrees(as5600, &degrees);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 打印校准后的角度。
     *
     * 期望值：
     * - 接近 0 度
     * - 或者接近 360 度
     */
    printf("AS5600 after ZPOS cal: degrees=%.2f (expect ~0 or ~360 wrap)\n",
           (double)degrees);

    /*
     * 判断校准后的角度是否接近 0 度或者接近 360 度。
     *
     * 如果不满足，测试失败，并打印错误信息。
     */
    if (!as5600_degrees_near_zero_or_wrap_360(degrees, AS5600_ZERO_CAL_TOL_DEG))
    {
        ESP_LOGE(TAG, "angle after zero calibration failed, degrees=%.2f", (double)degrees);
        return ESP_FAIL;
    }

    return 0;
}

/* ========================== 测试 3：CONF 配置寄存器写入和读回 ========================== */

/*
 * 测试 AS5600 的 CONF 配置寄存器。
 *
 * 测试思路：
 * 1. 写入一组配置
 * 2. 再把配置读出来
 * 3. 对比读出来的值和写进去的值是否一致
 * 4. 测试结束后恢复默认配置
 */
static int as5600_test_conf_readback(void)
{
    /*
     * 要写入 AS5600 的配置。
     *
     * AS5600 的 CONF 寄存器里通常包含：
     * - 电源模式
     * - 磁滞设置
     * - 输出模式
     * - PWM 频率
     * - 慢滤波
     * - 快滤波阈值
     * - 看门狗开关
     */
    const as5600_conf_t conf_write = {
        /*
         * 电源模式：低功耗模式 LPM1。
         *
         * 低功耗模式下功耗会下降，
         * 但是响应速度可能也会受影响。
         */
        .power_mode = AS5600_POWER_MODE_LPM1,

        /*
         * 磁滞设置：2 LSB。
         *
         * 磁滞可以抑制角度在边界附近来回抖动。
         */
        .hysteresis = AS5600_HYSTERESIS_2_LSB,

        /*
         * 输出级设置：模拟输出缩小范围。
         *
         * AS5600 除了 I2C 读角度外，还可以通过 OUT 引脚输出模拟电压或 PWM。
         */
        .output_stage = AS5600_OUTPUT_STAGE_ANALOG_REDUCED,

        /*
         * PWM 输出频率：460Hz。
         *
         * 如果使用 AS5600 的 PWM 输出功能，这个参数才比较关键。
         */
        .pwm_freq = AS5600_PWM_FREQ_460HZ,

        /*
         * 慢滤波设置：4X。
         *
         * 滤波越强，角度数据越平滑，
         * 但是响应速度会变慢。
         */
        .slow_filter = AS5600_SLOW_FILTER_4X,

        /*
         * 快滤波阈值：9 LSB。
         *
         * 快滤波用于在角度快速变化时提升响应速度。
         */
        .fast_filter = AS5600_FAST_FILTER_9_LSB,

        /*
         * 看门狗使能。
         *
         * Watchdog 通常用于低功耗或异常检测相关功能。
         */
        .watchdog = true,
    };

    /*
     * 把配置写入 AS5600。
     */
    esp_err_t ret = as5600_set_conf(as5600, &conf_write);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 用来保存从 AS5600 读回来的配置。
     *
     * 初始化为 0，避免里面有随机值。
     */
    as5600_conf_t conf_read = {0};

    /*
     * 从 AS5600 读取 CONF 配置。
     */
    ret = as5600_get_conf(as5600, &conf_read);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xxx failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * 逐项比较：
     * 写进去的配置 == 读出来的配置。
     *
     * 如果某一项不一致，说明：
     * - 写寄存器失败
     * - 读寄存器失败
     * - 驱动对 bit 位解析有问题
     * - 某些配置值不被芯片接受
     */
    AS5600_CHECK_CONF_FIELD_EQ(power_mode);
    AS5600_CHECK_CONF_FIELD_EQ(hysteresis);
    AS5600_CHECK_CONF_FIELD_EQ(output_stage);
    AS5600_CHECK_CONF_FIELD_EQ(pwm_freq);
    AS5600_CHECK_CONF_FIELD_EQ(slow_filter);
    AS5600_CHECK_CONF_FIELD_EQ(fast_filter);
    AS5600_CHECK_CONF_FIELD_EQ(watchdog);

    /*
     * 打印读回来的配置，方便串口观察。
     */
    printf("AS5600 CONF readback: pm=%d hyst=%d outs=%d pwmf=%d sf=%d fth=%d wd=%d\n",
           (int)conf_read.power_mode,
           (int)conf_read.hysteresis,
           (int)conf_read.output_stage,
           (int)conf_read.pwm_freq,
           (int)conf_read.slow_filter,
           (int)conf_read.fast_filter,
           (int)conf_read.watchdog);

    /*
     * 测试结束后恢复默认配置。
     *
     * const as5600_conf_t conf_default = {0};
     *
     * 结构体所有成员都为 0，
     * 一般对应 AS5600 上电默认配置。
     */
    const as5600_conf_t conf_default = {0};

    /*
     * 写回默认配置，避免测试代码改变芯片状态后影响后续测试。
     */
    // TEST_ASSERT_EQUAL(ESP_OK, as5600_set_conf(as5600, &conf_default));

    ret = as5600_set_conf(as5600, &conf_default);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_del_sensor failed: %s", esp_err_to_name(ret));
    }

    return 0;
}

/* ========================== Unity 测试用例 1：角度读取 ========================== */

/*
 * TEST_CASE 是 Unity 测试框架的测试用例宏。
 *
 * 第一个参数：
 * - 测试用例名称
 *
 * 第二个参数：
 * - 测试标签
 * - 可以通过标签筛选运行哪些测试
 */
// TEST_CASE("AS5600 angle sensor test", "[as5600][iot][sensor]")
void test_angle(void)
{
    /*
     * 初始化 I2C 和 AS5600。
     */
    as5600_test_init();

    /*
     * 执行零点校准测试。
     */
    as5600_test_zero_calibration();

    /*
     * 执行角度读取测试。
     */
    as5600_test_get_angle();

    /*
     * 释放资源。
     */
    // as5600_test_deinit();
}

/* ========================== Unity 测试用例 2：零点校准 ========================== */

// TEST_CASE("AS5600 ZPOS zero calibration", "[as5600][iot][sensor]")
void test_zero_calib(void)
{
    /*
     * 初始化 I2C 和 AS5600。
     */
    as5600_test_init();

    /*
     * 执行零点校准测试。
     */
    as5600_test_zero_calibration();

    /*
     * 释放资源。
     */
    as5600_test_deinit();
}

/* ========================== Unity 测试用例 3：CONF 配置读写 ========================== */

// TEST_CASE("AS5600 CONF write-readback", "[as5600][iot][sensor]")
void test_config_wr(void)
{
    /*
     * 初始化 I2C 和 AS5600。
     */
    as5600_test_init();

    /*
     * 执行 CONF 配置写入和读回测试。
     */
    as5600_test_conf_readback();

    /*
     * 释放资源。
     */
    as5600_test_deinit();
}

/**
 * @brief 读取 电机编码器AS5600 当前角度
 *
 * @param angle_deg 输出角度，范围 0~360
 * @return esp_err_t 0 表示读取成功
 */
esp_err_t motor_encoder_get_angle(float *angle_deg)
{

    if (angle_deg == NULL)
    {
        return 1;
    }

    /*
     * raw 用来保存 AS5600 的 12bit 原始角度值。
     */
    uint16_t raw;

    /*
     * degrees 用来保存换算后的角度值。
     */
    float degrees;

    /*
     * status 用来保存磁铁状态。
     */
    as5600_magnet_status_t status;

    esp_err_t ret;

    // /*
    //  * 读取 AS5600 原始角度值。
    //  *
    //  * 如果这里失败，可能原因：
    //  * - AS5600 没接好
    //  * - I2C 地址不对
    //  * - SDA/SCL 接反
    //  * - 没有上拉电阻
    //  * - AS5600 没供电
    //  */
    // ret = as5600_get_angle_raw(as5600, &raw);

    // /*
    //  * 如果读原始角度失败，说明设备可能没有 ACK。
    //  *
    //  * 这里没有用 TEST_ASSERT，而是直接打印后 return。
    //  * 这样做的效果是：
    //  * - 设备没连接时，不让整个测试硬崩
    //  * - 打印提示方便排查硬件连接
    //  */
    // if (ret != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "as5600_get_angle_raw_failed!\r\n");
    //     return 2;
    // }

    /*
     * 读取角度，并换算成角度单位 degree。
     *
     * 驱动内部一般会做类似换算：
     * degrees = raw * 360.0 / 4096.0
     */
    ret = as5600_get_angle_degrees(as5600, &degrees);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600_not_connected_or_no_ack!\r\n");
        return 3;
    }

    /*
     * 读取磁铁状态。
     *
     * 角度值是否可靠，很大程度取决于磁铁状态是否 OK。
     */
    ret = as5600_get_magnet_status(as5600, &status);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "AS5600_not_connected_or_no_ack!\r\n");
        return 4;
    }

    /*
     * 根据 status 枚举值，从字符串表中取出对应文本。
     *
     * 这里做了边界判断：
     * 如果 status 超过字符串表范围，就显示 unknown，避免数组越界。
     */
    const char *status_str = ((unsigned)status < AS5600_MAGNET_STATUS_STR_COUNT)
                                 ? s_as5600_magnet_status_str[status]
                                 : "unknown";
    if(status!=AS5600_MAGNET_OK)
    {
        ESP_LOGE(TAG, "magnet_status_failed: %s\r\n",
            status_str
        );
        return 5;
    }

    /*
     * 打印当前 AS5600 状态：
     *
     * raw：
     * - 原始角度值
     * - 0 ~ 4095
     *
     * degrees：
     * - 换算后的角度
     * - 0 ~ 360 度
     *
     * magnet status：
     * - 磁铁状态
     */
    // ESP_LOGI(TAG, "AS5600_raw: %u, degrees: %.2f, magnet status: %s\n",
    //          (unsigned)raw,
    //          (float)degrees,
    //          status_str);
    *angle_deg = (float)degrees;

    return 0;
}



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
 *      推荐：
 *          10kHz采样：0.25
 *          15kHz采样：0.32
 *          20kHz采样：0.40
 *
 * @return
 *      当前滤波后的输出值
 */
static inline float current_lpf(float in, float old)
{
    const float alpha = 0.35f;/*0.2~0.4*/

    return old + alpha * (in - old);
}

/**
 *       1KHZ,T = 1ms
 * @brief 根据当前机械角度,计算电机,机械角速度 deg/s
 * 
 * @param now_angle 当前机械角度，单位 deg，范围一般为 0~360
 * @return float 机械角速度，单位 deg/s
 */
float get_motor_w_deg_s_by_angle(float now_angle)
{
    static bool first_flag = true;          /* 第一次进入标志 */
    static float last_angle = 0.0f;         /* 上一次角度，单位 deg */
    static int64_t last_time_us = 0;        /* 上一次时间戳，单位 us */
    
    int64_t now_time_us = esp_timer_get_time();

    /*
     * 第一次进入时没有上一次角度和时间，无法计算速度。
     * 所以只记录当前值，返回 0。
     */
    if (first_flag)
    {
        first_flag = false;
        last_angle = now_angle;
        last_time_us = now_time_us;
        return 0.0f;
    }

    /*
     * 计算本次和上次的时间差，单位 us。
     */
    int64_t dt_us = now_time_us - last_time_us;

    /*
     * us 转成秒。
     */
    float dt_s = (float)dt_us / 1000000.0f;
    // float dt_s = (float)dt_us / 1e6f;

    /*
     * 计算角度变化量。
     *
     * 注意：
     * 不能直接用 now_angle - last_angle 后就结束，
     * 因为角度有 0/360 跳变。
     *
     * 例如：
     * last_angle = 359°
     * now_angle  = 1°
     *
     * 实际是正向转了 2°，
     * 不是反向转了 -358°。
     */
    float delta_deg = now_angle - last_angle;

    /*正转*/
    if (delta_deg > 180)  delta_deg -= 360;
    if (delta_deg < -180) delta_deg += 360;

    /*
    * 原始机械角速度，单位 deg/s。
    */
    float omega_raw = delta_deg / dt_s;


    static float omega_filt = 0.0f;
    omega_filt = current_lpf(omega_raw, omega_filt);

#if 0
    static uint32_t log_cnt = 0;

    if ((log_cnt++)>1)
    {
        log_cnt = 0;
        /*
         * 注意：
         * FOC 控制周期里不要高频 ESP_LOGI。
         * 1ms 打印会严重影响控制实时性。
         * 需要调试时，建议 100ms 打印一次。
         */
        ESP_LOGI(TAG,
                 "last_deg:%.2f, now_deg:%.2f, diff_deg:%.2f, dt_s:%lldus, omega_w:%.3f,omega_filt:%.3f",
                 last_angle,
                 now_angle,
                 delta_deg,
                 (dt_us),
                 omega_raw,
                 omega_filt
        );
    }
    

#endif

    /*
     * 更新历史角度和时间。
     */
    last_angle = now_angle;
    last_time_us = now_time_us;


    return omega_filt;
}



/**
 * @brief 根据机械角速度求转速
 * 
 * w(r/s)=2*pi*n
 * 
 * w(r/min) = 2*pi*n/60
 * 
 * n(转速，rpm)=60*w/2*pi=30*w/pi
 * 
 * @param w_mech 当前机械角速度,deg/s
 * @return float 机械角速度
 */
float get_motor_rpm_by_angle(float w_mech)
{

    float rpm_raw = (w_mech)/ (6.0f);

#if 0
    static uint32_t log_cnt = 0;
    
    if ((log_cnt++)>100)
    {
        log_cnt = 0;
        /*
         * 注意：
         * FOC 控制周期里不要高频 ESP_LOGI。
         * 1ms 打印会严重影响控制实时性。
         * 需要调试时，建议 100ms 打印一次。
         */
        ESP_LOGI(
            TAG,
            "w_mech: %.2f,%.3f\r\n",
            w_mech,
            rpm_raw
        );
    }
#endif
    return rpm_raw;
}

/**
 * @brief 机械角度，电角度，速度采样率1KHZ
 * 
 * @param arg 
 */
static void motor_get_angle_task(void *arg)
{
    float angle = 0.0f;
    float last_angle = 0.0f;
    float m0_mech_rpm = 0.0f;
    float last_m0_mech_rpm = 0.0f;

    static int log_cnt = 0;
    int64_t angle_time_stamp_start = 0; /*时间戳,单位:us*/
    int64_t angle_time_stamp_end = 0; /*时间戳,单位:us*/

    float angle_cont;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // angle_time_stamp_start = esp_timer_get_time();/*角度值时间戳us*/

        if (!motor_encoder_get_angle(&angle))
        {

            /*角度值不能滤波，原本就落后于FOC PWM周期，再滤波更落后 */
            // angle = current_lpf(angle, get_vfoc_theta_m_deg()); 
            /*
             * 设置 VFOC 的机械角度
             */
            angle = current_lpf(angle,last_angle);
            last_angle = angle;

            set_vfoc_theta_m_deg(angle);

            //展开角度
            angle_cont = angle_unwrap_update(
                &unwrap,
                angle
            );

            //PLL估计速度
            pll_update(
                &pll,
                angle_cont
            );

            m0_mech_rpm = (pll.omega/(6.0f));
            m0_mech_rpm = current_lpf(m0_mech_rpm,last_m0_mech_rpm);
            last_m0_mech_rpm = m0_mech_rpm;

            // set_vfoc_mech_w( get_motor_w_deg_s_by_angle(angle) );/*计算设置，机械角速度*/
            // set_vfoc_mech_w( pll.omega );/*计算设置，机械角速度*/
            // ========== 队列发送核心代码 ==========
            // 队列深度10，满了直接丢弃本次转速，不阻塞任务
            // xQueueSend(g_motor0_mech_rpm_queue, &m0_mech_rpm, 0);
            // 邮箱：覆盖写入，永远写最新值，不会阻塞，timeout写0
            xQueueOverwrite( g_motor0_mech_rpm_mailbox, &m0_mech_rpm );

            xQueueOverwrite( g_motor0_mech_deg_mailbox, &angle );
            // xQueueSend(g_motor0_mech_deg_queue, &angle, 0);/*发送机械角度值*/
            // if(xQueueSend(g_motor0_mech_rpm_queue, &m0_mech_rpm, 0) != pdPASS)
            // {
            //     // 队列已满，数据丢弃，可打印提示（调试用）
            //     ESP_LOGW(
            //         TAG,
            //         "g_motor0_mech_rpm_queue send fialed!,remi:%d,use:%d\r\n",
            //         uxQueueSpacesAvailable(g_motor0_mech_rpm_queue),
            //         uxQueueMessagesWaiting(g_motor0_mech_rpm_queue)
            //     );
            // }

            // set_vfoc_mech_rpm( get_motor_rpm_by_angle(get_vfoc_mech_w()) );/*计算转速*/

            // calc_vfoc_theta_e_w(&vfoc_m0_dt);/*计算 电角度转速*/
            
            // angle_time_stamp = esp_timer_get_time();/*加入时间戳*/
            
            // /*获取机械转速*/
            // rpm = get_vfoc_mech_rpm();

            #if 0
                /*
                * 低频打印，比如 100 次打印一次
                * 如果你的任务实际 10ms 一次，
                * 那么 100 次就是大约 1 秒打印一次
                */
                if (++log_cnt >= 100)
                {
                    log_cnt = 0;

                    ESP_LOGI(TAG,
                    //     "motor_angle: %.2f\r\n",
                    //     m0_mech_rpm
                    // );
                        "raw %.2f cont %.2f theta %.2f omega %.2f\n",
                        angle,
                        angle_cont,
                        pll.theta,
                        pll.omega
                    );
                    // ESP_LOGI(TAG,
                    //     "motor_angle = %.2f deg, theta_m_deg = %.2f, rpm = %.2f,task_T:%lldus,w_mech:%.2f,w_e%.2f\r\n",
                    //     angle,
                    //     get_vfoc_theta_m_deg(),
                    //     rpm,
                    //     (angle_time_stamp_end - angle_time_stamp_start),
                    //     get_vfoc_mech_w(),
                    //     get_vfoc_theta_e_w(&vfoc_m0_dt)

                    // );


                }
            #endif
        }
        else
        {
            ESP_LOGE(TAG, "motor_encoder_get_angle_failed!");
        }

        // /*计算读取，计算一下角度速度值，耗时时间*/
        // angle_time_stamp_end = esp_timer_get_time();
        
        // ESP_LOGI(TAG,
        //     "%lld\r\n",
        //     (angle_time_stamp_end - angle_time_stamp_start)

        // );
    }
}

/**
 * @brief 电机编码器AS5600零点位置校准
 *
 * @return uint8_t 0:OK, other:failed!
 */
uint8_t motor_encoder_zero_point_calib(void)
{
    esp_err_t ret;
    as5600_magnet_status_t status;

    /*
     * 先读取磁铁状态。
     *
     * 零点校准必须在磁铁状态正常时才有意义。
     */
    ret = as5600_get_magnet_status(as5600, &status);

    /*
     * 如果磁铁状态读取失败，说明 I2C 通信或者设备状态异常。
     * 这里直接跳过零点校准测试。
     */
    if (ret != ESP_OK)
    {
        printf("AS5600 magnet status read failed, skip zero calibration check\n");
        return 1;
    }

    /*
     * 如果磁铁状态不是 OK，也跳过测试。
     *
     * 因为磁场太弱、太强、没检测到磁铁时，
     * 读出来的角度不可靠，做零点校准没有意义。
     */
    if (status != AS5600_MAGNET_OK)
    {
        printf("AS5600 magnet not OK (status=%d), skip zero calibration check\n", (int)status);
        return 2;
    }

    /*
     * degrees_before 保存校准前角度。
     */
    float degrees_before;

    /*
     * 读取校准前的角度。
     */
    ret = as5600_get_angle_degrees(as5600, &degrees_before);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_get_angle_degrees_failed!\r\n");
        return 3;
    }

    /*
     * 打印校准前角度，方便观察校准效果。
     */
    printf("AS5600_before_ZPOS_cal: degrees=%.2f\n", (double)degrees_before);

    /*
     * 设置当前位置为零点。
     *
     * 注意：
     * 这个函数具体是写 AS5600 的 ZPOS 寄存器，
     * 还是只在驱动内部做软件零点偏移，
     * 要看 as5600_set_zero_position() 的驱动实现。
     */
    ret = as5600_set_zero_position(as5600);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_set_zero_position_Failed!\r\n");
        return 4;
    }

    /*
     * 设置零点后等待一小段时间。
     *
     * 原因：
     * AS5600 内部可能有滤波器，
     * 写入 ZPOS 后 ANGLE 输出不一定立刻稳定。
     *
     * 这里延时 20ms，让滤波后的角度输出稳定一下。
     */
    vTaskDelay(pdMS_TO_TICKS(20));

    /*
     * degrees 保存校准后的角度。
     */
    float degrees;

    /*
     * 读取校准后的角度。
     */
    ret = as5600_get_angle_degrees(as5600, &degrees);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "as5600_get_angle_degrees_Failed!\r\n");
        return 5;
    }

    /*
     * 打印校准后的角度。
     *
     * 期望值：
     * - 接近 0 度
     * - 或者接近 360 度
     */
    printf("AS5600_after_ZPOS_cal: degrees=%.2f (expect ~0 or ~360 wrap)\n",
           (double)degrees);

    /*
     * 判断校准后的角度是否接近 0 度或者接近 360 度。
     *
     * 如果不满足，测试失败，并打印错误信息。
     */
    if (!as5600_degrees_near_zero_or_wrap_360(degrees, AS5600_ZERO_CAL_TOL_DEG))
    {
        ESP_LOGE(TAG, "angle_after_zero_calibration_failed, degrees=%.2f", (double)degrees);
        return 6;
    }

    return 0;
}

/**
 * @brief 电机编码器AS5600初始化
 *
 * @return uint8_t 0:OK, other failed!
 */
uint8_t as5600_init(void)
{
    esp_err_t ret;

    /*
     * I2C Master 总线配置结构体。
     *
     * 这里配置的是 ESP32 作为 I2C 主机：
     * - 使用哪个 I2C 控制器
     * - SDA 引脚
     * - SCL 引脚
     * - 时钟源
     * - 毛刺过滤
     * - 是否开启内部上拉
     */
    i2c_master_bus_config_t bus_config = {
        /*
         * 使用 I2C_NUM_0 控制器。
         */
        .i2c_port = I2C_MASTER_NUM,

        /*
         * SDA 数据线 GPIO。
         */
        .sda_io_num = I2C_MASTER_SDA_IO,

        /*
         * SCL 时钟线 GPIO。
         */
        .scl_io_num = I2C_MASTER_SCL_IO,

        /*
         * I2C 时钟源。
         *
         * I2C_CLK_SRC_DEFAULT 表示使用 ESP-IDF 默认推荐的时钟源。
         */
        .clk_source = I2C_CLK_SRC_DEFAULT,

        /*
         * 毛刺过滤参数。
         *
         * I2C 信号线上可能会有很短的干扰毛刺。
         * glitch_ignore_cnt = 7 表示过滤掉非常短的脉冲干扰。
         */
        .glitch_ignore_cnt = 7,

        /*
         * 使能 ESP32 内部上拉电阻。
         *
         * I2C 总线是开漏结构，SDA/SCL 必须有上拉。
         *
         * 注意：
         * 内部上拉通常比较弱，正式硬件建议外接 4.7k 左右上拉电阻。
         */
        .flags.enable_internal_pullup = true,
    };

    /*
     * 创建 I2C Master 总线。
     *
     * 成功后，bus_handle 会被赋值。
     */
    ret = i2c_new_master_bus(&bus_config, &bus_handle);

    /*
     * Unity 断言：
     * 判断 i2c_new_master_bus() 的返回值是否等于 ESP_OK。
     *
     * 如果不等于 ESP_OK，测试直接失败。
     */
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_master_bus_failed!\r\n");
        return 1;
    }

    /*
     * AS5600 I2C 设备配置。
     *
     * 这里只配置了 I2C 速度。
     * 设备地址通常在驱动里固定为 AS5600 默认地址 0x36。
     */
    as5600_i2c_config_t i2c_conf = {
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };

    /*
     * 在 I2C bus 上创建 AS5600 传感器对象。
     *
     * 参数说明：
     * - bus_handle：前面创建好的 I2C 总线
     * - &i2c_conf：AS5600 的 I2C 配置
     * - &as5600：输出参数，用来保存 AS5600 设备句柄
     */
    ret = as5600_new_sensor(bus_handle, &i2c_conf, &as5600);

    /*
     * 检查 AS5600 设备创建是否成功。
     */
    // TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_ERROR_CHECK(ret);

    /*
     * 检查 as5600 句柄不为空。
     * 如果为空，说明 AS5600 设备对象没有创建成功。
     */
    if (as5600 == NULL)
    {
        ESP_LOGE(TAG, "as5600 handle is NULL");
        return 2;
    }

    return 0;
}



int64_t get_motor_angle_time_stamp(void)
{
    return angle_time_stamp;
}


/**
 * @brief 电机编码器AS5600初始化
 *
 * @return uint8_t 0:OK, other failed!
 */
void motor_encoder_init(void)
{
    float l_temp_angle = 0.0f;
    if (as5600_init())
    {
        ESP_LOGE(TAG, "as5600_init_failed!\r\n");
    }

    #if 0
        /*零角度位置初始化校准，不能每次上电后都运行这个*/
        if (motor_encoder_zero_point_calib())
        {
            ESP_LOGE(TAG, "motor_encoder_zero_point_calib_failed!\r\n");
        }
    #endif

    motor_encoder_get_angle(&l_temp_angle);
    ESP_LOGE(TAG, "motor_encoder_angle_deg:%.2f\r\n",l_temp_angle);

    angle_unwrap_init(&unwrap,0.0f);
    pll_init(&pll,l_temp_angle);

    xTaskCreatePinnedToCore(
        motor_get_angle_task, // 任务函数
        "encoder_angle_task",     // 任务名
        MOTOR_GET_ANGLE_TASK_STACK, // 栈大小
        NULL,                     // 参数
        MO_GET_ANGLE_TASK_PRIO,                        // 优先级
        &motor_get_angle_task_handle,                     // 任务句柄
        MOTOR_GET_ANGLE_TASK_CORE   // 跑在 core 1
    );
}
