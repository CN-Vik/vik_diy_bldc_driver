/**
 * @file motor_6step.c
 * @brief 有感六步换向无刷电机驱动（AS5600 + 单IO FD6287）
 */
#include "motor_6step.h"
#include "esp_log.h"
#include "math.h"

/* 你已有的驱动头文件，根据实际路径调整 */
#include "motor_cfg_pwm.h"
#include "motor_angle_acqu.h"
#include "app_rtos_resource.h"

static const char *TAG = "vik_six_step";

TaskHandle_t six_step_task_handle = NULL;

/************************ 内部静态变量 ************************/
static float s_zero_mech_angle = 0.0f;   // 电角度零位对应的机械角度（校准后写入）
static float s_target_duty = 0.0f;       // 当前目标占空比 0~100

/************************ 六步换向表（顺时针） ************************/
/* 顺序：扇区0~5，对应U/V/W三相状态 */
static const comm_table_t six_step_comm_table[6] =
{
    {PHASE_PWM, PHASE_OFF, PHASE_LOW},   // 扇区0：U上PWM，W下通，V悬空
    {PHASE_PWM, PHASE_LOW, PHASE_OFF},   // 扇区1：U上PWM，V下通，W悬空
    {PHASE_OFF, PHASE_LOW, PHASE_PWM},   // 扇区2：W上PWM，V下通，U悬空
    {PHASE_LOW, PHASE_OFF, PHASE_PWM},   // 扇区3：W上PWM，U下通，V悬空
    {PHASE_LOW, PHASE_PWM, PHASE_OFF},   // 扇区4：V上PWM，U下通，W悬空
    {PHASE_OFF, PHASE_PWM, PHASE_LOW},   // 扇区5：V上PWM，W下通，U悬空
};

/************************ 内部函数 ************************/
/**
 * @brief 电角度转扇区 0~5
 */
static int get_sector(float elec_angle)
{
    float angle = elec_angle;
    /* 归一化到0~360° */
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;
    
    return (int)(angle / 60.0f);
}

/**
 * @brief 根据扇区和目标占空比，更新三相PWM输出
 * @note  核心修复：状态枚举 → 真实占空比的映射
 */
static void update_commutation(uint8_t sector, float target_duty)
{
    float duty_u = 0.0f;
    float duty_v = 0.0f;
    float duty_w = 0.0f;
    const float half_duty = 50.0f; // 悬空相固定50%占空比，等效零电流

    comm_table_t p = six_step_comm_table[sector % 6];

    /* U相状态映射 */
    switch (p.U)
    {
        case PHASE_PWM: duty_u = target_duty; break;
        case PHASE_LOW: duty_u = 0.0f;       break;
        case PHASE_OFF: duty_u = half_duty;  break;
        default:        duty_u = half_duty;  break;
    }

    /* V相状态映射 */
    switch (p.V)
    {
        case PHASE_PWM: duty_v = target_duty; break;
        case PHASE_LOW: duty_v = 0.0f;       break;
        case PHASE_OFF: duty_v = half_duty;  break;
        default:        duty_v = half_duty;  break;
    }

    /* W相状态映射 */
    switch (p.W)
    {
        case PHASE_PWM: duty_w = target_duty; break;
        case PHASE_LOW: duty_w = 0.0f;       break;
        case PHASE_OFF: duty_w = half_duty;  break;
        default:        duty_w = half_duty;  break;
    }

    /* 调用底层PWM驱动，设置三相占空比（0~100%） */
    motor_set_pwm_duty(duty_u, duty_v, duty_w);
}

/************************ 对外接口 ************************/
void six_step_init(float init_duty)
{
    s_target_duty = init_duty;
    s_zero_mech_angle = 0.0f;
    ESP_LOGI(TAG, "six step init, duty:%.1f%%", init_duty);
}

void six_step_set_duty(float duty)
{
    if (duty < 0.0f)   duty = 0.0f;
    if (duty > 100.0f) duty = 100.0f;
    s_target_duty = duty;
}

void six_step_calibrate_zero(void)
{
    ESP_LOGI(TAG, "start calibrate zero position...");
    
    /* 强制输出扇区0，10%小占空比，让转子锁定到固定位置 */
    update_commutation(0, 10.0f);
    vTaskDelay(pdMS_TO_TICKS(500)); // 等待转子稳定

    /* 读取当前机械角度，保存为零位 */
    float mech_angle = 0.0f;
    xQueuePeek(g_motor0_mech_deg_mailbox, &mech_angle, portMAX_DELAY);
    s_zero_mech_angle = mech_angle;

    /* 校准完成，关闭输出（可选，也可以直接保持） */
    update_commutation(0, 0.0f);
    ESP_LOGI(TAG, "zero calibrate done, zero mech angle:%.2f°", s_zero_mech_angle);
}

/************************ 主任务 ************************/
void six_step_task(void *arg)
{
    float mech_angle = 0.0f;
    float elec_angle = 0.0f;
    uint8_t sector = 0;
    uint32_t cnt = 0;

    while (1)
    {
        /* 读取机械角度（从你的角度采集队列获取） */
        if (xQueueReceive(g_motor0_mech_deg_mailbox, &mech_angle, 5) != pdPASS)
        {
            ESP_LOGW(TAG, "mech angle queue receive failed");
            continue;
        }

        /* 1. 减去零位偏移，得到真实机械角度 */
        float mech_offset = mech_angle - s_zero_mech_angle;
        if (mech_offset < 0.0f) mech_offset += 360.0f;

        /* 2. 机械角度 → 电角度：机械角度 × 极对数 */
        elec_angle = mech_offset * MOTOR_POLR;

        /* 3. 电角度 → 扇区 */
        sector = get_sector(elec_angle);

        /* 4. 更新换向输出 */
        update_commutation(sector, s_target_duty);

        /* 调试日志，每100ms打印一次 */
        if ((cnt++) % 10 == 0)
        {
            ESP_LOGI(TAG, "mech:%.2f° | elec:%.2f° | sector:%d | duty:%.1f%%",
                     mech_angle, elec_angle, sector, s_target_duty);
        }

        /* 10ms执行一次换向，低速足够；高速场景可减小到1~5ms */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
