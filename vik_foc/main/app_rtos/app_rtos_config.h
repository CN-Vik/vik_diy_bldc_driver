/**
 * @file app_rtos_config.h
 * @author vik (ufo281@outlook.com)
 * @brief 进行统一管理RTOS任务分配、栈大小及核心绑定
 * @version 0.2
 * @date 2026-06-13
 */
#ifndef APP_RTOS_CONFIG_H
#define APP_RTOS_CONFIG_H

/* ================================================================
 * 1. 运行核心定义 (SMP Core Affinity)
 * ================================================================ */
#define APP_TASK_CORE_0             0   /* Core 0: 系统通信核 (Wi-Fi/lwIP/TCP/UDP/OTA) */
#define APP_TASK_CORE_1             1   /* Core 1: 电机运动核 (FOC/ADC/编码器，完全隔绝网络打扰) */

/* 原有宏保持完全一致：统一将电机相关任务收拢到 Core 1 */
#define FOC_TASK_RUN_CORE           APP_TASK_CORE_1 
#define SIX_STEP_RUN_CORE           APP_TASK_CORE_1 
#define MOTOR_CURRENT_TASK_CORE     APP_TASK_CORE_1 
#define MOTOR_GET_ANGLE_TASK_CORE   APP_TASK_CORE_1 

/* 新增网络通信任务核心分配：全部绑定在 Core 0 */
#define WIFI_SMART_CFG_TASK_CORE    APP_TASK_CORE_0
#define TCP_SERVER_TASK_CORE        APP_TASK_CORE_0
#define UDP_CLIENT_TASK_CORE        APP_TASK_CORE_0
#define OTA_HTTPS_TASK_CORE         APP_TASK_CORE_0


/* ================================================================
 * 2. 任务优先级统一管理 (0 ~ 24，数字越大优先级越高)
 * ================================================================ */
#define APP_TASK_PRIO_LOW           3
#define APP_TASK_PRIO_NORMAL        5
#define APP_TASK_PRIO_HIGH          8
#define APP_TASK_PRIO_REALTIME      10

/* 
 * Core 1 (电机控制) 优先级体系：
 * - FOC 必须是最高级别 (22)，到达 100us 周期时能立即打断正在读 I2C 的角度任务抢先执行；
 * - ADC 与角度任务在后台尽可能快地更新全局变量。
 */
#define FOC_TASK_PRIO               22  /* 核心脉搏，最高抢占权 */
#define SIX_STEP_PRIO               22  /* 六步换相与 FOC 二选一运行 */
#define MOTOR_CURRENT_ADC_TASK_PRIO 21  /* ADC 读取任务 */
#define MO_GET_ANGLE_TASK_PRIO      20  /* I2C 读取耗时长，设为20允许被 FOC 打断 */

/* 
 * Core 0 (通信与系统) 优先级体系：
 * - 避开 ESP-IDF 底层 Wi-Fi Driver 的 23；
 * - UDP 遥测 (VOFA+) 优先级高于控制 TCP 与 OTA。
 */
#define UDP_CLIENT_TASK_PRIO        14  /* 高频遥测推流 */
#define TCP_SERVER_TASK_PRIO        12  /* 指令接收响应 */
#define OTA_HTTPS_TASK_PRIO         8   /* 后台升级下载，适中优先级 */
#define WIFI_SMART_CFG_TASK_PRIO    APP_TASK_PRIO_LOW /* 仅配网时工作，配网完成即销毁 */


/* ================================================================
 * 3. 任务栈大小 (单位: Byte)
 * ================================================================ */
/* 原有电机任务栈 */
#define MOTOR_CURRENT_TASK_STACK    (4 * 1024)  /* 4KB 足够 ADC 运算 */
#define MOTOR_GET_ANGLE_TASK_STACK  (4 * 1024)  /* 4KB 足够 I2C 驱动执行 */
#define FOC_TASK_STACK              (8 * 1024)  /* 8KB 对矩阵/PID浮点计算极为充裕 */
#define SIX_STEP_TASK_STACK         (8 * 1024)

/* 新增通信任务栈 */
#define WIFI_SMART_CFG_TASK_STACK   (4 * 1024)  /* SmartConfig 需维护少量状态 */
#define TCP_SERVER_TASK_STACK       (4 * 1024)  /* TCP 通信栈 */
#define UDP_CLIENT_TASK_STACK       (4 * 1024)  /* UDP 发包栈 */
#define OTA_HTTPS_TASK_STACK        (8 * 1024)  /* HTTPS/HTTP 客户端与解包必须给 8KB */

#endif /* APP_RTOS_CONFIG_H */