/**
 * @file udp_logger.h
 * @author vik (ufo281@outlook.com)
 * @brief 基于UDP的纯净LOG/数据打印
 * @version 0.2
 * @date 2026-09-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef UDP_LOGGER_H
#define UDP_LOGGER_H

#include <stdarg.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_rtos_resource.h"

// 打印等级定义
typedef enum {
    UDP_LEVEL_ERROR = 0,
    UDP_LEVEL_WARN,
    UDP_LEVEL_INFO,
    UDP_LEVEL_DEBUG
} udp_log_level_t;

// 当前全局过滤等级
extern udp_log_level_t current_udp_log_level;

#define UDP_LOG_MAX_LEN 128


void udp_log_print(udp_log_level_t level, const char *fmt, ...);

// 简化后的宏，直接传格式化字符串即可
#define UDP_LOGE(fmt, ...) udp_log_print(UDP_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define UDP_LOGW(fmt, ...) udp_log_print(UDP_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define UDP_LOGI(fmt, ...) udp_log_print(UDP_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define UDP_LOGD(fmt, ...) udp_log_print(UDP_LEVEL_DEBUG, fmt, ##__VA_ARGS__)

#endif // _UDP_LOGGER_H_