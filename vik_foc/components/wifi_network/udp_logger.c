/**
 * @file udp_logger.c
 * @author vik (ufo281@outlook.com)
 * @brief 基于UDP的纯净LOG/数据打印
 * @version 0.2
 * @date 2026-09-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "udp_logger.h"

udp_log_level_t current_udp_log_level = UDP_LEVEL_INFO; // 默认过滤等级

void udp_log_print(udp_log_level_t level, const char *fmt, ...)
{
    // 1. 等级过滤
    if (level > current_udp_log_level || udp_log_queue == NULL) {
        return;
    }

    char *buf = calloc(1, UDP_LOG_MAX_LEN);
    if (buf == NULL) return; // 内存不足直接丢弃

    // 2. 直接处理可变参数，写入纯净数据
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, UDP_LOG_MAX_LEN, fmt, args);
    va_end(args);

    // 3. 发送到队列 (不要阻塞，保证 FOC/PID 核心算法不被卡死)
    if (xQueueSend(udp_log_queue, &buf, 0) != pdTRUE) {
        free(buf); // 队列满，没人接收，丢弃并释放内存
    }
}