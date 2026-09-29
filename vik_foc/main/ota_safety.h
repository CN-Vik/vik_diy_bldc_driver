#ifndef OTA_SAFETY_H
#define OTA_SAFETY_H

/**
 * @brief OTA 升级前的电机安全停机
 *
 * 在开始下载固件前调用，完成三件事：
 *   1. 关闭 MOS 电源（硬件断开，电机立即失电）
 *   2. 挂起 FOC 任务（停止最高优先级抢占，释放 CPU）
 *   3. PWM 三相输出零电压矢量
 *
 * 原因：OTA 写 flash 时 flash cache 会关闭，且 FOC 任务(优先级22)
 * 会抢占 CPU 并占用堆，导致 OTA 分配内存失败(ESP_ERR_NO_MEM)。
 */
void app_ota_prepare_shutdown(void);

#endif /* OTA_SAFETY_H */
