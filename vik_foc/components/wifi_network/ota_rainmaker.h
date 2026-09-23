#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 RainMaker OTA 模块
 */
esp_err_t ota_rainmaker_init(void);

#ifdef __cplusplus
}
#endif