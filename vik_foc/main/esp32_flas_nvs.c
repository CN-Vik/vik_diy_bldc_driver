/**
 * @file esp32_flas_nvs.c
 * @author vik (ufo281@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2026-07-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "esp32_flas_nvs.h"

DevConfig_t balance_vehicle_car = {0};


// 保存结构体到NVS
esp_err_t cfg_saveto_flash(const DevConfig_t *dev)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ESP32_FLASH_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    // 直接把结构体内存整块写入Blob
    err = nvs_set_blob(handle, ESP32_FLASH_NVS_KEY_CFG, dev, sizeof(DevConfig_t));
    if (err == ESP_OK)
    {
        err = nvs_commit(handle); // 提交到flash，必不可少
    }
    nvs_close(handle);
    return err;
}

// 从NVS读取结构体
esp_err_t cfg_readfrom_flash(DevConfig_t *dev)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ESP32_FLASH_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
        return err;

    size_t blob_len = 0;
    // 第一步：查询当前blob长度
    err = nvs_get_blob(handle, ESP32_FLASH_NVS_KEY_CFG, NULL, &blob_len);

    // 判断：key不存在，直接返回，上层可以初始化默认参数
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return ESP_ERR_NVS_NOT_FOUND;
    }
    if (err != ESP_OK)
    {
        nvs_close(handle);
        return err;
    }

    // 长度校验，防止旧版本结构体大小不一致导致越界
    if (blob_len != sizeof(DevConfig_t))
    {
        nvs_close(handle);
        return ESP_ERR_INVALID_SIZE;
    }

    // 正式读取到结构体
    err = nvs_get_blob(handle, ESP32_FLASH_NVS_KEY_CFG, dev, &blob_len);
    nvs_close(handle);
    return err;
}


// 删除保存的结构体（只删自己的参数，WiFi信息保留）
esp_err_t flash_cfg_data_erase(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ESP32_FLASH_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    nvs_erase_key(handle, ESP32_FLASH_NVS_KEY_CFG);
    err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

void flash_nvs_app_main(void)
{
    // NVS初始化标准代码
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // 分区异常，先擦除整个nvs分区
        ESP_ERROR_CHECK(nvs_flash_erase());
        // ⭐擦除完成【必须重新初始化】
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // flash_cfg_data_erase();


    // 填充默认参数
    // balance_vehicle_car.kp = 0.0082f;
    // balance_vehicle_car.ki = 0.1f;
    // balance_vehicle_car.kd = 0.05f;
    // balance_vehicle_car.freq = 10000;
    // balance_vehicle_car.m0_mech_ofset = 346.3f; 
    // balance_vehicle_car.enable = 1;
    // snprintf(balance_vehicle_car.dev_name, sizeof(balance_vehicle_car.dev_name), "ESP32_DEVICE");

    // // 保存默认配置
    // cfg_saveto_flash(&balance_vehicle_car);

    // 尝试加载参数
    err = cfg_readfrom_flash(&balance_vehicle_car);
    if (err != ESP_OK)
    {
        printf("read flash NVS failed! \r\n");
    }
    else
    {
        printf("读取成功!8 kp = %.2f \r\n", balance_vehicle_car.m0_mech_ofset);
    }

    printf("7kp = %.3f,\r\n",

        balance_vehicle_car.m0_mech_ofset

    );

}



