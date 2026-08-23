#ifndef FOC_TASK_H
#define FOC_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*任务运行时间统计*/
#define TASK_RUNTIME_STATIS     0

// 头文件里声明全局变量，所有包含此头文件的文件都能访问同一个变量
extern TaskHandle_t foc_task_handle;

float vfoc_calibrate_m0_offset(void);
void foc_task(void *arg);


#endif 