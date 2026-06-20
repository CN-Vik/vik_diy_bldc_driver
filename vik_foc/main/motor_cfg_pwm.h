#ifndef MOTOR_CFG_PWM_H
#define MOTOR_CFG_PWM_H

#include "esp_err.h"

int64_t get_motor_pwm_isr_time_stamp(void);
esp_err_t esp32_mcpwm_init(void);
esp_err_t motor_set_pwm_duty(float duty_u, float duty_v, float duty_w);

#endif
