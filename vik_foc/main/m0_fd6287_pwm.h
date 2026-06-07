#ifndef M0_FD6287_PWM_H
#define M0_FD6287_PWM_H

#include "esp_err.h"

esp_err_t m0_fd6287_mcpwm_init(void);
esp_err_t m0_fd6287_set_duty(float duty_u, float duty_v, float duty_w);
esp_err_t m0_fd6287_foc_start(void);

#endif