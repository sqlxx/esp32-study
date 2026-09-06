#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bldc_pwm_init(void);

/** 三相占空比，范围 0~1 */
esp_err_t bldc_pwm_set_duty(float u, float v, float w);

/** 驱动芯片 EN；没有接 EN 脚时只记录状态 */
esp_err_t bldc_pwm_set_enable(bool on);

#ifdef __cplusplus
}
#endif
