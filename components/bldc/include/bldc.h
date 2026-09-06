#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化开环无刷（MCPWM group 1，默认 GPIO25/26/32） */
esp_err_t bldc_init(void);

/** 打开驱动输出（EN，若有）。转速为 0 时只是使能，不转。 */
esp_err_t bldc_enable(void);

/** 关断输出，三相占空比置 0 */
void bldc_disable(void);

bool bldc_is_enabled(void);

/** 开环机械转速，单位 rpm；负值为反转。建议先从 ±30 试起。 */
esp_err_t bldc_set_openloop_rpm(float rpm);

float bldc_get_openloop_rpm(void);

/**
 * 调制度 0~1，等效于开环电压比例。
 * 默认 0.15，云台电机先保持较小值，避免过流发热。
 */
esp_err_t bldc_set_modulation(float modulation);

float bldc_get_modulation(void);

#ifdef __cplusplus
}
#endif
