#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 反 Park 后做正弦 PWM，并写入 MCPWM。
 *
 * ud、uq 是相电压峰值，单位与母线一半相比：1 表示占空比从 0.5 摆到 0 或 1。
 * theta_e 是 d 轴相对 U 相轴线的电气角，弧度。
 * uq > 0 且 theta_e 增加时，相序为 U → V → W。
 * duty_uvw 可传 NULL；非 NULL 时依次写入 U、V、W 占空比（限幅前）。
 */
esp_err_t bldc_modulate(float ud, float uq, float theta_e, float duty_uvw[3]);

#ifdef __cplusplus
}
#endif
