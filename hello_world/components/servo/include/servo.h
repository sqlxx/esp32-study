#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 SG90（GPIO18，MCPWM 50Hz） */
esp_err_t servo_init(void);

/** 设置角度，范围 0~180（会停止扫描） */
esp_err_t servo_set_angle(int angle_deg);

/** 当前角度 */
int servo_get_angle(void);

/** 本地往返扫描 0°↔180° */
esp_err_t servo_scan_start(void);
void servo_scan_stop(void);
bool servo_is_scanning(void);

/**
 * 扫描速度档位 1(慢)~10(快)。
 * 加速靠加大步距，并保持足够间隔，避免过密更新导致原地抖动。
 */
esp_err_t servo_set_scan_speed(int level);
int servo_get_scan_speed(void);

#ifdef __cplusplus
}
#endif
