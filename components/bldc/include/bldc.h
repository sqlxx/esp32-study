#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化开环无刷（MCPWM group 1，默认 GPIO25/26/32） */
esp_err_t bldc_init(void);

/** 请求打开驱动输出。EN 在下一个控制周期置位；转速为 0 时只使能，不转。 */
esp_err_t bldc_enable(void);

/** 请求关断。下一个控制周期把三相占空比置 0 并拉低 EN。 */
void bldc_disable(void);

bool bldc_is_enabled(void);

typedef enum {
    BLDC_MOTION_OPENLOOP = 0, /* 电气角 = ∫ 转速，uq = 调制度 */
    BLDC_MOTION_POSITION,     /* 位置 PI → 转速，速度 PI → uq */
    BLDC_MOTION_VELOCITY,     /* 编码器电气角，速度 PI → uq */
} bldc_motion_t;

/** 开环机械转速，单位 rpm；负值为反转。建议先从 ±30 试起，上限 ±300。 */
esp_err_t bldc_set_openloop_rpm(float rpm);

float bldc_get_openloop_rpm(void);

/** 切到 POSITION 或 VELOCITY 时需要 AS5600。切入 POSITION 时把目标锁在当前机械角。 */
esp_err_t bldc_set_motion(bldc_motion_t motion);

bldc_motion_t bldc_get_motion(void);

/** 位置目标，单位度，与 AS5600 角度同一参考。 */
esp_err_t bldc_set_position_deg(float deg);

float bldc_get_position_deg(void);

/**
 * 调制度 0~1。开环时就是 uq。速度环和位置环里，它是 |uq| 上限，
 * 用来限制堵住时的电流，最高 0.95。
 * 默认 0.15，云台电机先保持较小值，避免过流发热。
 */
esp_err_t bldc_set_modulation(float modulation);

float bldc_get_modulation(void);

/**
 * 速度 PI 与转速低通时间常数（秒）。位置环的内环用同一套。
 * kp、ki 取 0~1000，tau 取 0~10。ki 的数值变化时清掉积分。
 */
esp_err_t bldc_set_velocity_kp(float kp);
esp_err_t bldc_set_velocity_ki(float ki);
esp_err_t bldc_set_velocity_tau(float tau_s);

/** 位置环外层 PI。kp、ki 取 0~1000。 */
esp_err_t bldc_set_position_kp(float kp);
esp_err_t bldc_set_position_ki(float ki);

typedef struct {
    bool enabled;
    float rpm;
    float modulation;
    float theta_rad;
    float u;
    float v;
    float w;
} bldc_status_t;

void bldc_get_status(bldc_status_t *out);

#ifdef __cplusplus
}
#endif
