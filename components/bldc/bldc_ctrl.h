#pragma once

#include "bldc.h"
#include "esp_err.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BLDC_TORQUE_VOLTAGE = 0, /* 速度 PI 的输出当作 uq */
    BLDC_TORQUE_CURRENT,     /* 速度 PI 的输出当作 iq_ref；尚未打开 */
} bldc_torque_t;

void bldc_ctrl_init(void);

/** 只改运行请求。占空比和 EN 由 bldc_step() 写。 */
void bldc_ctrl_set_enabled(bool on);

bool bldc_ctrl_is_enabled(void);

esp_err_t bldc_ctrl_set_rpm(float rpm);
float bldc_ctrl_get_rpm(void);

esp_err_t bldc_ctrl_set_modulation(float modulation);
float bldc_ctrl_get_modulation(void);

/** 速度 PI 与转速低通。位置环的内环用同一套。kp、ki 为 0~1000，tau 为 0~10 秒。ki 变化时清积分。 */
esp_err_t bldc_ctrl_set_velocity_kp(float kp);
esp_err_t bldc_ctrl_set_velocity_ki(float ki);
esp_err_t bldc_ctrl_set_velocity_tau(float tau_s);

/** 位置环外层 PI。产出的转速交给速度环。 */
esp_err_t bldc_ctrl_set_position_kp(float kp);
esp_err_t bldc_ctrl_set_position_ki(float ki);

esp_err_t bldc_ctrl_set_motion(bldc_motion_t motion);
bldc_motion_t bldc_ctrl_get_motion(void);

esp_err_t bldc_ctrl_set_position_deg(float deg);
float bldc_ctrl_get_position_deg(void);

int bldc_ctrl_pole_pairs(void);

/** 编码器机械角（弧度，尚未乘方向）→ 电气角。方向和零位在第一次闭环使能时测好，只乘这一次。 */
float bldc_ctrl_mech_to_elec(float mech_rad);

typedef struct {
    bldc_motion_t motion;
    bldc_torque_t torque;
    float rpm;
    float modulation;
    float theta_rad;
    float ud;
    float uq;
    float u;
    float v;
    float w;
    float pos_deg;
    float mech_deg;
} bldc_ctrl_status_t;

void bldc_ctrl_get_status(bldc_ctrl_status_t *out);

/**
 * 跑一个控制周期。关断请求在本函数里生效：清电气角、三相占空比置 0、拉低 EN。
 * 开环：theta_e 由机械转速积分，ud = 0，转速非 0 时 uq = modulation。
 * 位置 / 速度：第一次使能先对电气零位。之后 theta_e 由编码器计算。
 * 位置环和速度环共用速度 PI：反馈是低通后的 AS5600 转速。
 * 位置环先把角度误差变成目标转速。
 * |uq| 不超过调制度，最高 0.95。
 */
void bldc_step(float dt);

#ifdef __cplusplus
}
#endif
