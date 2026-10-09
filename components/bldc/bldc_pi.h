#pragma once

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} bldc_pi_t;

void bldc_pi_reset(bldc_pi_t *pi);

/**
 * 输出限在 [out_min, out_max]。积分项单独限在同一区间，
 * 所以误差回到 0 时，输出仍能停在稳住工况所需的值，而不是只剩比例项。
 */
float bldc_pi_step(bldc_pi_t *pi, float err, float dt);

/**
 * 比例项和积分项各用一份误差，其余同 bldc_pi_step。
 * 输出已经顶到限幅、积分还往同一边走时，积分不再累加。
 */
float bldc_pi_step_split(bldc_pi_t *pi, float err_p, float err_i, float dt);
