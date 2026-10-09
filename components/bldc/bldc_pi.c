#include "bldc_pi.h"

#include <stdbool.h>
#include <stddef.h>

static float clamp(float x, float lo, float hi)
{
    if (x > hi) {
        return hi;
    }
    if (x < lo) {
        return lo;
    }
    return x;
}

void bldc_pi_reset(bldc_pi_t *pi)
{
    if (pi == NULL) {
        return;
    }
    pi->integral = 0.0f;
}

float bldc_pi_step(bldc_pi_t *pi, float err, float dt)
{
    return bldc_pi_step_split(pi, err, err, dt);
}

float bldc_pi_step_split(bldc_pi_t *pi, float err_p, float err_i, float dt)
{
    if (pi == NULL) {
        return 0.0f;
    }

    const float p_term = pi->kp * err_p;

    /* 积分项本身不超过输出限幅。比例项再大也不能把积分清掉，
     * 否则转速误差一缩小，uq 立刻掉下去，盖不住反电动势。 */
    float i_term = clamp(pi->ki * pi->integral, pi->out_min, pi->out_max);
    if (dt > 0.0f) {
        const float i_next =
            clamp(pi->ki * (pi->integral + err_i * dt), pi->out_min, pi->out_max);
        const float out_next = p_term + i_next;
        const bool wind_hi = out_next > pi->out_max && i_next > i_term;
        const bool wind_lo = out_next < pi->out_min && i_next < i_term;
        if (!wind_hi && !wind_lo) {
            i_term = i_next;
        }
    }

    if (pi->ki != 0.0f) {
        pi->integral = i_term / pi->ki;
    } else {
        pi->integral = 0.0f;
    }
    return clamp(p_term + i_term, pi->out_min, pi->out_max);
}
