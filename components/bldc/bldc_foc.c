#include "bldc_foc.h"

#include "bldc_pwm.h"

#include <math.h>

/* √3 / 2 */
#define BLDC_SQRT3_OVER_2 0.8660254037844386f

esp_err_t bldc_modulate(float ud, float uq, float theta_e, float duty_uvw[3])
{
    const float c = cosf(theta_e);
    const float s = sinf(theta_e);

    /* 反 Park：d 轴在电气角 theta_e，q 轴超前 90° */
    const float alpha = ud * c - uq * s;
    const float beta = ud * s + uq * c;

    /* 反 Clarke，正弦 PWM，三相之和为 0 */
    const float u = alpha;
    const float v = -0.5f * alpha + BLDC_SQRT3_OVER_2 * beta;
    const float w = -0.5f * alpha - BLDC_SQRT3_OVER_2 * beta;

    const float duty_u = 0.5f + 0.5f * u;
    const float duty_v = 0.5f + 0.5f * v;
    const float duty_w = 0.5f + 0.5f * w;

    if (duty_uvw != NULL) {
        duty_uvw[0] = duty_u;
        duty_uvw[1] = duty_v;
        duty_uvw[2] = duty_w;
    }
    return bldc_pwm_set_duty(duty_u, duty_v, duty_w);
}
