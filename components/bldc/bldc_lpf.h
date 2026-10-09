#pragma once

typedef struct {
    float tau; /* 时间常数，秒。≤ 0 时输出直接跟上输入 */
    float y;
} bldc_lpf_t;

void bldc_lpf_reset(bldc_lpf_t *lpf);

/**
 * 一阶低通：y += dt / (dt + tau) * (x - y)。
 * dt ≤ 0 时保持上次输出。
 */
float bldc_lpf_step(bldc_lpf_t *lpf, float x, float dt);
