#include "bldc_lpf.h"

#include <stddef.h>

void bldc_lpf_reset(bldc_lpf_t *lpf)
{
    if (lpf == NULL) {
        return;
    }
    lpf->y = 0.0f;
}

float bldc_lpf_step(bldc_lpf_t *lpf, float x, float dt)
{
    if (lpf == NULL) {
        return 0.0f;
    }
    if (dt <= 0.0f) {
        return lpf->y;
    }
    if (lpf->tau <= 0.0f) {
        lpf->y = x;
        return lpf->y;
    }

    const float alpha = dt / (dt + lpf->tau);
    lpf->y += alpha * (x - lpf->y);
    return lpf->y;
}
