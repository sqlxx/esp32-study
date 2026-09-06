#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} bmi088_vec3_t;

/** SPI + 软复位 + 量程/ODR。加速度单位 g，陀螺仪单位 dps。 */
esp_err_t bmi088_init(void);

esp_err_t bmi088_read_accel(bmi088_vec3_t *out);
esp_err_t bmi088_read_gyro(bmi088_vec3_t *out);

#ifdef __cplusplus
}
#endif
