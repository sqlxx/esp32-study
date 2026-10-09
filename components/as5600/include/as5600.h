#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t raw;       /* 0..4095, ANGLE 0x0E */
    float angle_deg;    /* 0..360 机械角 */
    float rpm;          /* 约 2 ms 内的角度差除以这段时间，未满 2 ms 时保持上次值 */
    bool magnet_ok;     /* STATUS MD */
    bool magnet_weak;   /* ML */
    bool magnet_strong; /* MH */
} as5600_sample_t;

/** I2C 总线 + 读 STATUS 探活。默认 SDA=21 SCL=22。 */
esp_err_t as5600_init(void);

esp_err_t as5600_read(as5600_sample_t *out);

/** 读一次机械角，单位度，范围 0..360。 */
esp_err_t as5600_read_angle_deg(float *angle_deg);

/** 读一次 ANGLE。转速按约 2 ms 的角度差计算，单位 rpm。 */
esp_err_t as5600_read_rpm(float *rpm);

#ifdef __cplusplus
}
#endif
