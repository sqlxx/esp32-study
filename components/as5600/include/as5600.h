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
    float rpm;          /* unwrap 后约 200ms 窗口平均 */
    bool magnet_ok;     /* STATUS MD */
    bool magnet_weak;   /* ML */
    bool magnet_strong; /* MH */
} as5600_sample_t;

/** I2C 总线 + 读 STATUS 探活。默认 SDA=21 SCL=22。 */
esp_err_t as5600_init(void);

esp_err_t as5600_read(as5600_sample_t *out);

#ifdef __cplusplus
}
#endif
