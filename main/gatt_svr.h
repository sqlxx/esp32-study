#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bldc.h"
#include "bmi088.h"

#ifdef __cplusplus
extern "C" {
#endif

int gatt_svr_init(void);

void gatt_svr_on_connect(uint16_t conn_handle);
void gatt_svr_on_disconnect(void);
void gatt_svr_on_subscribe(uint16_t attr_handle, uint16_t conn_handle, bool notify_enabled);

/** 已订阅时推送 12 字节 LE：acc(0.001 g) + gyro(0.1 dps)，各 int16 x/y/z。 */
void gatt_svr_notify_imu(const bmi088_vec3_t *acc, const bmi088_vec3_t *gyr);

/** 已订阅时推送 14 字节 LE：rpm×10, en, pad, u/v/w×1000, θ°×10, m×1000。 */
void gatt_svr_notify_bldc(const bldc_status_t *st);

#ifdef __cplusplus
}
#endif
