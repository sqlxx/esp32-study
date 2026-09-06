#pragma once

#include "esp_err.h"
#include "bosch/bmi08_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bmi088_port_init(struct bmi08_dev *dev);

#ifdef __cplusplus
}
#endif
