#include "bldc.h"
#include "bldc_pwm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "bldc";

#define BLDC_POLE_PAIRS          7
#define BLDC_MODULATION_DEFAULT  0.15f
#define BLDC_TASK_PERIOD_MS      2
#define BLDC_TWO_PI              6.283185307179586f
#define BLDC_ONE_TWENTY_RAD      2.0943951023931953f

#define BLDC_RPM_MAX 300.0f

static volatile bool s_enabled;
static volatile float s_rpm;
static volatile float s_modulation = BLDC_MODULATION_DEFAULT;
static volatile float s_theta;
static volatile float s_u;
static volatile float s_v;
static volatile float s_w;
static TaskHandle_t s_task;

static void store_duty(float u, float v, float w)
{
    s_u = u;
    s_v = v;
    s_w = w;
    (void)bldc_pwm_set_duty(u, v, w);
}

static void openloop_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(BLDC_TASK_PERIOD_MS);
    if (period < 1) {
        period = 1;
    }
    const float dt = (float)period / (float)configTICK_RATE_HZ;

    while (true) {
        if (!s_enabled) {
            s_theta = 0.0f;
            vTaskDelayUntil(&last, period);
            continue;
        }

        const float rpm = s_rpm;
        const float m = s_modulation;
        if (rpm == 0.0f || m <= 0.0f) {
            store_duty(0.5f, 0.5f, 0.5f);
            vTaskDelayUntil(&last, period);
            continue;
        }

        const float elec_hz = (rpm / 60.0f) * (float)BLDC_POLE_PAIRS;
        s_theta += BLDC_TWO_PI * elec_hz * dt;
        if (s_theta > BLDC_TWO_PI) {
            s_theta -= BLDC_TWO_PI;
        } else if (s_theta < 0.0f) {
            s_theta += BLDC_TWO_PI;
        }

        store_duty(0.5f + 0.5f * m * sinf(s_theta),
                   0.5f + 0.5f * m * sinf(s_theta - BLDC_ONE_TWENTY_RAD),
                   0.5f + 0.5f * m * sinf(s_theta + BLDC_ONE_TWENTY_RAD));

        vTaskDelayUntil(&last, period);
    }
}

esp_err_t bldc_init(void)
{
    if (s_task != NULL) {
        return ESP_OK;
    }

    esp_err_t err = bldc_pwm_init();
    if (err != ESP_OK) {
        return err;
    }

    store_duty(0.0f, 0.0f, 0.0f);

    BaseType_t ok = xTaskCreate(openloop_task, "bldc_ol", 2048, NULL, 6, &s_task);
    if (ok != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "create openloop task failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "openloop ready, pp=%d, m=%.2f, rpm=0 (call bldc_enable + set rpm)",
             BLDC_POLE_PAIRS, (double)s_modulation);
    return ESP_OK;
}

esp_err_t bldc_enable(void)
{
    if (s_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = bldc_pwm_set_enable(true);
    if (err != ESP_OK) {
        return err;
    }
    s_enabled = true;
    ESP_LOGI(TAG, "enabled, rpm=%.1f m=%.2f", (double)s_rpm, (double)s_modulation);
    return ESP_OK;
}

void bldc_disable(void)
{
    s_enabled = false;
    (void)bldc_pwm_set_enable(false);
    store_duty(0.0f, 0.0f, 0.0f);
    ESP_LOGI(TAG, "disabled");
}

bool bldc_is_enabled(void)
{
    return s_enabled;
}

esp_err_t bldc_set_openloop_rpm(float rpm)
{
    if (rpm > BLDC_RPM_MAX) {
        rpm = BLDC_RPM_MAX;
    } else if (rpm < -BLDC_RPM_MAX) {
        rpm = -BLDC_RPM_MAX;
    }
    s_rpm = rpm;
    ESP_LOGI(TAG, "rpm -> %.1f", (double)s_rpm);
    return ESP_OK;
}

float bldc_get_openloop_rpm(void)
{
    return s_rpm;
}

esp_err_t bldc_set_modulation(float modulation)
{
    if (modulation < 0.0f) {
        modulation = 0.0f;
    } else if (modulation > 1.0f) {
        modulation = 1.0f;
    }
    s_modulation = modulation;
    ESP_LOGI(TAG, "modulation -> %.2f", (double)s_modulation);
    return ESP_OK;
}

float bldc_get_modulation(void)
{
    return s_modulation;
}

void bldc_get_status(bldc_status_t *out)
{
    if (out == NULL) {
        return;
    }
    out->enabled = s_enabled;
    out->rpm = s_rpm;
    out->modulation = s_modulation;
    out->theta_rad = s_theta;
    out->u = s_u;
    out->v = s_v;
    out->w = s_w;
}
