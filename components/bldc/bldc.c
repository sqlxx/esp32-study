#include "bldc.h"
#include "bldc_ctrl.h"
#include "bldc_pwm.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

static const char *TAG = "bldc";

#define BLDC_TASK_PERIOD_MS 1

static TaskHandle_t s_task;

static void bldc_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(BLDC_TASK_PERIOD_MS);
    if (period < 1) {
        period = 1;
    }
    int64_t prev_us = esp_timer_get_time();

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - prev_us) * 1.0e-6f;
        prev_us = now_us;
        if (dt < 1.0e-4f) {
            dt = 1.0e-4f;
        } else if (dt > 0.05f) {
            dt = 0.05f;
        }
        bldc_step(dt);
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

    bldc_ctrl_init();

    BaseType_t ok = xTaskCreate(bldc_task, "bldc", 3072, NULL, 6, &s_task);
    if (ok != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "create task failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "openloop ready, pp=%d, m=%.2f, rpm=0 (call bldc_enable + set rpm)",
             bldc_ctrl_pole_pairs(), (double)bldc_ctrl_get_modulation());
    return ESP_OK;
}

esp_err_t bldc_enable(void)
{
    if (s_task == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bldc_ctrl_set_enabled(true);
    ESP_LOGI(TAG, "enabled, rpm=%.1f m=%.2f", (double)bldc_ctrl_get_rpm(),
             (double)bldc_ctrl_get_modulation());
    return ESP_OK;
}

void bldc_disable(void)
{
    bldc_ctrl_set_enabled(false);
    ESP_LOGI(TAG, "disabled");
}

bool bldc_is_enabled(void)
{
    return bldc_ctrl_is_enabled();
}

esp_err_t bldc_set_openloop_rpm(float rpm)
{
    esp_err_t err = bldc_ctrl_set_rpm(rpm);
    ESP_LOGI(TAG, "rpm -> %.1f", (double)bldc_ctrl_get_rpm());
    return err;
}

float bldc_get_openloop_rpm(void)
{
    return bldc_ctrl_get_rpm();
}

esp_err_t bldc_set_modulation(float modulation)
{
    esp_err_t err = bldc_ctrl_set_modulation(modulation);
    ESP_LOGI(TAG, "modulation -> %.2f", (double)bldc_ctrl_get_modulation());
    return err;
}

float bldc_get_modulation(void)
{
    return bldc_ctrl_get_modulation();
}

static esp_err_t log_gain(const char *name, float v, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s rejected: %.4f", name, (double)v);
        return err;
    }
    ESP_LOGI(TAG, "%s -> %.4f", name, (double)v);
    return ESP_OK;
}

esp_err_t bldc_set_velocity_kp(float kp)
{
    return log_gain("vel kp", kp, bldc_ctrl_set_velocity_kp(kp));
}

esp_err_t bldc_set_velocity_ki(float ki)
{
    return log_gain("vel ki", ki, bldc_ctrl_set_velocity_ki(ki));
}

esp_err_t bldc_set_velocity_tau(float tau_s)
{
    return log_gain("vel tau", tau_s, bldc_ctrl_set_velocity_tau(tau_s));
}

esp_err_t bldc_set_position_kp(float kp)
{
    return log_gain("pos kp", kp, bldc_ctrl_set_position_kp(kp));
}

esp_err_t bldc_set_position_ki(float ki)
{
    return log_gain("pos ki", ki, bldc_ctrl_set_position_ki(ki));
}

esp_err_t bldc_set_motion(bldc_motion_t motion)
{
    esp_err_t err = bldc_ctrl_set_motion(motion);
    if (err != ESP_OK) {
        return err;
    }
    if (motion == BLDC_MOTION_POSITION) {
        ESP_LOGI(TAG, "motion -> position, hold %.1f deg", (double)bldc_ctrl_get_position_deg());
    } else if (motion == BLDC_MOTION_VELOCITY) {
        ESP_LOGI(TAG, "motion -> velocity, %.1f rpm", (double)bldc_ctrl_get_rpm());
    } else {
        ESP_LOGI(TAG, "motion -> openloop");
    }
    return ESP_OK;
}

bldc_motion_t bldc_get_motion(void)
{
    return bldc_ctrl_get_motion();
}

esp_err_t bldc_set_position_deg(float deg)
{
    esp_err_t err = bldc_ctrl_set_position_deg(deg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "position -> %.1f deg", (double)bldc_ctrl_get_position_deg());
    }
    return err;
}

float bldc_get_position_deg(void)
{
    return bldc_ctrl_get_position_deg();
}

void bldc_get_status(bldc_status_t *out)
{
    if (out == NULL) {
        return;
    }

    bldc_ctrl_status_t st;
    bldc_ctrl_get_status(&st);
    out->enabled = bldc_ctrl_is_enabled();
    out->rpm = st.rpm;
    out->modulation = st.modulation;
    out->theta_rad = st.theta_rad;
    out->u = st.u;
    out->v = st.v;
    out->w = st.w;
}
