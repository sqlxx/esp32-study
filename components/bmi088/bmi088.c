#include "bmi088.h"
#include "bmi088_port.h"
#include "bosch/bmi08x.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

static const char *TAG = "bmi088";
static struct bmi08_dev s_dev;
static bool s_ready;

static esp_err_t bosch_err(int8_t rslt)
{
    if (rslt == BMI08_OK) {
        return ESP_OK;
    }
    if (rslt == BMI08_E_DEV_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_FAIL;
}

static float accel_lsb_per_g(void)
{
    switch (s_dev.accel_cfg.range) {
    case BMI088_ACCEL_RANGE_3G:
        return 10920.0f;
    case BMI088_ACCEL_RANGE_6G:
        return 5460.0f;
    case BMI088_ACCEL_RANGE_12G:
        return 2730.0f;
    case BMI088_ACCEL_RANGE_24G:
        return 1365.0f;
    default:
        return 5460.0f;
    }
}

static float gyro_lsb_per_dps(void)
{
    switch (s_dev.gyro_cfg.range) {
    case BMI08_GYRO_RANGE_125_DPS:
        return 262.144f;
    case BMI08_GYRO_RANGE_250_DPS:
        return 131.072f;
    case BMI08_GYRO_RANGE_500_DPS:
        return 65.536f;
    case BMI08_GYRO_RANGE_1000_DPS:
        return 32.768f;
    case BMI08_GYRO_RANGE_2000_DPS:
        return 16.384f;
    default:
        return 65.536f;
    }
}

esp_err_t bmi088_init(void)
{
    esp_err_t err = bmi088_port_init(&s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed: %s", esp_err_to_name(err));
        return err;
    }

    int8_t rslt = bmi08xa_init(&s_dev);
    if (rslt != BMI08_OK) {
        ESP_LOGE(TAG, "accel init failed, rslt=%d chip_id=0x%02x", rslt, s_dev.accel_chip_id);
        return bosch_err(rslt);
    }

    rslt = bmi08g_init(&s_dev);
    if (rslt != BMI08_OK) {
        ESP_LOGE(TAG, "gyro init failed, rslt=%d chip_id=0x%02x", rslt, s_dev.gyro_chip_id);
        return bosch_err(rslt);
    }

    s_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    s_dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    s_dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
    s_dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    rslt = bmi08a_set_power_mode(&s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    rslt = bmi08xa_set_meas_conf(&s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }

    s_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    s_dev.gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS;
    s_dev.gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;
    s_dev.gyro_cfg.bw = BMI08_GYRO_BW_47_ODR_400_HZ;
    rslt = bmi08g_set_power_mode(&s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }
    rslt = bmi08g_set_meas_conf(&s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }

    s_ready = true;
    ESP_LOGI(TAG, "ready, acc_id=0x%02x gyro_id=0x%02x", s_dev.accel_chip_id, s_dev.gyro_chip_id);
    return ESP_OK;
}

esp_err_t bmi088_read_accel(bmi088_vec3_t *out)
{
    if (!s_ready || out == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi08_sensor_data raw;
    int8_t rslt = bmi08a_get_data(&raw, &s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }

    float scale = accel_lsb_per_g();
    out->x = (float)raw.x / scale;
    out->y = (float)raw.y / scale;
    out->z = (float)raw.z / scale;
    return ESP_OK;
}

esp_err_t bmi088_read_gyro(bmi088_vec3_t *out)
{
    if (!s_ready || out == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    struct bmi08_sensor_data raw;
    int8_t rslt = bmi08g_get_data(&raw, &s_dev);
    if (rslt != BMI08_OK) {
        return bosch_err(rslt);
    }

    float scale = gyro_lsb_per_dps();
    out->x = (float)raw.x / scale;
    out->y = (float)raw.y / scale;
    out->z = (float)raw.z / scale;
    return ESP_OK;
}
