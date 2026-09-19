#include "as5600.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "as5600";

#define AS5600_PIN_SDA       21
#define AS5600_PIN_SCL       22
#define AS5600_I2C_ADDR      0x36
#define AS5600_I2C_HZ        400000
#define AS5600_XFER_MS       50

#define AS5600_REG_STATUS    0x0B
#define AS5600_STATUS_MH     (1u << 3)
#define AS5600_STATUS_ML     (1u << 4)
#define AS5600_STATUS_MD     (1u << 5)
#define AS5600_RAW_MAX       4096
#define AS5600_RPM_WIN_US    200000
#define AS5600_RPM_DEADZONE  0.4f

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ready;
static bool s_have_prev;
static uint16_t s_prev_raw;
static int32_t s_unwrapped;
static int32_t s_rpm_ref_unwrap;
static int64_t s_rpm_ref_us;
static float s_rpm;

static void teardown(void)
{
    if (s_dev != NULL) {
        (void)i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus != NULL) {
        (void)i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_ready = false;
    s_have_prev = false;
    s_unwrapped = 0;
    s_rpm_ref_unwrap = 0;
    s_rpm_ref_us = 0;
    s_rpm = 0.0f;
}

static esp_err_t read_regs(uint8_t start, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &start, 1, buf, len, AS5600_XFER_MS);
}

esp_err_t as5600_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = AS5600_PIN_SDA,
        .scl_io_num = AS5600_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_master_probe(s_bus, AS5600_I2C_ADDR, AS5600_XFER_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "probe 0x%02x failed: %s", AS5600_I2C_ADDR, esp_err_to_name(err));
        teardown();
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = AS5600_I2C_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(err));
        teardown();
        return err;
    }

    uint8_t status = 0;
    err = read_regs(AS5600_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STATUS read failed: %s", esp_err_to_name(err));
        teardown();
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "ready SDA=%d SCL=%d addr=0x%02x status=0x%02x MD=%d ML=%d MH=%d",
             AS5600_PIN_SDA, AS5600_PIN_SCL, AS5600_I2C_ADDR, status,
             (status & AS5600_STATUS_MD) != 0,
             (status & AS5600_STATUS_ML) != 0,
             (status & AS5600_STATUS_MH) != 0);
    return ESP_OK;
}

esp_err_t as5600_read(as5600_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[5];
    esp_err_t err = read_regs(AS5600_REG_STATUS, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t status = buf[0];
    const uint16_t raw = (uint16_t)(((buf[3] & 0x0F) << 8) | buf[4]);
    const int64_t now_us = esp_timer_get_time();

    if (!s_have_prev) {
        s_unwrapped = (int32_t)raw;
        s_rpm_ref_unwrap = s_unwrapped;
        s_rpm_ref_us = now_us;
        s_rpm = 0.0f;
    } else {
        int32_t delta = (int32_t)raw - (int32_t)s_prev_raw;
        if (delta > (AS5600_RAW_MAX / 2)) {
            delta -= AS5600_RAW_MAX;
        } else if (delta < -(AS5600_RAW_MAX / 2)) {
            delta += AS5600_RAW_MAX;
        }
        s_unwrapped += delta;

        const int64_t win_us = now_us - s_rpm_ref_us;
        if (win_us >= AS5600_RPM_WIN_US) {
            const float dt = (float)win_us * 1e-6f;
            float rpm = ((float)(s_unwrapped - s_rpm_ref_unwrap) / (float)AS5600_RAW_MAX) / dt * 60.0f;
            if (rpm > -AS5600_RPM_DEADZONE && rpm < AS5600_RPM_DEADZONE) {
                rpm = 0.0f;
            }
            s_rpm = rpm;
            s_rpm_ref_unwrap = s_unwrapped;
            s_rpm_ref_us = now_us;
        }
    }

    s_prev_raw = raw;
    s_have_prev = true;

    out->raw = raw;
    out->angle_deg = (float)raw * (360.0f / (float)AS5600_RAW_MAX);
    out->rpm = s_rpm;
    out->magnet_ok = (status & AS5600_STATUS_MD) != 0;
    out->magnet_weak = (status & AS5600_STATUS_ML) != 0;
    out->magnet_strong = (status & AS5600_STATUS_MH) != 0;
    return ESP_OK;
}
