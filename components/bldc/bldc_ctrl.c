#include "bldc_ctrl.h"

#include "as5600.h"
#include "bldc_foc.h"
#include "bldc_lpf.h"
#include "bldc_pi.h"
#include "bldc_pwm.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include <math.h>

static const char *TAG = "bldc_ctrl";

#define BLDC_POLE_PAIRS         7
/* 方向和电气零位的初值。第一次进入位置环或速度环时按编码器改掉。 */
#define BLDC_SENSOR_DIRECTION   1
#define BLDC_ELEC_OFFSET_RAD    0.0f
#define BLDC_CAL_ALIGN_US       500000
#define BLDC_CAL_NUDGE_US       400000
#define BLDC_CAL_MIN_COUNTS     40
#define BLDC_CAL_MAX_TRIES      3
#define BLDC_MODULATION_DEFAULT 0.15f
#define BLDC_RPM_MAX            300.0f
#define BLDC_TWO_PI             6.283185307179586f
#define BLDC_PI                 3.141592653589793f
/* 机械角误差（rad）到机械角速度（rad/s）。180° 误差约 120 rpm。 */
#define BLDC_POS_KP             4.0f
#define BLDC_POS_KI             0.0f
/* 位置环和速度环共用：AS5600 转速经低通后进速度 PI。 */
#define BLDC_SPD_KP_VEL         0.07f
#define BLDC_SPD_KI_VEL         0.1f
#define BLDC_SPD_LPF_S          0.05f
#define BLDC_GAIN_MAX           1000.0f
#define BLDC_TAU_MAX            10.0f
#define BLDC_UQ_ABS_MAX         0.95f
#define BLDC_DBG_US             1000000
#define BLDC_RAW_MAX            4096.0f

static volatile bldc_motion_t s_motion = BLDC_MOTION_OPENLOOP;
static bldc_torque_t s_torque = BLDC_TORQUE_VOLTAGE;
static volatile bool s_enabled;
static volatile bool s_reset_pi;
static bool s_output_off;
static volatile float s_rpm;
static volatile float s_modulation = BLDC_MODULATION_DEFAULT;
static volatile float s_pos_target;
static volatile float s_mech;
static volatile float s_theta;
static volatile float s_ud;
static volatile float s_uq;
static volatile float s_u;
static volatile float s_v;
static volatile float s_w;
static bldc_pi_t s_pos_pi;
static bldc_pi_t s_spd_pi;
static portMUX_TYPE s_pi_mux = portMUX_INITIALIZER_UNLOCKED;
static bldc_lpf_t s_vel_lpf;
static int64_t s_enc_log_us;
static int64_t s_dbg_us;
static int s_sensor_direction = BLDC_SENSOR_DIRECTION;
static float s_elec_offset = BLDC_ELEC_OFFSET_RAD;

typedef enum {
    CAL_IDLE = 0,
    CAL_ALIGN,
    CAL_NUDGE,
    CAL_FAIL,
} cal_state_t;

static cal_state_t s_cal;
static bool s_calibrated;
static int s_cal_tries;
static int64_t s_cal_t0;
static uint16_t s_cal_raw0;

static float rpm_limit_rad_s(void)
{
    return (float)BLDC_RPM_MAX * BLDC_TWO_PI / 60.0f;
}

/** 速度环 |uq| 上限：不超过调制度，最高 0.95。 */
static float uq_limit(float modulation)
{
    if (modulation <= 0.0f) {
        return 0.0f;
    }
    if (modulation > BLDC_UQ_ABS_MAX) {
        return BLDC_UQ_ABS_MAX;
    }
    return modulation;
}

static void write_spd_limit(float modulation)
{
    const float lim = uq_limit(modulation);
    portENTER_CRITICAL(&s_pi_mux);
    s_spd_pi.out_min = -lim;
    s_spd_pi.out_max = lim;
    portEXIT_CRITICAL(&s_pi_mux);
}

/** ki 数值变化时清积分。与步进共用锁，避免这一拍的写回盖掉清零。 */
static void write_pi_ki(bldc_pi_t *pi, float ki)
{
    portENTER_CRITICAL(&s_pi_mux);
    if (ki != pi->ki) {
        pi->ki = ki;
        pi->integral = 0.0f;
    }
    portEXIT_CRITICAL(&s_pi_mux);
}

static float step_pi(bldc_pi_t *pi, float err, float dt)
{
    portENTER_CRITICAL(&s_pi_mux);
    const float out = bldc_pi_step(pi, err, dt);
    portEXIT_CRITICAL(&s_pi_mux);
    return out;
}

static void reset_pi(bldc_pi_t *pi)
{
    portENTER_CRITICAL(&s_pi_mux);
    bldc_pi_reset(pi);
    portEXIT_CRITICAL(&s_pi_mux);
}

static bool gain_in_range(float v, float max)
{
    return isfinite(v) && v >= 0.0f && v <= max;
}

static void init_pos_pi(void)
{
    const float lim = rpm_limit_rad_s();
    s_pos_pi.kp = BLDC_POS_KP;
    s_pos_pi.ki = BLDC_POS_KI;
    s_pos_pi.integral = 0.0f;
    s_pos_pi.out_min = -lim;
    s_pos_pi.out_max = lim;
}

static void init_spd_pi(void)
{
    s_spd_pi.kp = BLDC_SPD_KP_VEL;
    s_spd_pi.ki = BLDC_SPD_KI_VEL;
    s_spd_pi.integral = 0.0f;
    write_spd_limit(BLDC_MODULATION_DEFAULT);
}

static void reset_speed_state(void)
{
    bldc_lpf_reset(&s_vel_lpf);
    reset_pi(&s_spd_pi);
}

static float wrap_two_pi(float theta)
{
    theta = fmodf(theta, BLDC_TWO_PI);
    if (theta < 0.0f) {
        theta += BLDC_TWO_PI;
    }
    return theta;
}

static float wrap_pi(float theta)
{
    theta = wrap_two_pi(theta);
    if (theta > BLDC_PI) {
        theta -= BLDC_TWO_PI;
    }
    return theta;
}

static float rad_to_deg(float rad)
{
    float deg = rad * (180.0f / BLDC_PI);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

static bool read_sample(as5600_sample_t *sample)
{
    return as5600_read(sample) == ESP_OK && sample->magnet_ok;
}

static float raw_to_rad(uint16_t raw)
{
    return (float)raw * (BLDC_TWO_PI / BLDC_RAW_MAX);
}

static bool read_raw(uint16_t *raw)
{
    as5600_sample_t sample;
    if (!read_sample(&sample)) {
        return false;
    }
    *raw = sample.raw;
    return true;
}

static int32_t raw_delta(uint16_t now, uint16_t prev)
{
    int32_t delta = (int32_t)now - (int32_t)prev;
    const int32_t half = (int32_t)(BLDC_RAW_MAX / 2.0f);
    const int32_t full = (int32_t)BLDC_RAW_MAX;
    if (delta > half) {
        delta -= full;
    } else if (delta < -half) {
        delta += full;
    }
    return delta;
}

/** enc_rad、rpm_enc 都不加方向。mech_rad 是编码器坐标系，给状态显示。 */
static bool read_shaft(float *enc_rad, float *mech_rad, float *rpm_enc)
{
    as5600_sample_t sample;
    if (!read_sample(&sample)) {
        return false;
    }

    *enc_rad = raw_to_rad(sample.raw);
    *mech_rad = wrap_pi(*enc_rad);
    *rpm_enc = sample.rpm;
    return true;
}

static float calib_voltage(float modulation)
{
    float v = modulation;
    if (v < 0.12f) {
        v = 0.12f;
    } else if (v > 0.25f) {
        v = 0.25f;
    }
    return v;
}

static void calib_fail(void)
{
    s_cal = CAL_FAIL;
    ESP_LOGW(TAG, "electrical align failed");
}

/** 零位还没对准时写 ud/uq/theta，调用方不要跑位置环和速度环。 */
static void drive_uncalibrated(float modulation, float *ud, float *uq, float *theta)
{
    const int64_t now = esp_timer_get_time();
    const float v = calib_voltage(modulation);

    if (s_cal == CAL_IDLE) {
        s_cal = CAL_ALIGN;
        s_cal_t0 = now;
        ESP_LOGI(TAG, "align electrical zero");
    }
    if (s_cal == CAL_FAIL) {
        *ud = 0.0f;
        *uq = 0.0f;
        *theta = 0.0f;
        return;
    }

    const int64_t elapsed = now - s_cal_t0;
    if (s_cal == CAL_ALIGN) {
        *ud = v;
        *uq = 0.0f;
        *theta = 0.0f;
        if (elapsed < BLDC_CAL_ALIGN_US) {
            return;
        }
        uint16_t raw = 0;
        if (!read_raw(&raw)) {
            if (elapsed > 2000000) {
                calib_fail();
            }
            return;
        }
        s_cal_raw0 = raw;
        s_cal = CAL_NUDGE;
        s_cal_t0 = now;
        return;
    }

    float prog = (float)elapsed / (float)BLDC_CAL_NUDGE_US;
    if (prog > 1.0f) {
        prog = 1.0f;
    }
    *ud = 0.0f;
    *uq = v;
    *theta = prog * BLDC_PI;
    if (elapsed < BLDC_CAL_NUDGE_US) {
        return;
    }

    uint16_t raw = 0;
    if (!read_raw(&raw)) {
        if (elapsed > BLDC_CAL_NUDGE_US + 1500000) {
            calib_fail();
        }
        return;
    }

    const int32_t delta = raw_delta(raw, s_cal_raw0);
    if (delta > -BLDC_CAL_MIN_COUNTS && delta < BLDC_CAL_MIN_COUNTS) {
        s_cal_tries++;
        ESP_LOGW(TAG, "align nudge only %d counts", (int)delta);
        if (s_cal_tries >= BLDC_CAL_MAX_TRIES) {
            calib_fail();
            return;
        }
        s_cal = CAL_ALIGN;
        s_cal_t0 = now;
        return;
    }

    s_sensor_direction = delta > 0 ? 1 : -1;
    s_elec_offset = -(float)s_sensor_direction * (float)BLDC_POLE_PAIRS * raw_to_rad(s_cal_raw0);
    s_calibrated = true;
    s_cal = CAL_IDLE;
    reset_pi(&s_pos_pi);
    reset_speed_state();
    ESP_LOGI(TAG, "aligned dir=%d offset=%.3f", s_sensor_direction, (double)s_elec_offset);
}

static void note_encoder_lost(void)
{
    const int64_t now = esp_timer_get_time();
    if (now - s_enc_log_us < 1000000) {
        return;
    }
    s_enc_log_us = now;
    ESP_LOGW(TAG, "encoder unavailable, uq=0");
}

static float elec_rad_per_sec(float rpm)
{
    return (rpm / 60.0f) * (float)BLDC_POLE_PAIRS * BLDC_TWO_PI;
}

static void apply_voltage(float ud, float uq, float theta_e)
{
    float duty[3];
    (void)bldc_modulate(ud, uq, theta_e, duty);
    s_ud = ud;
    s_uq = uq;
    s_u = duty[0];
    s_v = duty[1];
    s_w = duty[2];
    s_output_off = false;
}

static void force_output_off(void)
{
    if (s_output_off) {
        return;
    }
    s_theta = 0.0f;
    s_ud = 0.0f;
    s_uq = 0.0f;
    s_u = 0.0f;
    s_v = 0.0f;
    s_w = 0.0f;
    reset_pi(&s_pos_pi);
    reset_speed_state();
    s_reset_pi = false;
    (void)bldc_pwm_set_duty(0.0f, 0.0f, 0.0f);
    (void)bldc_pwm_set_enable(false);
    s_output_off = true;
}

void bldc_ctrl_init(void)
{
    s_motion = BLDC_MOTION_OPENLOOP;
    s_torque = BLDC_TORQUE_VOLTAGE;
    s_enabled = false;
    s_reset_pi = false;
    s_output_off = false;
    s_rpm = 0.0f;
    s_pos_target = 0.0f;
    s_mech = 0.0f;
    s_cal = CAL_IDLE;
    s_calibrated = false;
    s_cal_tries = 0;
    s_sensor_direction = BLDC_SENSOR_DIRECTION;
    s_elec_offset = BLDC_ELEC_OFFSET_RAD;
    init_pos_pi();
    init_spd_pi();
    s_vel_lpf.tau = BLDC_SPD_LPF_S;
    reset_speed_state();
    s_modulation = BLDC_MODULATION_DEFAULT;
    s_theta = 0.0f;
    s_ud = 0.0f;
    s_uq = 0.0f;
    s_u = 0.0f;
    s_v = 0.0f;
    s_w = 0.0f;
    force_output_off();
}

void bldc_ctrl_set_enabled(bool on)
{
    s_enabled = on;
}

bool bldc_ctrl_is_enabled(void)
{
    return s_enabled;
}

esp_err_t bldc_ctrl_set_rpm(float rpm)
{
    if (rpm > BLDC_RPM_MAX) {
        rpm = BLDC_RPM_MAX;
    } else if (rpm < -BLDC_RPM_MAX) {
        rpm = -BLDC_RPM_MAX;
    }
    s_rpm = rpm;
    return ESP_OK;
}

float bldc_ctrl_get_rpm(void)
{
    return s_rpm;
}

esp_err_t bldc_ctrl_set_modulation(float modulation)
{
    if (modulation < 0.0f) {
        modulation = 0.0f;
    } else if (modulation > 1.0f) {
        modulation = 1.0f;
    }
    s_modulation = modulation;
    write_spd_limit(modulation);
    return ESP_OK;
}

float bldc_ctrl_get_modulation(void)
{
    return s_modulation;
}

esp_err_t bldc_ctrl_set_velocity_kp(float kp)
{
    if (!gain_in_range(kp, BLDC_GAIN_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_spd_pi.kp = kp;
    return ESP_OK;
}

esp_err_t bldc_ctrl_set_velocity_ki(float ki)
{
    if (!gain_in_range(ki, BLDC_GAIN_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    write_pi_ki(&s_spd_pi, ki);
    return ESP_OK;
}

esp_err_t bldc_ctrl_set_velocity_tau(float tau_s)
{
    if (!gain_in_range(tau_s, BLDC_TAU_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_vel_lpf.tau = tau_s;
    return ESP_OK;
}

esp_err_t bldc_ctrl_set_position_kp(float kp)
{
    if (!gain_in_range(kp, BLDC_GAIN_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_pos_pi.kp = kp;
    return ESP_OK;
}

esp_err_t bldc_ctrl_set_position_ki(float ki)
{
    if (!gain_in_range(ki, BLDC_GAIN_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }
    write_pi_ki(&s_pos_pi, ki);
    return ESP_OK;
}

esp_err_t bldc_ctrl_set_motion(bldc_motion_t motion)
{
    if (motion != BLDC_MOTION_OPENLOOP && motion != BLDC_MOTION_POSITION &&
        motion != BLDC_MOTION_VELOCITY) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motion == BLDC_MOTION_POSITION || motion == BLDC_MOTION_VELOCITY) {
        uint16_t raw = 0;
        if (!read_raw(&raw)) {
            return ESP_ERR_INVALID_STATE;
        }
        if (motion == BLDC_MOTION_POSITION && s_motion != BLDC_MOTION_POSITION) {
            s_pos_target = wrap_pi(raw_to_rad(raw));
            s_mech = s_pos_target;
        }
    }
    s_motion = motion;
    s_reset_pi = true;
    return ESP_OK;
}

bldc_motion_t bldc_ctrl_get_motion(void)
{
    return s_motion;
}

esp_err_t bldc_ctrl_set_position_deg(float deg)
{
    s_pos_target = wrap_pi(deg * (BLDC_PI / 180.0f));
    return ESP_OK;
}

float bldc_ctrl_get_position_deg(void)
{
    return rad_to_deg(s_pos_target);
}

int bldc_ctrl_pole_pairs(void)
{
    return BLDC_POLE_PAIRS;
}

float bldc_ctrl_mech_to_elec(float mech_rad)
{
    float theta = (float)s_sensor_direction * (float)BLDC_POLE_PAIRS * mech_rad + s_elec_offset;
    return wrap_two_pi(theta);
}

void bldc_ctrl_get_status(bldc_ctrl_status_t *out)
{
    if (out == NULL) {
        return;
    }
    out->motion = s_motion;
    out->torque = s_torque;
    out->rpm = s_rpm;
    out->modulation = s_modulation;
    out->theta_rad = s_theta;
    out->ud = s_ud;
    out->uq = s_uq;
    out->u = s_u;
    out->v = s_v;
    out->w = s_w;
    out->pos_deg = bldc_ctrl_get_position_deg();
    out->mech_deg = rad_to_deg(s_mech);
}

void bldc_step(float dt)
{
    if (!s_enabled) {
        if (!s_calibrated) {
            s_cal = CAL_IDLE;
            s_cal_tries = 0;
        }
        force_output_off();
        return;
    }

    if (s_reset_pi) {
        reset_pi(&s_pos_pi);
        reset_speed_state();
        s_reset_pi = false;
    }

    const bldc_motion_t motion = s_motion;
    const float modulation = s_modulation;
    float theta = s_theta;
    float ud = 0.0f;
    float uq = 0.0f;

    if ((motion == BLDC_MOTION_POSITION || motion == BLDC_MOTION_VELOCITY) &&
        s_torque == BLDC_TORQUE_VOLTAGE) {
        if (!s_calibrated) {
            drive_uncalibrated(modulation, &ud, &uq, &theta);
        } else {
            /* 读取编码器，得到机械角、电角度、转速。 */
            float enc_rad = 0.0f;
            float mech = 0.0f;
            float rpm_enc = 0.0f;
            const bool have_shaft = read_shaft(&enc_rad, &mech, &rpm_enc);
            if (!have_shaft) {
                note_encoder_lost();
            } else {
                s_mech = mech;
            }

            float omega_ref = s_rpm * (BLDC_TWO_PI / 60.0f);
            float omega_meas = s_vel_lpf.y;
            float omega_raw = 0.0f;
            if (have_shaft) {
                omega_raw = (float)s_sensor_direction * rpm_enc * (BLDC_TWO_PI / 60.0f);
                omega_meas = bldc_lpf_step(&s_vel_lpf, omega_raw, dt); //低通滤波
                if (motion == BLDC_MOTION_POSITION) {
                    omega_ref = step_pi(
                        &s_pos_pi,
                        (float)s_sensor_direction * wrap_pi(s_pos_target - mech),
                        dt);
                }
                /* 电流环打开后，这里的 uq 改为 iq_ref，再由电流 PI 得到 ud、uq。 */
                uq = step_pi(&s_spd_pi, omega_ref - omega_meas, dt);
                theta = bldc_ctrl_mech_to_elec(enc_rad);
            }

            const int64_t now = esp_timer_get_time();
            if (now - s_dbg_us >= BLDC_DBG_US) {
                s_dbg_us = now;
                const float rad_s_to_rpm = 60.0f / BLDC_TWO_PI;
                ESP_LOGI(TAG, "loop: motion=%s ref=%.1f rpm meas=%.1f raw=%.1f uq=%+.3f lim=%.3f",
                         motion == BLDC_MOTION_POSITION ? "pos" : "vel",
                         (double)(omega_ref * rad_s_to_rpm), (double)(omega_meas * rad_s_to_rpm),
                         (double)(omega_raw * rad_s_to_rpm), (double)uq, (double)s_spd_pi.out_max);
            }
        }
    } else {
        if (!s_calibrated) {
            s_cal = CAL_IDLE;
            s_cal_tries = 0;
        }
        if (motion == BLDC_MOTION_OPENLOOP && s_torque == BLDC_TORQUE_VOLTAGE && s_rpm != 0.0f &&
            modulation > 0.0f) {
            theta = wrap_two_pi(theta + elec_rad_per_sec(s_rpm) * dt);
            uq = modulation;
        }
    }

    /* 本拍计算期间若已请求关断，丢掉电压，只关输出。 */
    if (!s_enabled) {
        force_output_off();
        return;
    }

    s_theta = theta;
    apply_voltage(ud, uq, theta);
    if (!s_enabled) {
        force_output_off();
        return;
    }
    (void)bldc_pwm_set_enable(true);
}
