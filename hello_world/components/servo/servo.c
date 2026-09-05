#include "servo.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

static const char *TAG = "servo";

/* SG90：多数廉价舵机到不了 2500us，实测约到 160°(旧映射)后无响应。
 * 将 0~180° 映射到可动脉宽，避免滑条末端“没反应”。 */
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2270
#define SERVO_MIN_DEGREE        0
#define SERVO_MAX_DEGREE        180
#define SERVO_PULSE_GPIO        18
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD        20000
#define SERVO_SCAN_SPEED_MIN    1
#define SERVO_SCAN_SPEED_MAX    10
#define SERVO_SCAN_SPEED_DEFAULT 5

typedef struct {
    int step_deg;
    int step_ms;
} scan_profile_t;

/* 档位越高越快：先缩短间隔，再加大步距；最短约 14ms，避免 SG90 跟不上 */
static const scan_profile_t s_speed_table[SERVO_SCAN_SPEED_MAX] = {
    {1, 45}, /* 1 */
    {1, 35},
    {1, 28},
    {1, 22},
    {1, 18}, /* 5 */
    {2, 18},
    {2, 15},
    {3, 15},
    {4, 14},
    {5, 14}, /* 10 */
};

static mcpwm_cmpr_handle_t s_comparator;
static int s_angle = 90;
static volatile bool s_scan_enabled;
static volatile int s_scan_speed = SERVO_SCAN_SPEED_DEFAULT;
static TaskHandle_t s_scan_task;

static inline uint32_t angle_to_compare(int angle)
{
    return (uint32_t)((angle - SERVO_MIN_DEGREE) *
                      (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
                      (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
                      SERVO_MIN_PULSEWIDTH_US);
}

static inline const scan_profile_t *current_profile(void)
{
    int level = s_scan_speed;
    if (level < SERVO_SCAN_SPEED_MIN) {
        level = SERVO_SCAN_SPEED_MIN;
    } else if (level > SERVO_SCAN_SPEED_MAX) {
        level = SERVO_SCAN_SPEED_MAX;
    }
    return &s_speed_table[level - 1];
}

static esp_err_t set_angle_raw(int angle_deg)
{
    if (angle_deg < SERVO_MIN_DEGREE) {
        angle_deg = SERVO_MIN_DEGREE;
    } else if (angle_deg > SERVO_MAX_DEGREE) {
        angle_deg = SERVO_MAX_DEGREE;
    }

    esp_err_t err = mcpwm_comparator_set_compare_value(s_comparator, angle_to_compare(angle_deg));
    if (err != ESP_OK) {
        return err;
    }
    s_angle = angle_deg;
    return ESP_OK;
}

static void scan_task(void *arg)
{
    (void)arg;
    int angle = s_angle;
    int dir = 1;

    while (true) {
        if (!s_scan_enabled) {
            vTaskDelay(pdMS_TO_TICKS(50));
            angle = s_angle;
            dir = 1;
            continue;
        }

        const scan_profile_t *prof = current_profile();
        angle += dir * prof->step_deg;

        if (angle >= SERVO_MAX_DEGREE) {
            angle = SERVO_MAX_DEGREE;
            dir = -1;
        } else if (angle <= SERVO_MIN_DEGREE) {
            angle = SERVO_MIN_DEGREE;
            dir = 1;
        }

        /* 停止命令可能在上次延时期间到达，避免再覆盖一次目标角度 */
        if (!s_scan_enabled) {
            continue;
        }

        set_angle_raw(angle);
        vTaskDelay(pdMS_TO_TICKS(prof->step_ms));
    }
}

esp_err_t servo_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &s_comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_ERROR_CHECK(set_angle_raw(s_angle));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparator,
                                       MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    BaseType_t ok = xTaskCreate(scan_task, "servo_scan", 2048, NULL, 5, &s_scan_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create scan task failed");
        return ESP_ERR_NO_MEM;
    }

    const scan_profile_t *prof = current_profile();
    ESP_LOGI(TAG, "SG90 ready on GPIO%d, angle=%d, speed=%d (%ddeg/%dms)",
             SERVO_PULSE_GPIO, s_angle, s_scan_speed, prof->step_deg, prof->step_ms);
    return ESP_OK;
}

esp_err_t servo_set_angle(int angle_deg)
{
    servo_scan_stop();
    esp_err_t err = set_angle_raw(angle_deg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "angle -> %d", s_angle);
    }
    return err;
}

int servo_get_angle(void)
{
    return s_angle;
}

esp_err_t servo_scan_start(void)
{
    if (s_scan_enabled) {
        return ESP_OK;
    }
    s_scan_enabled = true;
    const scan_profile_t *prof = current_profile();
    ESP_LOGI(TAG, "scan start, speed=%d (%ddeg/%dms)",
             s_scan_speed, prof->step_deg, prof->step_ms);
    return ESP_OK;
}

void servo_scan_stop(void)
{
    if (!s_scan_enabled) {
        return;
    }
    s_scan_enabled = false;
    ESP_LOGI(TAG, "scan stop, angle=%d", s_angle);
}

bool servo_is_scanning(void)
{
    return s_scan_enabled;
}

esp_err_t servo_set_scan_speed(int level)
{
    if (level < SERVO_SCAN_SPEED_MIN) {
        level = SERVO_SCAN_SPEED_MIN;
    } else if (level > SERVO_SCAN_SPEED_MAX) {
        level = SERVO_SCAN_SPEED_MAX;
    }
    s_scan_speed = level;
    const scan_profile_t *prof = current_profile();
    ESP_LOGI(TAG, "scan speed -> %d (%ddeg/%dms)",
             s_scan_speed, prof->step_deg, prof->step_ms);
    return ESP_OK;
}

int servo_get_scan_speed(void)
{
    return s_scan_speed;
}
