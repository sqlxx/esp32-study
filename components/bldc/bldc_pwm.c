#include "bldc_pwm.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char *TAG = "bldc_pwm";

/* group 1，避开舵机占用的 group 0 / GPIO18 */
#define BLDC_MCPWM_GROUP           1
#define BLDC_PIN_U                 25
#define BLDC_PIN_V                 26
#define BLDC_PIN_W                 32
#define BLDC_PIN_EN                33 /* 驱动 EN，没有则保持 -1 */
#define BLDC_PWM_RESOLUTION_HZ     10000000
#define BLDC_PWM_PERIOD_TICKS      500 /* 20kHz */
#define BLDC_PHASE_COUNT           3

static const int s_pins[BLDC_PHASE_COUNT] = {
    BLDC_PIN_U,
    BLDC_PIN_V,
    BLDC_PIN_W,
};

static mcpwm_cmpr_handle_t s_cmpr[BLDC_PHASE_COUNT];
static bool s_ready;

static uint32_t duty_to_ticks(float duty)
{
    if (duty <= 0.0f) {
        return 0;
    }
    if (duty >= 1.0f) {
        return BLDC_PWM_PERIOD_TICKS;
    }
    return (uint32_t)(duty * (float)BLDC_PWM_PERIOD_TICKS + 0.5f);
}

esp_err_t bldc_pwm_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = BLDC_MCPWM_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BLDC_PWM_RESOLUTION_HZ,
        .period_ticks = BLDC_PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    for (int i = 0; i < BLDC_PHASE_COUNT; i++) {
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t operator_config = {
            .group_id = BLDC_MCPWM_GROUP,
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &s_cmpr[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmpr[i], 0));

        mcpwm_gen_handle_t gen = NULL;
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = s_pins[i],
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &gen));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_cmpr[i],
                                           MCPWM_GEN_ACTION_LOW)));
    }

#if BLDC_PIN_EN >= 0
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BLDC_PIN_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)BLDC_PIN_EN, 0));
#endif

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    s_ready = true;
    ESP_LOGI(TAG, "3PWM ready group=%d U=%d V=%d W=%d EN=%d, %dHz",
             BLDC_MCPWM_GROUP, BLDC_PIN_U, BLDC_PIN_V, BLDC_PIN_W, BLDC_PIN_EN,
             BLDC_PWM_RESOLUTION_HZ / BLDC_PWM_PERIOD_TICKS);
    return ESP_OK;
}

esp_err_t bldc_pwm_set_duty(float u, float v, float w)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const float duty[BLDC_PHASE_COUNT] = {u, v, w};
    for (int i = 0; i < BLDC_PHASE_COUNT; i++) {
        esp_err_t err = mcpwm_comparator_set_compare_value(s_cmpr[i], duty_to_ticks(duty[i]));
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t bldc_pwm_set_enable(bool on)
{
#if BLDC_PIN_EN < 0
    (void)on;
    return ESP_OK;
#else
    return gpio_set_level((gpio_num_t)BLDC_PIN_EN, on ? 1 : 0);
#endif
}
