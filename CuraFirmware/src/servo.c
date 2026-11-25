/*
Today Targets
- Write servo driver
- test servo control code to ensure it meets timing requirements
- develop control code to synchronize servo position with lidar data 
- Get it all working from start to finish. (servo needs to calibrate itself)
*/

/*
Red = Battery(+) 
Brown = Battery(-) 
Orange = Signal (PWM)
*/

#include "servo.h"
#include "esp_log.h"

static const char *TAG = "sg90";

#define SG90_FREQ_HZ        50                      /* 20 ms period */
#define SG90_PERIOD_US      (1000000 / SG90_FREQ_HZ)
#define SG90_DUTY_RES       LEDC_TIMER_14_BIT
#define SG90_DUTY_MAX       ((1U << 14) - 1)
#define SG90_SPEED_MODE     LEDC_LOW_SPEED_MODE

#define SG90_ABS_MIN_US     500
#define SG90_ABS_MAX_US     2500

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    return (pulse_us * (SG90_DUTY_MAX + 1) + SG90_PERIOD_US / 2) / SG90_PERIOD_US;
}

esp_err_t sg90_init(sg90_t *servo, const sg90_config_t *cfg)
{
    if (!servo || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    servo->cfg = *cfg;
    if (servo->cfg.min_pulse_us == 0) servo->cfg.min_pulse_us = SG90_DEFAULT_MIN_PULSE_US;
    if (servo->cfg.max_pulse_us == 0) servo->cfg.max_pulse_us = SG90_DEFAULT_MAX_PULSE_US;
    if (servo->cfg.min_pulse_us >= servo->cfg.max_pulse_us) {
        return ESP_ERR_INVALID_ARG;
    }

    ledc_timer_config_t timer = {
        .speed_mode      = SG90_SPEED_MODE,
        .duty_resolution = SG90_DUTY_RES,
        .timer_num       = servo->cfg.timer,
        .freq_hz         = SG90_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch = {
        .gpio_num   = servo->cfg.gpio,
        .speed_mode = SG90_SPEED_MODE,
        .channel    = servo->cfg.channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = servo->cfg.timer,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    servo->initialized = true;
    return sg90_set_angle(servo, 0);
}

esp_err_t sg90_set_pulse_us(sg90_t *servo, uint32_t pulse_us)
{
    if (!servo || !servo->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (pulse_us < SG90_ABS_MIN_US) pulse_us = SG90_ABS_MIN_US;
    if (pulse_us > SG90_ABS_MAX_US) pulse_us = SG90_ABS_MAX_US;

    esp_err_t err = ledc_set_duty(SG90_SPEED_MODE, servo->cfg.channel, pulse_to_duty(pulse_us));
    if (err != ESP_OK) return err;
    return ledc_update_duty(SG90_SPEED_MODE, servo->cfg.channel);
}

esp_err_t sg90_set_angle(sg90_t *servo, int angle)
{
    if (!servo || !servo->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (angle < SG90_ANGLE_MIN) angle = SG90_ANGLE_MIN;
    if (angle > SG90_ANGLE_MAX) angle = SG90_ANGLE_MAX;

    /* Linear map: -90 -> min_pulse, 0 -> center, +90 -> max_pulse */
    int32_t span = (int32_t)servo->cfg.max_pulse_us - servo->cfg.min_pulse_us;
    uint32_t pulse = servo->cfg.min_pulse_us +
                     (uint32_t)(((angle - SG90_ANGLE_MIN) * span) / (SG90_ANGLE_MAX - SG90_ANGLE_MIN));

    esp_err_t err = sg90_set_pulse_us(servo, pulse);
    if (err == ESP_OK) {
        servo->angle = angle;
    }
    return err;
}

int sg90_get_angle(const sg90_t *servo)
{
    return servo ? servo->angle : 0;
}

esp_err_t sg90_release(sg90_t *servo)
{
    if (!servo || !servo->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return ledc_stop(SG90_SPEED_MODE, servo->cfg.channel, 0);
}

/* ======================================================================
 * EXAMPLE USAGE (put this in your main.c)
 * ======================================================================
 *
 * #include "freertos/FreeRTOS.h"
 * #include "freertos/task.h"
 * #include "sg90.h"
 *
 * void app_main(void)
 * {
 *     sg90_t servo;
 *     sg90_config_t cfg = {
 *         .gpio    = GPIO_NUM_18,
 *         .channel = LEDC_CHANNEL_0,
 *         .timer   = LEDC_TIMER_0,
 *         // .min_pulse_us = 500,    // optional: widen range for full travel
 *         // .max_pulse_us = 2400,   // (defaults are 1000 / 2000 us)
 *     };
 *
 *     ESP_ERROR_CHECK(sg90_init(&servo, &cfg));   // starts at 0 (center)
 *     vTaskDelay(pdMS_TO_TICKS(1000));
 *
 *     sg90_set_angle(&servo, -90);                // all the way left
 *     vTaskDelay(pdMS_TO_TICKS(1000));
 *
 *     sg90_set_angle(&servo, 90);                 // all the way right
 *     vTaskDelay(pdMS_TO_TICKS(1000));
 *
 *     sg90_set_angle(&servo, 0);                  // back to center
 *     vTaskDelay(pdMS_TO_TICKS(1000));
 *
 *     // Smooth sweep
 *     for (int a = -90; a <= 90; a += 2) {
 *         sg90_set_angle(&servo, a);
 *         vTaskDelay(pdMS_TO_TICKS(20));
 *     }
 *
 *     sg90_release(&servo);                       // stop pulses, servo goes limp
 * }
 *
 * Required in src/CMakeLists.txt:
 *     idf_component_register(SRCS "main.c" "sg90.c" INCLUDE_DIRS "." REQUIRES driver)
 */