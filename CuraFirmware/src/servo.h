#ifndef SG90_H
#define SG90_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SG90_DEFAULT_MIN_PULSE_US   1000   /* -90 deg (all the way left)  */
#define SG90_DEFAULT_MAX_PULSE_US   2000   /* +90 deg (all the way right) */

#define SG90_ANGLE_MIN  (-90)
#define SG90_ANGLE_MAX  ( 90)

typedef struct {
    gpio_num_t     gpio;          /* PWM output pin                        */
    ledc_channel_t channel;       /* LEDC channel, e.g. LEDC_CHANNEL_0     */
    ledc_timer_t   timer;         /* LEDC timer,   e.g. LEDC_TIMER_0       */
    uint16_t       min_pulse_us;  /* pulse for -90 deg, 0 = default        */
    uint16_t       max_pulse_us;  /* pulse for +90 deg, 0 = default        */
} sg90_config_t;

typedef struct {
    sg90_config_t cfg;
    int           angle;
    bool          initialized;
} sg90_t;

esp_err_t sg90_init(sg90_t *servo, const sg90_config_t *cfg);
esp_err_t sg90_set_angle(sg90_t *servo, int angle);
esp_err_t sg90_set_pulse_us(sg90_t *servo, uint32_t pulse_us);
int       sg90_get_angle(const sg90_t *servo);
esp_err_t sg90_release(sg90_t *servo);

#ifdef __cplusplus
}
#endif

#endif /* SG90_H */