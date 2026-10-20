#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define APP_SERVO_CHANNEL_COUNT 16

/* Business boundary: pulse widths and monotonic time only, no PWM/I2C/RTOS types.
 * The application owns the referenced context for the entire scheduler lifetime.
 */
typedef struct {
    void *context;
    esp_err_t (*validate_pulses)(void *context, const uint32_t pulses[APP_SERVO_CHANNEL_COUNT]);
    esp_err_t (*write_pulses)(void *context, const uint32_t pulses[APP_SERVO_CHANNEL_COUNT]);
    esp_err_t (*all_off)(void *context);
    int64_t (*now_us)(void *context);
} app_servo_port_t;

static inline bool app_servo_port_valid(const app_servo_port_t *port)
{
    return port != NULL && port->validate_pulses != NULL && port->write_pulses != NULL &&
           port->all_off != NULL && port->now_us != NULL;
}
