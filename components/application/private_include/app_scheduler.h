#pragma once

#include "app.h"
#include "business_servo_cycle.h"

/* Only app.c's owner task may call this scheduler. Other tasks submit requests
 * through app_select_business(), never directly through these private headers.
 */
typedef struct {
    app_servo_port_t port;
    servo_cycle_config_t config;
    business_servo_cycle_t servo_cycle;
    app_business_id_t active;
    esp_err_t off_error;
    bool initialized;
} app_scheduler_t;

esp_err_t app_scheduler_init(app_scheduler_t *scheduler, const app_servo_port_t *port,
                              const servo_cycle_config_t *config);
esp_err_t app_scheduler_select(app_scheduler_t *scheduler, app_business_id_t business);
esp_err_t app_scheduler_process(app_scheduler_t *scheduler);
