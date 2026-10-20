#pragma once

#include "app_servo_port.h"
#include "app.h"

typedef struct {
    uint32_t start_us[APP_SERVO_CHANNEL_COUNT];
    uint32_t end_us[APP_SERVO_CHANNEL_COUNT];
    int64_t motion_us;
    int64_t hold_us;
    int64_t update_us;
} servo_cycle_config_t;

/* Private business state. Lifecycle calls are serialized by app_scheduler. */
typedef struct {
    app_servo_port_t port;
    servo_cycle_config_t config;
    int64_t next_command_us;
    int64_t motion_start_us;
    app_motion_state_t state;
} business_servo_cycle_t;

esp_err_t business_servo_cycle_start(business_servo_cycle_t *cycle, const app_servo_port_t *port,
                                     const servo_cycle_config_t *config);
esp_err_t business_servo_cycle_process(business_servo_cycle_t *cycle);
/* Relinquishes logical control; scheduler is responsible for physical all-OFF. */
void business_servo_cycle_stop(business_servo_cycle_t *cycle);
