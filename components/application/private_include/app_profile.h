#pragma once

#include "business_servo_cycle.h"
#include "servo.h"

/* Composition only: allowed output limits and requested motion are distinct. */
void app_profile_make(servo_cycle_config_t *motion,
                      servo_pulse_range_t limits[SERVO_CHANNEL_COUNT]);
