#include "app_profile.h"
#include "sdkconfig.h"

void app_profile_make(servo_cycle_config_t *motion,
                      servo_pulse_range_t limits[SERVO_CHANNEL_COUNT])
{
    static const uint32_t upper_us[] = {1000, 1500, 2000, 2500};
    const uint32_t lower_us = 600;
    _Static_assert(APP_SERVO_CHANNEL_COUNT == SERVO_CHANNEL_COUNT, "Channel count mismatch");
    *motion = (servo_cycle_config_t){
        .motion_us = INT64_C(1000) * CONFIG_SERVO_MOTION_TIME_MS,
        .hold_us = INT64_C(1000) * CONFIG_SERVO_HOLD_TIME_MS,
        .update_us = 20000,
    };
    for (unsigned i = 0; i < SERVO_CHANNEL_COUNT; ++i) {
        /* Preserve current per-channel limits; future calibration belongs here. */
        limits[i] = (servo_pulse_range_t){lower_us, upper_us[i % 4]};
        motion->start_us[i] = lower_us;
        motion->end_us[i] = upper_us[i % 4];
    }
}
