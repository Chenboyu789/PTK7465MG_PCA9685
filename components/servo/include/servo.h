#pragma once

#include "pca9685.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_CHANNEL_COUNT PCA9685_CHANNEL_COUNT

typedef struct {
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
} servo_pulse_range_t;

typedef struct {
    pca9685_handle_t pwm; /* Borrowed; must outlive the bank. */
    servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT];
} servo_bank_t;

/* Copies ranges without issuing motion. Pass a zero-initialized bank. */
esp_err_t servo_bank_init(servo_bank_t *bank, pca9685_handle_t pwm,
                          const servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT]);
esp_err_t servo_bank_set_pulse(servo_bank_t *bank, uint8_t channel, uint32_t pulse_us);
esp_err_t servo_bank_validate_pulses(const servo_bank_t *bank,
                                     const uint32_t pulses[SERVO_CHANNEL_COUNT]);
/* Validates every pulse before writing all 16 channels in one transfer. */
esp_err_t servo_bank_set_pulses(servo_bank_t *bank,
                                const uint32_t pulses[SERVO_CHANNEL_COUNT]);
esp_err_t servo_bank_all_off(servo_bank_t *bank);

#ifdef __cplusplus
}
#endif
