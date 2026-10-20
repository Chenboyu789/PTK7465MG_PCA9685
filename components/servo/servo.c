#include "servo.h"

#include <string.h>

esp_err_t servo_bank_init(servo_bank_t *bank, pca9685_handle_t pwm,
                          const servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT])
{
    if (bank == NULL || pwm == NULL || ranges == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bank->pwm != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t low[SERVO_CHANNEL_COUNT], high[SERVO_CHANNEL_COUNT];
    for (size_t i = 0; i < SERVO_CHANNEL_COUNT; ++i) {
        if (ranges[i].min_pulse_us == 0 ||
            ranges[i].min_pulse_us >= ranges[i].max_pulse_us) {
            return ESP_ERR_INVALID_ARG;
        }
        low[i] = ranges[i].min_pulse_us;
        high[i] = ranges[i].max_pulse_us;
    }
    esp_err_t err = pca9685_validate_pulse_widths_us(pwm, low, SERVO_CHANNEL_COUNT);
    if (err == ESP_OK) { err = pca9685_validate_pulse_widths_us(pwm, high, SERVO_CHANNEL_COUNT); }
    if (err != ESP_OK) { return err; }
    memcpy(bank->ranges, ranges, sizeof(bank->ranges));
    bank->pwm = pwm;
    return ESP_OK;
}

esp_err_t servo_bank_validate_pulses(const servo_bank_t *bank,
                                     const uint32_t pulses[SERVO_CHANNEL_COUNT])
{
    if (bank == NULL || bank->pwm == NULL || pulses == NULL) { return ESP_ERR_INVALID_ARG; }
    for (size_t i = 0; i < SERVO_CHANNEL_COUNT; ++i) {
        if (pulses[i] < bank->ranges[i].min_pulse_us ||
            pulses[i] > bank->ranges[i].max_pulse_us) { return ESP_ERR_INVALID_ARG; }
    }
    return ESP_OK;
}

esp_err_t servo_bank_set_pulse(servo_bank_t *bank, uint8_t channel, uint32_t pulse_us)
{
    if (bank == NULL || bank->pwm == NULL || channel >= SERVO_CHANNEL_COUNT ||
        pulse_us < bank->ranges[channel].min_pulse_us ||
        pulse_us > bank->ranges[channel].max_pulse_us) { return ESP_ERR_INVALID_ARG; }
    return pca9685_set_pulse_width_us(bank->pwm, channel, pulse_us);
}

esp_err_t servo_bank_set_pulses(servo_bank_t *bank,
                                const uint32_t pulses[SERVO_CHANNEL_COUNT])
{
    esp_err_t err = servo_bank_validate_pulses(bank, pulses);
    return err == ESP_OK ? pca9685_set_pulse_widths_us(bank->pwm, 0, pulses, SERVO_CHANNEL_COUNT) : err;
}

esp_err_t servo_bank_all_off(servo_bank_t *bank)
{
    if (bank == NULL || bank->pwm == NULL) { return ESP_ERR_INVALID_ARG; }
    return pca9685_all_off(bank->pwm);
}
