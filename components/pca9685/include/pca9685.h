#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_CHANNEL_COUNT 16
#define PCA9685_PWM_TICKS 4096

typedef struct pca9685 *pca9685_handle_t;

typedef struct {
    uint16_t address;              /**< Unshifted 7-bit hardware address. */
    uint32_t i2c_speed_hz;         /**< 100000 or up to 400000 on ESP32-S3. */
    uint32_t oscillator_hz;        /**< Calibrated internal oscillator; nominal 25000000. */
    float frequency_hz;            /**< Shared by all 16 channels. */
    int timeout_ms;                /**< Positive timeout per transfer and mutex acquisition. */
} pca9685_config_t;

#define PCA9685_CONFIG_DEFAULT() { \
    .address = 0x40, .i2c_speed_hz = 100000, .oscillator_hz = 25000000, \
    .frequency_hz = 50.0f, .timeout_ms = 100 \
}

typedef struct {
    uint16_t on_tick;              /**< 0..4095; supports a phase offset. */
    uint16_t off_tick;             /**< 0..4095; equal to on_tick means full OFF. */
} pca9685_pwm_t;

/**
 * Add and initialize one chip on a caller-owned, synchronous I2C bus.
 * Starts with every channel full OFF, auto-increment enabled, non-inverted
 * totem-pole outputs, update-on-STOP, and All Call/Sub Call addresses disabled.
 * Supports the internal oscillator only; EXTCLK already set returns INVALID_STATE.
 * On failure *out_handle is NULL. Does not delete the caller's bus.
 */
esp_err_t pca9685_create(i2c_master_bus_handle_t bus, const pca9685_config_t *config,
                         pca9685_handle_t *out_handle);

/**
 * Disable outputs and release this device. Caller must stop all concurrent users.
 * On failure the handle remains owned by the caller and deletion can be retried.
 * Successful deletion invalidates the handle; the caller's bus remains alive.
 */
esp_err_t pca9685_delete(pca9685_handle_t handle);

/**
 * Set shared frequency using the nearest representable prescaler (3..255).
 * ALWAYS clears every channel to full OFF; callers must reapply pulse commands.
 * An I2C failure during the change invalidates pulse/PWM writes until a successful
 * retry. Invalid arguments or a mutex timeout leave the previous state unchanged.
 */
esp_err_t pca9685_set_frequency(pca9685_handle_t handle, float frequency_hz);

/** Estimated realized frequency from the configured oscillator and prescaler. */
esp_err_t pca9685_get_frequency(pca9685_handle_t handle, float *frequency_hz);

/** Check electrical representability using the same conversion as writes.
 * No register I/O. Accepts zero as OFF; requires valid frequency; 1..16 entries.
 */
esp_err_t pca9685_validate_pulse_widths_us(pca9685_handle_t handle,
                                          const uint32_t *pulse_widths_us, size_t count);

esp_err_t pca9685_set_pwm(pca9685_handle_t handle, uint8_t channel,
                          uint16_t on_tick, uint16_t off_tick);

/** One contiguous I2C transfer, with outputs updated on STOP. */
esp_err_t pca9685_set_channels(pca9685_handle_t handle, uint8_t first_channel,
                               const pca9685_pwm_t *values, size_t count);

/**
 * Convert microseconds using the realized prescaler, with phase starting at zero.
 * Zero disables the channel. Rejects pulses rounding to zero or >= one period.
 * This is an electrical pulse interface; servo motion limits belong above it.
 */
esp_err_t pca9685_set_pulse_width_us(pca9685_handle_t handle, uint8_t channel,
                                    uint32_t pulse_width_us);
/** Convert and update contiguous pulse widths in one transfer, under one lock.
 * Zero entries mean full OFF. Requires a valid frequency, even for all-zero data.
 * Invalid entries reject the entire batch before any register write.
 */
esp_err_t pca9685_set_pulse_widths_us(pca9685_handle_t handle, uint8_t first_channel,
                                     const uint32_t *pulse_widths_us, size_t count);
esp_err_t pca9685_disable_channel(pca9685_handle_t handle, uint8_t channel);
esp_err_t pca9685_all_off(pca9685_handle_t handle);

/* Task-context only. Operations on one handle are mutex-protected. Own each chip
 * through one handle; do not access its registers outside this driver. Timeouts
 * are per operation, not an end-to-end deadline. I2C errors cannot guarantee OFF;
 * use a separately controlled hardware OE line if that guarantee is required.
 */

#ifdef __cplusplus
}
#endif
