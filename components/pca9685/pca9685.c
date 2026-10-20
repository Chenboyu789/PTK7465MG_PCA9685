#include "pca9685.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define REG_MODE1         0x00
#define REG_MODE2         0x01
#define REG_LED0_ON_L     0x06
#define REG_ALL_LED_ON_L  0xFA
#define REG_ALL_LED_OFF_H 0xFD
#define REG_PRE_SCALE     0xFE
#define MODE1_SLEEP      0x10
#define MODE1_AI         0x20
#define MODE1_EXTCLK     0x40
#define MODE2_OUTDRV     0x04
#define FULL_OFF         0x10

static const char *TAG = "pca9685";

struct pca9685 {
    i2c_master_dev_handle_t device;
    SemaphoreHandle_t mutex;
    uint32_t oscillator_hz;
    int timeout_ms;
    uint8_t prescale;
    bool frequency_valid;
};

static esp_err_t lock_device(pca9685_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    TickType_t ticks = pdMS_TO_TICKS(handle->timeout_ms);
    return xSemaphoreTake(handle->mutex, ticks > 0 ? ticks : 1) == pdTRUE
               ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t write_reg(pca9685_handle_t handle, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(handle->device, data, sizeof(data), handle->timeout_ms);
}

static esp_err_t read_reg(pca9685_handle_t handle, uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(handle->device, &reg, 1, value, 1,
                                       handle->timeout_ms);
}

static esp_err_t calculate_prescale(uint32_t oscillator_hz, float hz, uint8_t *prescale)
{
    if (!isfinite(hz) || hz <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    double divisor = (double)oscillator_hz / (PCA9685_PWM_TICKS * (double)hz);
    if (divisor < 4.0 || divisor > 256.0) {
        return ESP_ERR_INVALID_ARG;
    }
    *prescale = (uint8_t)((unsigned)(divisor + 0.5) - 1);
    return ESP_OK;
}

static esp_err_t all_off_locked(pca9685_handle_t handle)
{
    /* A single-register write works even before auto-increment is configured. */
    return write_reg(handle, REG_ALL_LED_OFF_H, FULL_OFF);
}

static esp_err_t set_frequency_locked(pca9685_handle_t handle, uint8_t prescale)
{
    handle->frequency_valid = false;
    esp_err_t err = all_off_locked(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial all-OFF write failed: %s", esp_err_to_name(err));
        return err;
    }
    uint8_t mode;
    err = read_reg(handle, REG_MODE1, &mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MODE1 read failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "MODE1 before configuration: 0x%02X", (unsigned)mode);
    /* EXTCLK is sticky and cannot be cleared by simply writing MODE1. */
    if (mode & MODE1_EXTCLK) {
        ESP_LOGE(TAG, "EXTCLK is set; internal-clock initialization rejected. "
                 "Power-cycle PCA9685 VCC or perform a software reset.");
        return ESP_ERR_INVALID_STATE;
    }
    err = write_reg(handle, REG_MODE1, MODE1_AI | MODE1_SLEEP);
    if (err == ESP_OK) {
        err = write_reg(handle, REG_MODE2, MODE2_OUTDRV);
    }
    if (err == ESP_OK) {
        /* Clear stale full-ON bits and phase counts on a previously used chip. */
        const uint8_t off[] = {REG_ALL_LED_ON_L, 0, 0, 0, FULL_OFF};
        err = i2c_master_transmit(handle->device, off, sizeof(off), handle->timeout_ms);
    }
    if (err == ESP_OK) {
        err = write_reg(handle, REG_PRE_SCALE, prescale);
    }
    if (err == ESP_OK) {
        err = write_reg(handle, REG_MODE1, MODE1_AI);
    }
    if (err != ESP_OK) {
        (void)all_off_locked(handle);
        return err;
    }
    /* All channels were explicitly stopped: no RESTART of old pulses is needed. */
    esp_rom_delay_us(500);
    uint8_t actual_prescale;
    err = read_reg(handle, REG_PRE_SCALE, &actual_prescale);
    if (err == ESP_OK && actual_prescale != prescale) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    if (err == ESP_OK) {
        err = read_reg(handle, REG_MODE1, &mode);
        if (err == ESP_OK && (mode & (MODE1_SLEEP | MODE1_EXTCLK | MODE1_AI)) != MODE1_AI) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (err == ESP_OK) {
        handle->prescale = prescale;
        handle->frequency_valid = true;
    } else {
        (void)all_off_locked(handle);
    }
    return err;
}

esp_err_t pca9685_create(i2c_master_bus_handle_t bus, const pca9685_config_t *config,
                         pca9685_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;
    if (bus == NULL || config == NULL || config->address < 0x08 || config->address > 0x77 ||
        config->i2c_speed_hz == 0 || config->i2c_speed_hz > 400000 ||
        config->oscillator_hz == 0 || config->oscillator_hz > 50000000 || config->timeout_ms <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t prescale;
    esp_err_t err = calculate_prescale(config->oscillator_hz, config->frequency_hz, &prescale);
    if (err != ESP_OK) {
        return err;
    }
    pca9685_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }
    handle->mutex = xSemaphoreCreateMutex();
    if (handle->mutex == NULL) {
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    handle->oscillator_hz = config->oscillator_hz;
    handle->timeout_ms = config->timeout_ms;
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->address,
        .scl_speed_hz = config->i2c_speed_hz,
    };
    err = i2c_master_bus_add_device(bus, &device_config, &handle->device);
    if (err == ESP_OK) {
        err = set_frequency_locked(handle, prescale);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Frequency initialization failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        if (handle->device != NULL) {
            (void)all_off_locked(handle);
            (void)i2c_master_bus_rm_device(handle->device);
        }
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return err;
    }
    *out_handle = handle;
    return ESP_OK;
}

esp_err_t pca9685_set_frequency(pca9685_handle_t handle, float frequency_hz)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t prescale;
    esp_err_t err = calculate_prescale(handle->oscillator_hz, frequency_hz, &prescale);
    if (err != ESP_OK) {
        return err;
    }
    err = lock_device(handle);
    if (err == ESP_OK) {
        err = set_frequency_locked(handle, prescale);
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_get_frequency(pca9685_handle_t handle, float *frequency_hz)
{
    if (frequency_hz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        if (handle->frequency_valid) {
            *frequency_hz = (float)((double)handle->oscillator_hz /
                                  (PCA9685_PWM_TICKS * (handle->prescale + 1.0)));
        } else {
            err = ESP_ERR_INVALID_STATE;
        }
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

static esp_err_t write_channels_locked(pca9685_handle_t handle, uint8_t first,
                                        const pca9685_pwm_t *values, size_t count)
{
    if (!handle->frequency_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t data[1 + 4 * PCA9685_CHANNEL_COUNT];
    data[0] = REG_LED0_ON_L + 4 * first;
    for (size_t i = 0; i < count; ++i) {
        data[1 + 4 * i] = values[i].on_tick & 0xff;
        data[2 + 4 * i] = values[i].on_tick >> 8;
        data[3 + 4 * i] = values[i].off_tick & 0xff;
        data[4 + 4 * i] = values[i].off_tick >> 8;
        if (values[i].on_tick == values[i].off_tick) {
            data[4 + 4 * i] |= FULL_OFF;
        }
    }
    return i2c_master_transmit(handle->device, data, 1 + 4 * count, handle->timeout_ms);
}

esp_err_t pca9685_set_channels(pca9685_handle_t handle, uint8_t first_channel,
                               const pca9685_pwm_t *values, size_t count)
{
    if (first_channel >= PCA9685_CHANNEL_COUNT || values == NULL || count == 0 ||
        count > (size_t)(PCA9685_CHANNEL_COUNT - first_channel)) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; ++i) {
        if (values[i].on_tick >= PCA9685_PWM_TICKS || values[i].off_tick >= PCA9685_PWM_TICKS) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        err = write_channels_locked(handle, first_channel, values, count);
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_set_pwm(pca9685_handle_t handle, uint8_t channel,
                          uint16_t on_tick, uint16_t off_tick)
{
    const pca9685_pwm_t value = {.on_tick = on_tick, .off_tick = off_tick};
    return pca9685_set_channels(handle, channel, &value, 1);
}

esp_err_t pca9685_set_pulse_width_us(pca9685_handle_t handle, uint8_t channel,
                                    uint32_t pulse_width_us)
{
    if (channel >= PCA9685_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pulse_width_us == 0) {
        return pca9685_disable_channel(handle, channel);
    }
    return pca9685_set_pulse_widths_us(handle, channel, &pulse_width_us, 1);
}

static esp_err_t convert_pulses_locked(pca9685_handle_t handle, const uint32_t *pulses,
                                       size_t count, pca9685_pwm_t *values)
{
    if (!handle->frequency_valid) { return ESP_ERR_INVALID_STATE; }
    uint64_t denominator = (uint64_t)(handle->prescale + 1) * 1000000;
    for (size_t i = 0; i < count; ++i) {
        uint64_t ticks = 0;
        if (pulses[i] != 0) {
            ticks = ((uint64_t)pulses[i] * handle->oscillator_hz + denominator / 2) / denominator;
            if (ticks == 0 || ticks >= PCA9685_PWM_TICKS) { return ESP_ERR_INVALID_ARG; }
        }
        if (values != NULL) {
            values[i] = (pca9685_pwm_t){.on_tick = 0, .off_tick = (uint16_t)ticks};
        }
    }
    return ESP_OK;
}

esp_err_t pca9685_validate_pulse_widths_us(pca9685_handle_t handle,
                                          const uint32_t *pulses, size_t count)
{
    if (pulses == NULL || count == 0 || count > PCA9685_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        err = convert_pulses_locked(handle, pulses, count, NULL);
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_set_pulse_widths_us(pca9685_handle_t handle, uint8_t first_channel,
                                     const uint32_t *pulse_widths_us, size_t count)
{
    if (first_channel >= PCA9685_CHANNEL_COUNT || pulse_widths_us == NULL || count == 0 ||
        count > (size_t)(PCA9685_CHANNEL_COUNT - first_channel)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        pca9685_pwm_t values[PCA9685_CHANNEL_COUNT];
        err = convert_pulses_locked(handle, pulse_widths_us, count, values);
        if (err == ESP_OK) {
            err = write_channels_locked(handle, first_channel, values, count);
        }
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_disable_channel(pca9685_handle_t handle, uint8_t channel)
{
    if (channel >= PCA9685_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        /* Do not depend on known frequency or auto-increment for shutdown. */
        err = write_reg(handle, REG_LED0_ON_L + 4 * channel + 3, FULL_OFF);
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_all_off(pca9685_handle_t handle)
{
    esp_err_t err = lock_device(handle);
    if (err == ESP_OK) {
        err = all_off_locked(handle);
        xSemaphoreGive(handle->mutex);
    }
    return err;
}

esp_err_t pca9685_delete(pca9685_handle_t handle)
{
    esp_err_t err = lock_device(handle);
    if (err != ESP_OK) {
        return err;
    }
    err = all_off_locked(handle);
    if (err == ESP_OK) {
        err = i2c_master_bus_rm_device(handle->device);
    }
    xSemaphoreGive(handle->mutex);
    if (err == ESP_OK) {
        vSemaphoreDelete(handle->mutex);
        free(handle);
    }
    return err;
}
