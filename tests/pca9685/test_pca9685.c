/* Compile the production sources against a register-level I2C fake.
 * This checks software/protocol behavior, not physical I2C timing or PWM signals.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "freertos/semphr.h"

static uint8_t registers[256];
static uint8_t last_write[65];
static size_t last_size;
static int transfer_count, fail_transfer, devices, buses, delayed_us;
static bool fail_lock, corrupt_prescale, fail_remove;
static i2c_device_config_t device_config;
static i2c_master_bus_config_t bus_config;

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return calloc(1, sizeof(int)); }
int xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks)
{
    assert(ticks > 0 && *(int *)sem == 0);
    if (fail_lock) { return 0; }
    *(int *)sem = 1;
    return pdTRUE;
}
int xSemaphoreGive(SemaphoreHandle_t sem)
{
    assert(*(int *)sem == 1);
    *(int *)sem = 0;
    return pdTRUE;
}
void vSemaphoreDelete(SemaphoreHandle_t sem) { free(sem); }
void esp_rom_delay_us(uint32_t us) { delayed_us += (int)us; }

esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *cfg, i2c_master_bus_handle_t *bus)
{
    bus_config = *cfg;
    ++buses;
    *bus = &buses;
    return ESP_OK;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus)
{
    assert(bus != NULL && devices == 0);
    --buses;
    return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t *cfg,
                                   i2c_master_dev_handle_t *device)
{
    assert(bus != NULL);
    device_config = *cfg;
    ++devices;
    *device = registers;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
    assert(device == registers);
    if (fail_remove) { return ESP_ERR_INVALID_STATE; }
    --devices;
    return ESP_OK;
}
static bool transfer_fails(void) { return ++transfer_count == fail_transfer; }
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device, const uint8_t *data,
                             size_t size, int timeout)
{
    assert(device == registers && size >= 2 && size <= sizeof(last_write) && timeout > 0);
    if (transfer_fails()) { return ESP_ERR_TIMEOUT; }
    memcpy(last_write, data, size);
    last_size = size;
    if (size > 2) { assert(registers[0] & 0x20); }
    for (size_t i = 1; i < size; ++i) {
        unsigned reg = data[0] + (unsigned)i - 1;
        if (reg == 0xfe) { assert(registers[0] & 0x10); }
        if (reg >= 0xfa && reg <= 0xfd) {
            for (int ch = 0; ch < 16; ++ch) {
                registers[6 + ch * 4 + reg - 0xfa] = data[i];
            }
        } else {
            registers[reg] = data[i];
        }
    }
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device, const uint8_t *tx,
                                     size_t tx_size, uint8_t *rx, size_t rx_size, int timeout)
{
    assert(device == registers && tx_size == 1 && rx_size == 1 && timeout > 0);
    if (transfer_fails()) { return ESP_ERR_TIMEOUT; }
    *rx = registers[*tx];
    if (corrupt_prescale && *tx == 0xfe) { *rx ^= 1; }
    return ESP_OK;
}

static void reset_fake(void)
{
    assert(devices == 0 && buses == 0);
    memset(registers, 0xff, sizeof(registers));
    registers[0] = 1; /* Power-on MODE1; stale channel values intentionally retained. */
    transfer_count = fail_transfer = delayed_us = 0;
    fail_lock = corrupt_prescale = fail_remove = false;
}
static void assert_all_off(void)
{
    for (int ch = 0; ch < 16; ++ch) { assert(registers[9 + ch * 4] & 0x10); }
}
static pca9685_handle_t create_driver(void)
{
    const pca9685_config_t config = PCA9685_CONFIG_DEFAULT();
    pca9685_handle_t handle = NULL;
    assert(pca9685_create(&buses, &config, &handle) == ESP_OK);
    assert(handle != NULL && devices == 1);
    assert(device_config.device_address == 0x40 && device_config.scl_speed_hz == 100000);
    assert(registers[0] == 0x20 && registers[1] == 4 && registers[0xfe] == 121);
    assert(delayed_us >= 500);
    assert_all_off();
    return handle;
}

static void test_outputs(void)
{
    reset_fake();
    pca9685_handle_t handle = create_driver();
    float hz;
    assert(pca9685_get_frequency(handle, &hz) == ESP_OK);
    assert(fabsf(hz - 50.02882f) < 0.001f);
    const uint32_t valid_pulses[] = {0, 500, 2500};
    uint32_t tiny = 1;
    int validation_transfers = transfer_count;
    assert(pca9685_validate_pulse_widths_us(handle, valid_pulses, 3) == ESP_OK);
    assert(pca9685_validate_pulse_widths_us(handle, &tiny, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_validate_pulse_widths_us(NULL, valid_pulses, 3) == ESP_ERR_INVALID_ARG);
    assert(pca9685_validate_pulse_widths_us(handle, NULL, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_validate_pulse_widths_us(handle, valid_pulses, 0) == ESP_ERR_INVALID_ARG);
    assert(pca9685_validate_pulse_widths_us(handle, valid_pulses, 17) == ESP_ERR_INVALID_ARG);
    fail_lock = true;
    assert(pca9685_validate_pulse_widths_us(handle, valid_pulses, 3) == ESP_ERR_TIMEOUT);
    fail_lock = false;
    assert(transfer_count == validation_transfers);
    assert(pca9685_set_pulse_width_us(handle, 0, 1500) == ESP_OK);
    assert(last_size == 5 && last_write[0] == 6);
    assert(registers[6] == 0 && registers[7] == 0 && registers[8] == 0x33 && registers[9] == 1);
    assert(pca9685_set_pulse_width_us(handle, 15, 1000) == ESP_OK);
    assert(registers[0x44] == 205 && registers[0x45] == 0);
    assert(pca9685_set_pulse_width_us(handle, 15, 2000) == ESP_OK);
    assert(registers[0x44] == 0x9a && registers[0x45] == 1);
    assert(pca9685_set_pulse_width_us(handle, 15, 0) == ESP_OK);
    assert(registers[0x45] == 0x10);
    const uint32_t pulses[] = {0, 500, 833, 1000, 1167, 1333, 2500};
    const unsigned expected_ticks[] = {0, 102, 171, 205, 239, 273, 512};
    int previous_transfers = transfer_count;
    assert(pca9685_set_pulse_widths_us(handle, 0, pulses, 7) == ESP_OK);
    assert(transfer_count == previous_transfers + 1 && last_size == 29);
    for (unsigned i = 0; i < 7; ++i) {
        assert(registers[6 + 4 * i] == 0 && registers[7 + 4 * i] == 0);
        if (i == 0) { assert(registers[9] == 0x10); }
        else { assert(registers[8 + 4 * i] + ((unsigned)registers[9 + 4 * i] << 8) == expected_ticks[i]); }
    }
    pca9685_pwm_t values[16];
    for (int i = 0; i < 16; ++i) { values[i] = (pca9685_pwm_t){4000, 200}; }
    assert(pca9685_set_channels(handle, 0, values, 16) == ESP_OK);
    assert(last_size == 65 && registers[6] == 0xa0 && registers[7] == 0x0f);
    assert(registers[0x44] == 200 && registers[0x45] == 0);
    assert(pca9685_set_pwm(handle, 3, 20, 20) == ESP_OK);
    assert(registers[9 + 3 * 4] & 0x10);
    assert(pca9685_all_off(handle) == ESP_OK);
    assert_all_off();
    assert(pca9685_set_pwm(handle, 1, 0, 307) == ESP_OK);
    assert(!(registers[13] & 0x10));
    assert(pca9685_set_frequency(handle, 60) == ESP_OK);
    assert(registers[0xfe] == 101);
    assert_all_off();
    assert(pca9685_set_pulse_width_us(handle, 0, 1500) == ESP_OK);
    assert(registers[8] == 112 && registers[9] == 1); /* 368 ticks at 60 Hz. */
    assert(pca9685_delete(handle) == ESP_OK && devices == 0);
}

static void test_arguments_and_lock(void)
{
    reset_fake();
    pca9685_handle_t handle = create_driver();
    const pca9685_pwm_t value = {0, 307};
    int before = transfer_count;
    assert(pca9685_set_pwm(handle, 16, 0, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pwm(handle, 0, 0, 4096) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_channels(handle, 15, &value, 2) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_channels(handle, 0, &value, 0) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_channels(handle, 0, NULL, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_width_us(handle, 0, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_width_us(handle, 0, 20000) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_width_us(handle, 0, UINT32_MAX) == ESP_ERR_INVALID_ARG);
    const uint32_t invalid_pulses[] = {1500, 20000};
    assert(pca9685_set_pulse_widths_us(handle, 0, invalid_pulses, 2) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_widths_us(handle, 15, invalid_pulses, 2) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_widths_us(handle, 0, NULL, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_widths_us(handle, 0, invalid_pulses, 0) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_frequency(handle, 0) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_frequency(handle, NAN) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_frequency(handle, INFINITY) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_frequency(handle, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_frequency(handle, 2000) == ESP_ERR_INVALID_ARG);
    assert(pca9685_get_frequency(handle, NULL) == ESP_ERR_INVALID_ARG);
    assert(pca9685_all_off(NULL) == ESP_ERR_INVALID_ARG);
    assert(pca9685_delete(NULL) == ESP_ERR_INVALID_ARG);
    assert(transfer_count == before);
    fail_lock = true;
    assert(pca9685_set_pwm(handle, 0, 0, 307) == ESP_ERR_TIMEOUT);
    assert(pca9685_set_frequency(handle, 60) == ESP_ERR_TIMEOUT);
    assert(transfer_count == before);
    fail_lock = false;
    assert(pca9685_set_pwm(handle, 0, 0, 307) == ESP_OK);
    fail_remove = true;
    assert(pca9685_delete(handle) == ESP_ERR_INVALID_STATE && devices == 1);
    fail_remove = false;
    assert(pca9685_delete(handle) == ESP_OK);
}

static void test_failures_and_retry(void)
{
    /* Each I2C step in frequency setup fails once; retry must restore validity. */
    for (int step = 1; step <= 9; ++step) {
        reset_fake();
        pca9685_handle_t handle = create_driver();
        fail_transfer = transfer_count + step;
        assert(pca9685_set_frequency(handle, 60) == ESP_ERR_TIMEOUT);
        assert(pca9685_set_pwm(handle, 0, 0, 307) == ESP_ERR_INVALID_STATE);
        float hz;
        assert(pca9685_get_frequency(handle, &hz) == ESP_ERR_INVALID_STATE);
        assert(pca9685_set_pulse_width_us(handle, 0, 1500) == ESP_ERR_INVALID_STATE);
        assert(pca9685_disable_channel(handle, 0) == ESP_OK);
        assert(pca9685_all_off(handle) == ESP_OK);
        assert(pca9685_set_frequency(handle, 50) == ESP_OK);
        assert(pca9685_set_pulse_width_us(handle, 0, 1500) == ESP_OK);
        fail_transfer = transfer_count + 1;
        assert(pca9685_delete(handle) == ESP_ERR_TIMEOUT && devices == 1);
        assert(pca9685_delete(handle) == ESP_OK);
    }
    for (int step = 1; step <= 9; ++step) {
        reset_fake();
        fail_transfer = step;
        const pca9685_config_t config = PCA9685_CONFIG_DEFAULT();
        pca9685_handle_t handle = NULL;
        assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_TIMEOUT);
        assert(handle == NULL && devices == 0);
    }
    reset_fake();
    registers[0] = 0x40;
    const pca9685_config_t config = PCA9685_CONFIG_DEFAULT();
    pca9685_handle_t handle = NULL;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_STATE);
    assert(handle == NULL && devices == 0);
    reset_fake();
    corrupt_prescale = true;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_RESPONSE);
    assert(handle == NULL && devices == 0);
}

static void test_board(void)
{
    reset_fake();
    board_hardware_t board = {0};
    assert(board_init(NULL) == ESP_ERR_INVALID_ARG);
    assert(board_init(&board) == ESP_OK);
    assert(bus_config.i2c_port == 0 && bus_config.sda_io_num == 39 && bus_config.scl_io_num == 38);
    assert(registers[0xfe] == 18); /* Project board requests 321.24 Hz. */
    float frequency_hz;
    assert(pca9685_get_frequency(board.pwm, &frequency_hz) == ESP_OK);
    assert(fabsf(frequency_hz - 321.2377f) < 0.001f);
    const uint32_t edge_valid = 3112, edge_invalid = 3113;
    int validation_transfers = transfer_count;
    assert(pca9685_validate_pulse_widths_us(board.pwm, &edge_valid, 1) == ESP_OK);
    assert(pca9685_validate_pulse_widths_us(board.pwm, &edge_invalid, 1) == ESP_ERR_INVALID_ARG);
    assert(pca9685_set_pulse_width_us(board.pwm, 0, edge_invalid) == ESP_ERR_INVALID_ARG);
    assert(transfer_count == validation_transfers);
    assert(board_init(&board) == ESP_ERR_INVALID_STATE);
    assert_all_off();
    const uint32_t pulses[] = {500, 1000, 1500, 2000, 2500};
    const unsigned expected_ticks[] = {658, 1316, 1974, 2632, 3289};
    assert(pca9685_set_pulse_widths_us(board.pwm, 0, pulses, 5) == ESP_OK);
    for (unsigned i = 0; i < 5; ++i) {
        assert(registers[8 + 4 * i] + ((unsigned)registers[9 + 4 * i] << 8) == expected_ticks[i]);
    }
    int before = transfer_count;
    assert(pca9685_set_pulse_width_us(board.pwm, 0, 3200) == ESP_ERR_INVALID_ARG);
    assert(transfer_count == before);
    assert(board_deinit(&board) == ESP_OK);
    assert(board.i2c_bus == NULL && board.pwm == NULL && devices == 0 && buses == 0);
    assert(board_deinit(&board) == ESP_OK);
    reset_fake();
    fail_transfer = 1;
    assert(board_init(&board) == ESP_ERR_TIMEOUT);
    assert(board.i2c_bus == NULL && board.pwm == NULL && devices == 0 && buses == 0);
}

static void test_configuration(void)
{
    reset_fake();
    pca9685_config_t config = PCA9685_CONFIG_DEFAULT();
    pca9685_handle_t handle = NULL;
    assert(pca9685_create(NULL, &config, &handle) == ESP_ERR_INVALID_ARG);
    assert(pca9685_create(&buses, NULL, &handle) == ESP_ERR_INVALID_ARG);
    assert(pca9685_create(&buses, &config, NULL) == ESP_ERR_INVALID_ARG);
    config.address = 0x80;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_ARG);
    config.address = 0x40;
    config.timeout_ms = 0;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_ARG);
    config.timeout_ms = 100;
    config.i2c_speed_hz = 1000000;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_ARG);
    config.i2c_speed_hz = 100000;
    config.oscillator_hz = 0;
    assert(pca9685_create(&buses, &config, &handle) == ESP_ERR_INVALID_ARG);
    assert(handle == NULL && transfer_count == 0 && devices == 0);
    config.oscillator_hz = 27000000;
    assert(pca9685_create(&buses, &config, &handle) == ESP_OK);
    assert(registers[0xfe] == 131);
    assert(pca9685_set_pulse_width_us(handle, 0, 1500) == ESP_OK);
    assert(registers[8] == 0x33 && registers[9] == 1);
    assert(pca9685_delete(handle) == ESP_OK);
}

int main(void)
{
    test_outputs();
    test_arguments_and_lock();
    test_failures_and_retry();
    test_board();
    test_configuration();
    puts("PASS: PCA9685 outputs, bounds, failures/retry, cleanup and board wiring");
    return 0;
}
