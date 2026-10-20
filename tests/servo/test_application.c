/* Exercise production pulse validation, business rules, exclusive scheduling and
 * application startup/queueing. No real servo hardware is used.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "servo.h"
#include "app_scheduler.h"
#include "app_profile.h"
#include "trajectory.h"
#include "board.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <setjmp.h>
#include <stdlib.h>

static int fake_device;
static int64_t now_us;
static unsigned write_count, off_count;
static uint32_t last_pulses[SERVO_CHANNEL_COUNT];
static esp_err_t write_error, off_error, frequency_error;
static servo_cycle_config_t test_motion;

int64_t esp_timer_get_time(void) { return now_us; }
esp_err_t pca9685_get_frequency(pca9685_handle_t pwm, float *hz)
{
    assert(pwm == (pca9685_handle_t)&fake_device);
    *hz = 321.2377f;
    return frequency_error;
}
esp_err_t pca9685_validate_pulse_widths_us(pca9685_handle_t pwm,
                                          const uint32_t *pulses, size_t count)
{
    assert(pwm == (pca9685_handle_t)&fake_device && count == 16);
    if (frequency_error != ESP_OK) { return frequency_error; }
    for (size_t i = 0; i < count; ++i) {
        if (pulses[i] != 0 && (pulses[i] + 0.38) / 0.76 >= 4096) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}
esp_err_t pca9685_set_pulse_widths_us(pca9685_handle_t pwm, uint8_t first,
                                     const uint32_t *pulses, size_t count)
{
    assert(pwm == (pca9685_handle_t)&fake_device && first == 0 && count == 16);
    ++write_count;
    now_us += 6000; /* Model transfer duration: hold time must begin AFTER this. */
    if (write_error != ESP_OK) { return write_error; }
    memcpy(last_pulses, pulses, sizeof(last_pulses));
    return ESP_OK;
}
esp_err_t pca9685_set_pulse_width_us(pca9685_handle_t pwm, uint8_t channel, uint32_t pulse)
{
    assert(pwm == (pca9685_handle_t)&fake_device && channel < 16);
    ++write_count;
    last_pulses[channel] = pulse;
    return write_error;
}
esp_err_t pca9685_all_off(pca9685_handle_t pwm)
{
    assert(pwm == (pca9685_handle_t)&fake_device);
    ++off_count;
    return off_error;
}

static void reset_fake(void)
{
    now_us = 123000;
    write_count = off_count = 0;
    write_error = off_error = frequency_error = ESP_OK;
    memset(last_pulses, 0, sizeof(last_pulses));
}
static void init_bank(servo_bank_t *bank)
{
    servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT];
    for (size_t i = 0; i < SERVO_CHANNEL_COUNT; ++i) {
        ranges[i] = (servo_pulse_range_t){600, 1000 + (uint32_t)(i % 4) * 500};
    }
    assert(servo_bank_init(bank, (pca9685_handle_t)&fake_device, ranges) == ESP_OK);
}
static void assert_low_pulses(void)
{
    for (size_t i = 0; i < SERVO_CHANNEL_COUNT; ++i) { assert(last_pulses[i] == 600); }
}
static void assert_high_pulses(void)
{
    const uint32_t expected[] = {1000, 1500, 2000, 2500};
    for (size_t i = 0; i < SERVO_CHANNEL_COUNT; ++i) { assert(last_pulses[i] == expected[i % 4]); }
}

static void test_pulse_validation(void)
{
    reset_fake();
    servo_bank_t bank = {0};
    init_bank(&bank);
    uint32_t pulses[SERVO_CHANNEL_COUNT];
    for (unsigned i = 0; i < SERVO_CHANNEL_COUNT; ++i) { pulses[i] = 600; }
    pulses[15] = 2501;
    assert(servo_bank_set_pulses(&bank, pulses) == ESP_ERR_INVALID_ARG && write_count == 0);
    pulses[15] = 599;
    assert(servo_bank_set_pulses(&bank, pulses) == ESP_ERR_INVALID_ARG && write_count == 0);
    pulses[15] = 2500;
    assert(servo_bank_set_pulses(&bank, pulses) == ESP_OK && last_pulses[15] == 2500);
    assert(servo_bank_set_pulse(&bank, 16, 600) == ESP_ERR_INVALID_ARG);
    assert(servo_bank_set_pulse(&bank, 0, 1001) == ESP_ERR_INVALID_ARG);
    assert(servo_bank_set_pulse(&bank, 15, 2000) == ESP_OK && last_pulses[15] == 2000);
    assert(servo_bank_validate_pulses(NULL, pulses) == ESP_ERR_INVALID_ARG);
    assert(servo_bank_validate_pulses(&bank, NULL) == ESP_ERR_INVALID_ARG);
    servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT];
    memcpy(ranges, bank.ranges, sizeof(ranges));
    assert(servo_bank_init(&bank, (pca9685_handle_t)&fake_device, ranges) == ESP_ERR_INVALID_STATE);
    bank = (servo_bank_t){0};
    ranges[15].min_pulse_us = 0;
    assert(servo_bank_init(&bank, (pca9685_handle_t)&fake_device, ranges) == ESP_ERR_INVALID_ARG);
    ranges[15] = (servo_pulse_range_t){500, 20000};
    assert(servo_bank_init(&bank, (pca9685_handle_t)&fake_device, ranges) == ESP_ERR_INVALID_ARG);
    assert(bank.pwm == NULL);
}

static esp_err_t validate_port(void *context, const uint32_t *pulses)
{
    return servo_bank_validate_pulses(context, pulses);
}
static esp_err_t write_port(void *context, const uint32_t *pulses)
{
    return servo_bank_set_pulses(context, pulses);
}
static esp_err_t off_port(void *context) { return servo_bank_all_off(context); }
static int64_t time_port(void *context) { (void)context; return now_us; }
static app_servo_port_t make_port(servo_bank_t *bank)
{
    return (app_servo_port_t){bank, validate_port, write_port, off_port, time_port};
}

static void test_cycle_timing(void)
{
    reset_fake();
    servo_bank_t bank = {0};
    init_bank(&bank);
    app_servo_port_t port = make_port(&bank);
    business_servo_cycle_t cycle = {0};
    assert(business_servo_cycle_start(&cycle, &port, &test_motion) == ESP_OK && cycle.state != APP_MOTION_STOPPED);
    assert(write_count == 1);
    assert_low_pulses();
    assert(cycle.next_command_us == 2129000);
    assert(business_servo_cycle_start(&cycle, &port, &test_motion) == ESP_ERR_INVALID_STATE);
    now_us = cycle.next_command_us - 1;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && write_count == 1);
    ++now_us;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && write_count == 2 && cycle.state == APP_MOTION_TO_END);
    assert_low_pulses();
    int64_t start = cycle.motion_start_us;
    uint32_t forward[101][SERVO_CHANNEL_COUNT];
    memcpy(forward[0], last_pulses, sizeof(last_pulses));
    for (int step = 1; step <= 100; ++step) {
        now_us = start + step * INT64_C(20000);
        unsigned previous = write_count;
        assert(business_servo_cycle_process(&cycle) == ESP_OK && write_count == previous + 1);
        memcpy(forward[step], last_pulses, sizeof(last_pulses));
        for (unsigned ch = 0; ch < SERVO_CHANNEL_COUNT; ++ch) {
            assert(forward[step][ch] >= forward[step - 1][ch]);
            assert(forward[step][ch] <= 1000 + (ch % 4) * 500);
            /* Frozen pre-refactor trajectory: verify every scheduled command. */
            double t = step / 100.0;
            double s = t * t * t * (10 + t * (-15 + 6 * t));
            uint32_t expected = 600 + (uint32_t)(s * (400 + (ch % 4) * 500) + 0.5);
            assert(forward[step][ch] == expected);
        }
    }
    assert_high_pulses();
    assert(cycle.state == APP_MOTION_HOLD_END);
    assert(cycle.next_command_us == now_us + 2000000);
    /* Midpoint, symmetry and slow start/finish distinguish S interpolation. */
    assert(forward[50][3] == 1550);
    assert(forward[25][3] == 797 && forward[75][3] == 2303);
    assert(forward[1][3] - forward[0][3] < forward[50][3] - forward[49][3]);
    assert(forward[100][3] - forward[99][3] < forward[50][3] - forward[49][3]);
    unsigned previous = write_count;
    now_us = cycle.next_command_us - 1;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && write_count == previous);
    ++now_us;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && cycle.state == APP_MOTION_TO_START);
    start = cycle.motion_start_us;
    for (int step = 1; step <= 100; ++step) {
        now_us = start + step * INT64_C(20000);
        assert(business_servo_cycle_process(&cycle) == ESP_OK);
        for (unsigned ch = 0; ch < SERVO_CHANNEL_COUNT; ++ch) {
            assert(last_pulses[ch] == forward[100 - step][ch]);
        }
    }
    assert_low_pulses();
    now_us = cycle.next_command_us + 10000000;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && cycle.state == APP_MOTION_TO_END);
    assert_low_pulses(); /* Late hold poll starts a fresh motion. */
    previous = write_count;
    now_us += 10000000;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && write_count == previous + 1);
    assert_high_pulses(); /* Late motion poll emits one bounded endpoint. */
    assert(cycle.next_command_us == now_us + 2000000);
    business_servo_cycle_stop(&cycle);
    assert(cycle.state == APP_MOTION_STOPPED && off_count == 0);
    assert(business_servo_cycle_process(&cycle) == ESP_ERR_INVALID_STATE);
    assert(business_servo_cycle_start(NULL, &port, &test_motion) == ESP_ERR_INVALID_ARG);
    assert(business_servo_cycle_process(NULL) == ESP_ERR_INVALID_ARG);
    port.write_pulses = NULL;
    assert(business_servo_cycle_start(&cycle, &port, &test_motion) == ESP_ERR_INVALID_ARG);
}

static void test_scheduler_ownership(void)
{
    reset_fake();
    servo_bank_t bank = {0};
    init_bank(&bank);
    app_servo_port_t port = make_port(&bank);
    app_scheduler_t manager = {0};
    assert(app_scheduler_process(&manager) == ESP_ERR_INVALID_STATE);
    assert(app_scheduler_init(&manager, &port, &test_motion) == ESP_OK);
    assert(app_scheduler_init(&manager, &port, &test_motion) == ESP_ERR_INVALID_STATE);
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_OK);
    assert(manager.active == APP_BUSINESS_SERVO_CYCLE && write_count == 1 && off_count == 1);
    int64_t deadline = manager.servo_cycle.next_command_us;
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_OK);
    assert(write_count == 1 && off_count == 1 && manager.servo_cycle.next_command_us == deadline);
    assert(app_scheduler_select(&manager, (app_business_id_t)99) == ESP_ERR_INVALID_ARG);
    assert(manager.active == APP_BUSINESS_SERVO_CYCLE && off_count == 1);
    now_us = deadline;
    assert(app_scheduler_process(&manager) == ESP_OK && write_count == 2);
    assert_low_pulses();
    assert(app_scheduler_select(&manager, APP_BUSINESS_NONE) == ESP_OK);
    assert(manager.active == APP_BUSINESS_NONE && manager.servo_cycle.state == APP_MOTION_STOPPED && off_count == 2);
    now_us += 10000000;
    assert(app_scheduler_process(&manager) == ESP_OK && write_count == 2);
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_OK && write_count == 3);
    assert_low_pulses(); /* Restart resets phase and endpoint hold. */
    assert(manager.servo_cycle.next_command_us == now_us + 2000000);

    off_error = ESP_ERR_TIMEOUT;
    assert(app_scheduler_select(&manager, APP_BUSINESS_NONE) == ESP_ERR_TIMEOUT);
    assert(manager.active == APP_BUSINESS_NONE && manager.servo_cycle.state == APP_MOTION_STOPPED);
    unsigned previous = write_count;
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_ERR_TIMEOUT);
    assert(manager.active == APP_BUSINESS_NONE && write_count == previous);
    assert(app_scheduler_process(&manager) == ESP_OK && write_count == previous);
    off_error = ESP_OK;
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_OK);
    assert(write_count == previous + 1);
    now_us = manager.servo_cycle.next_command_us;
    write_error = ESP_ERR_TIMEOUT;
    off_error = ESP_ERR_INVALID_STATE;
    assert(app_scheduler_process(&manager) == ESP_ERR_TIMEOUT);
    assert(manager.active == APP_BUSINESS_NONE && manager.servo_cycle.state == APP_MOTION_STOPPED);
    assert(manager.off_error == ESP_ERR_INVALID_STATE);
    previous = write_count;
    assert(app_scheduler_process(&manager) == ESP_OK && write_count == previous);
    off_error = write_error = ESP_OK;
    assert(app_scheduler_select(&manager, APP_BUSINESS_NONE) == ESP_OK && manager.off_error == ESP_OK);
    write_error = ESP_ERR_TIMEOUT;
    previous = off_count;
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_ERR_TIMEOUT);
    assert(manager.active == APP_BUSINESS_NONE && off_count == previous + 2);
    assert(app_scheduler_select(NULL, APP_BUSINESS_NONE) == ESP_ERR_INVALID_ARG);
    assert(app_scheduler_process(NULL) == ESP_ERR_INVALID_ARG);
}

static void test_invalid_range_rejects_business(void)
{
    reset_fake();
    servo_bank_t bank = {0};
    init_bank(&bank);
    bank.ranges[15].max_pulse_us = 2000;
    app_servo_port_t port = make_port(&bank);
    app_scheduler_t manager = {0};
    assert(app_scheduler_init(&manager, &port, &test_motion) == ESP_OK);
    assert(app_scheduler_select(&manager, APP_BUSINESS_SERVO_CYCLE) == ESP_ERR_INVALID_ARG);
    assert(manager.active == APP_BUSINESS_NONE && write_count == 0);
}

static void test_motion_configuration(void)
{
    reset_fake();
    servo_bank_t bank = {0};
    init_bank(&bank);
    app_servo_port_t port = make_port(&bank);
    business_servo_cycle_t cycle = {0};
    servo_cycle_config_t motion = test_motion;
    assert(trajectory_quintic(0) == 0 && trajectory_quintic(1) == 1);
    assert(trajectory_quintic(0.5) == 0.5);
    motion.motion_us = 0;
    assert(business_servo_cycle_start(&cycle, &port, &motion) == ESP_ERR_INVALID_ARG);
    motion = test_motion;
    motion.hold_us = -1;
    assert(business_servo_cycle_start(&cycle, &port, &motion) == ESP_ERR_INVALID_ARG);
    motion = test_motion;
    motion.update_us = 0;
    assert(business_servo_cycle_start(&cycle, &port, &motion) == ESP_ERR_INVALID_ARG);
    motion = test_motion;
    motion.start_us[0] = 599;
    assert(business_servo_cycle_start(&cycle, &port, &motion) == ESP_ERR_INVALID_ARG);
    assert(write_count == 0);
    /* A different valid motion uses the same independent allowed ranges.
     * Descending endpoints, zero dwell and copied configuration are supported. */
    motion = test_motion;
    motion.start_us[0] = 1000;
    motion.end_us[0] = 700;
    motion.hold_us = 0;
    assert(business_servo_cycle_start(&cycle, &port, &motion) == ESP_OK);
    assert(last_pulses[0] == 1000);
    motion.end_us[0] = 900;
    assert(business_servo_cycle_process(&cycle) == ESP_OK);
    now_us = cycle.motion_start_us + 1000000;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && last_pulses[0] == 850);
    now_us = cycle.motion_start_us + 2000000;
    assert(business_servo_cycle_process(&cycle) == ESP_OK && last_pulses[0] == 700);
    assert(cycle.state == APP_MOTION_HOLD_END);
}

/* Runtime adapter fakes exercise public startup/queueing and the actual owner
 * task. The receive fake exits the infinite task only after queued switches and
 * one outward S-curve motion has been executed.
 */
typedef struct {
    app_business_id_t items[4];
    unsigned count;
} fake_queue_t;
static unsigned queue_objects, board_objects, cleanup_count, idle_receives;
static bool fail_queue, fail_task;
static esp_err_t board_error;
static esp_err_t cleanup_error;
static TaskFunction_t created_task;
static jmp_buf task_exit;
static unsigned fault_stage;

QueueHandle_t xQueueCreate(unsigned length, unsigned item_size)
{
    assert(length == 4 && item_size == sizeof(app_business_id_t));
    if (fail_queue) { return NULL; }
    fake_queue_t *queue = calloc(1, sizeof(*queue));
    assert(queue != NULL);
    ++queue_objects;
    return queue;
}
int xQueueSend(QueueHandle_t handle, const void *item, TickType_t wait)
{
    fake_queue_t *queue = handle;
    assert(wait == 0);
    if (queue->count == 4) { return 0; }
    queue->items[queue->count++] = *(const app_business_id_t *)item;
    return pdTRUE;
}
int xQueueReceive(QueueHandle_t handle, void *item, TickType_t wait)
{
    fake_queue_t *queue = handle;
    assert(wait == 5);
    app_status_t snapshot;
    assert(app_get_status(&snapshot) == ESP_OK && snapshot.lifecycle == APP_LIFECYCLE_READY);
    if (fault_stage == 1) {
        fault_stage = 2;
        write_error = ESP_ERR_TIMEOUT;
        off_error = ESP_ERR_INVALID_STATE;
        now_us += 2000000;
        return 0;
    }
    if (fault_stage == 2) {
        assert(snapshot.active == APP_BUSINESS_NONE && snapshot.motion == APP_MOTION_STOPPED);
        assert(snapshot.last_error == ESP_ERR_TIMEOUT);
        assert(snapshot.shutdown_error == ESP_ERR_INVALID_STATE);
        write_error = off_error = ESP_OK;
        assert(app_select_business(APP_BUSINESS_NONE) == ESP_OK);
        fault_stage = 3;
    } else if (fault_stage == 3) {
        assert(snapshot.active == APP_BUSINESS_NONE && snapshot.motion == APP_MOTION_STOPPED);
        assert(snapshot.last_error == ESP_OK && snapshot.shutdown_error == ESP_OK);
        fault_stage = 0;
        longjmp(task_exit, 1);
    }
    if (queue->count != 0) {
        *(app_business_id_t *)item = queue->items[0];
        --queue->count;
        memmove(queue->items, queue->items + 1, queue->count * sizeof(queue->items[0]));
        return pdTRUE;
    }
    if (++idle_receives <= 2) { now_us += 2000000; return 0; }
    longjmp(task_exit, 1);
}
void vQueueDelete(QueueHandle_t handle) { free(handle); --queue_objects; }
int xTaskCreate(TaskFunction_t task, const char *name, uint32_t stack, void *arg,
                 unsigned priority, void **handle)
{
    assert(strcmp(name, "application") == 0 && stack == 4096 && arg == NULL);
    assert(priority == 5 && handle == NULL);
    if (fail_task) { return 0; }
    created_task = task;
    return pdPASS;
}
esp_err_t board_init(board_hardware_t *board)
{
    ++board_objects;
    board->i2c_bus = &fake_device;
    if (board_error != ESP_OK) { return board_error; }
    board->pwm = (pca9685_handle_t)&fake_device;
    return ESP_OK;
}
esp_err_t board_deinit(board_hardware_t *board)
{
    assert(board->i2c_bus != NULL);
    if (cleanup_error != ESP_OK) { return cleanup_error; }
    *board = (board_hardware_t){0};
    --board_objects;
    ++cleanup_count;
    return ESP_OK;
}

static void test_application_lifecycle(void)
{
    reset_fake();
    app_status_t snapshot;
    assert(app_get_status(NULL) == ESP_ERR_INVALID_ARG);
    assert(app_get_status(&snapshot) == ESP_OK);
    assert(snapshot.lifecycle == APP_LIFECYCLE_UNINITIALIZED);
    assert(app_retry_cleanup() == ESP_ERR_INVALID_STATE);
    assert(app_select_business(APP_BUSINESS_SERVO_CYCLE) == ESP_ERR_INVALID_STATE);
    fail_queue = true;
    assert(app_start() == ESP_ERR_NO_MEM && board_objects == 0 && queue_objects == 0);
    fail_queue = false;
    board_error = ESP_ERR_TIMEOUT;
    assert(app_start() == ESP_ERR_TIMEOUT && board_objects == 0 && queue_objects == 0);
    assert(cleanup_count == 1);
    board_error = ESP_OK;
    frequency_error = ESP_ERR_INVALID_STATE;
    assert(app_start() == ESP_ERR_INVALID_STATE && board_objects == 0 && queue_objects == 0);
    frequency_error = ESP_OK;
    fail_task = true;
    assert(app_start() == ESP_ERR_NO_MEM && board_objects == 0 && queue_objects == 0);
    assert(cleanup_count == 3 && write_count == 0);
    cleanup_error = ESP_ERR_TIMEOUT;
    assert(app_start() == ESP_ERR_NO_MEM && board_objects == 1 && queue_objects == 0);
    assert(app_get_status(&snapshot) == ESP_OK);
    assert(snapshot.lifecycle == APP_LIFECYCLE_CLEANUP_REQUIRED);
    assert(snapshot.last_error == ESP_ERR_NO_MEM && snapshot.cleanup_error == ESP_ERR_TIMEOUT);
    assert(app_start() == ESP_ERR_INVALID_STATE);
    assert(app_select_business(APP_BUSINESS_SERVO_CYCLE) == ESP_ERR_INVALID_STATE);
    assert(app_retry_cleanup() == ESP_ERR_TIMEOUT && board_objects == 1 && queue_objects == 0);
    cleanup_error = ESP_OK;
    assert(app_retry_cleanup() == ESP_OK && board_objects == 0 && queue_objects == 0);
    assert(app_get_status(&snapshot) == ESP_OK);
    assert(snapshot.lifecycle == APP_LIFECYCLE_UNINITIALIZED && snapshot.cleanup_error == ESP_OK);
    fail_task = false;
    assert(app_start() == ESP_OK && board_objects == 1 && queue_objects == 1);
    assert(app_retry_cleanup() == ESP_ERR_INVALID_STATE);
    assert(app_get_status(&snapshot) == ESP_OK);
    assert(snapshot.lifecycle == APP_LIFECYCLE_READY && snapshot.active == APP_BUSINESS_NONE);
    assert(app_start() == ESP_ERR_INVALID_STATE);
    assert(app_select_business((app_business_id_t)99) == ESP_ERR_INVALID_ARG);
    assert(app_select_business(APP_BUSINESS_NONE) == ESP_OK);
    assert(app_select_business(APP_BUSINESS_SERVO_CYCLE) == ESP_OK);
    assert(app_select_business(APP_BUSINESS_NONE) == ESP_OK);
    assert(app_select_business(APP_BUSINESS_SERVO_CYCLE) == ESP_OK);
    assert(app_select_business(APP_BUSINESS_NONE) == ESP_ERR_TIMEOUT);
    assert(write_count == 0 && off_count == 0); /* API callers never drive hardware. */
    assert(created_task != NULL);
    if (setjmp(task_exit) == 0) { created_task(NULL); }
    assert(write_count == 5 && off_count == 5);
    assert_high_pulses();
    assert(app_get_status(&snapshot) == ESP_OK);
    assert(snapshot.motion == APP_MOTION_HOLD_END && snapshot.active == APP_BUSINESS_SERVO_CYCLE);
    unsigned before = write_count;
    for (unsigned i = 0; i < 10; ++i) { assert(app_get_status(&snapshot) == ESP_OK); }
    assert(write_count == before); /* Read-only snapshot performs no PWM I/O. */
    fault_stage = 1;
    if (setjmp(task_exit) == 0) { created_task(NULL); }
    assert(fault_stage == 0);
}

int main(void)
{
    servo_pulse_range_t limits[SERVO_CHANNEL_COUNT];
    app_profile_make(&test_motion, limits);
    test_pulse_validation();
    test_cycle_timing();
    test_scheduler_ownership();
    test_invalid_range_rejects_business();
    test_motion_configuration();
    test_application_lifecycle();
    puts("PASS: pulse ranges/S-curve timing, exclusive ownership, switching, fault shutdown, startup and queue");
    return 0;
}
