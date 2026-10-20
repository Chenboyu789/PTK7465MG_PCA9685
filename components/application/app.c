#include "app.h"
#include "app_scheduler.h"
#include "app_profile.h"
#include "board.h"
#include "servo.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

_Static_assert(APP_SERVO_CHANNEL_COUNT == SERVO_CHANNEL_COUNT, "Servo bank size mismatch");

static const char *TAG = "app";
static board_hardware_t hardware;
static servo_bank_t servos;
static app_scheduler_t scheduler;
static QueueHandle_t requests;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static app_status_t status;

esp_err_t app_get_status(app_status_t *out)
{
    if (out == NULL) { return ESP_ERR_INVALID_ARG; }
    taskENTER_CRITICAL(&status_mux);
    *out = status;
    taskEXIT_CRITICAL(&status_mux);
    return ESP_OK;
}

/* Only the owner task publishes scheduler state after startup. */
static void publish_operation(esp_err_t err, bool clear_error)
{
    taskENTER_CRITICAL(&status_mux);
    status.active = scheduler.active;
    status.motion = scheduler.servo_cycle.state;
    status.shutdown_error = scheduler.off_error;
    if (err != ESP_OK || clear_error) { status.last_error = err; }
    taskEXIT_CRITICAL(&status_mux);
}

static void publish_lifecycle(app_lifecycle_t lifecycle, esp_err_t err, esp_err_t cleanup_err)
{
    taskENTER_CRITICAL(&status_mux);
    status = (app_status_t){.lifecycle = lifecycle, .last_error = err,
                            .cleanup_error = cleanup_err};
    taskEXIT_CRITICAL(&status_mux);
}

static esp_err_t validate_pulses(void *context, const uint32_t *pulses)
{
    return servo_bank_validate_pulses(context, pulses);
}

static esp_err_t write_pulses(void *context, const uint32_t *pulses)
{
    return servo_bank_set_pulses(context, pulses);
}

static esp_err_t all_off(void *context)
{
    return servo_bank_all_off(context);
}

static int64_t now_us(void *context)
{
    (void)context;
    return esp_timer_get_time();
}

static void report_error(const char *operation, esp_err_t err)
{
    ESP_LOGE(TAG, "%s: %s", operation, esp_err_to_name(err));
    if (scheduler.off_error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable servo outputs: %s", esp_err_to_name(scheduler.off_error));
    }
}

static void select_business(app_business_id_t business)
{
    app_business_id_t previous = scheduler.active;
    esp_err_t err = app_scheduler_select(&scheduler, business);
    publish_operation(err, true);
    if (err != ESP_OK) {
        report_error("Business selection failed", err);
    } else if (business != previous) {
        if (business == APP_BUSINESS_SERVO_CYCLE) {
            ESP_LOGI(TAG, "16-channel motion started (servo cycle)");
        } else {
            ESP_LOGI(TAG, "Business stopped; all channels OFF");
        }
    }
}

static void application_task(void *arg)
{
    (void)arg;
    select_business(APP_BUSINESS_SERVO_CYCLE);
    TickType_t ticks = pdMS_TO_TICKS(5);
    for (;;) {
        app_business_id_t business;
        if (xQueueReceive(requests, &business, ticks > 0 ? ticks : 1) == pdTRUE) {
            select_business(business);
        }
        esp_err_t err = app_scheduler_process(&scheduler);
        publish_operation(err, false);
        if (err != ESP_OK) {
            report_error("Business stopped after processing failure", err);
        }
    }
}

static esp_err_t cleanup_startup(esp_err_t startup_error)
{
    esp_err_t err = board_deinit(&hardware);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Hardware cleanup failed: %s", esp_err_to_name(err));
    } else {
        servos = (servo_bank_t){0};
    }
    scheduler = (app_scheduler_t){0};
    if (requests != NULL) { vQueueDelete(requests); }
    requests = NULL;
    publish_lifecycle(err == ESP_OK ? APP_LIFECYCLE_UNINITIALIZED
                                    : APP_LIFECYCLE_CLEANUP_REQUIRED,
                      startup_error, err);
    return err;
}

esp_err_t app_retry_cleanup(void)
{
    app_status_t snapshot;
    app_get_status(&snapshot);
    if (snapshot.lifecycle != APP_LIFECYCLE_CLEANUP_REQUIRED) {
        return ESP_ERR_INVALID_STATE;
    }
    return cleanup_startup(snapshot.last_error);
}

esp_err_t app_start(void)
{
    app_status_t snapshot;
    app_get_status(&snapshot);
    if (snapshot.lifecycle != APP_LIFECYCLE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    publish_lifecycle(APP_LIFECYCLE_INITIALIZING, ESP_OK, ESP_OK);
    ESP_LOGI(TAG, "ServoTorqueTesting started");
    requests = xQueueCreate(4, sizeof(app_business_id_t));
    if (requests == NULL) {
        publish_lifecycle(APP_LIFECYCLE_UNINITIALIZED, ESP_ERR_NO_MEM, ESP_OK);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = board_init(&hardware);
    if (err != ESP_OK) {
        cleanup_startup(err);
        return err;
    }
    ESP_LOGI(TAG, "PCA9685 ready; all channels OFF");
    servo_pulse_range_t ranges[SERVO_CHANNEL_COUNT];
    servo_cycle_config_t motion;
    app_profile_make(&motion, ranges);
    err = servo_bank_init(&servos, hardware.pwm, ranges);
    if (err == ESP_OK) {
        const app_servo_port_t port = {
            .context = &servos,
            .validate_pulses = validate_pulses,
            .write_pulses = write_pulses,
            .all_off = all_off,
            .now_us = now_us,
        };
        err = app_scheduler_init(&scheduler, &port, &motion);
    }
    if (err == ESP_OK) {
        /* Publish before create: a newly created higher-priority task may run
         * immediately. Public control is supported after app_start returns. */
        publish_lifecycle(APP_LIFECYCLE_READY, ESP_OK, ESP_OK);
        if (xTaskCreate(application_task, "application", 4096, NULL, 5, NULL) != pdPASS) {
            err = ESP_ERR_NO_MEM;
        }
    }
    if (err != ESP_OK) {
        cleanup_startup(err);
    }
    return err;
}

esp_err_t app_select_business(app_business_id_t business)
{
    if (business != APP_BUSINESS_NONE && business != APP_BUSINESS_SERVO_CYCLE) {
        return ESP_ERR_INVALID_ARG;
    }
    app_status_t snapshot;
    app_get_status(&snapshot);
    if (snapshot.lifecycle != APP_LIFECYCLE_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(requests, &business, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}
