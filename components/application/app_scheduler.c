#include "app_scheduler.h"

static esp_err_t release_servos(app_scheduler_t *scheduler)
{
    business_servo_cycle_stop(&scheduler->servo_cycle);
    scheduler->active = APP_BUSINESS_NONE;
    scheduler->off_error = scheduler->port.all_off(scheduler->port.context);
    return scheduler->off_error;
}

esp_err_t app_scheduler_init(app_scheduler_t *scheduler, const app_servo_port_t *port,
                              const servo_cycle_config_t *config)
{
    if (scheduler == NULL || !app_servo_port_valid(port) || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (scheduler->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    *scheduler = (app_scheduler_t){.port = *port, .config = *config, .initialized = true};
    return ESP_OK;
}

esp_err_t app_scheduler_select(app_scheduler_t *scheduler, app_business_id_t business)
{
    if (scheduler == NULL ||
        (business != APP_BUSINESS_NONE && business != APP_BUSINESS_SERVO_CYCLE)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!scheduler->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (business == scheduler->active && scheduler->off_error == ESP_OK) {
        return ESP_OK;
    }
    /* Revoke the old owner and require successful shutdown before granting a
     * new owner, including an explicit restart after a previous shutdown error.
     */
    esp_err_t err = release_servos(scheduler);
    if (err != ESP_OK || business == APP_BUSINESS_NONE) {
        return err;
    }
    err = business_servo_cycle_start(&scheduler->servo_cycle, &scheduler->port, &scheduler->config);
    if (err != ESP_OK) {
        (void)release_servos(scheduler);
        return err;
    }
    scheduler->active = business;
    return ESP_OK;
}

esp_err_t app_scheduler_process(app_scheduler_t *scheduler)
{
    if (scheduler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!scheduler->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (scheduler->active == APP_BUSINESS_NONE) {
        return ESP_OK;
    }
    esp_err_t err = business_servo_cycle_process(&scheduler->servo_cycle);
    if (err != ESP_OK) {
        (void)release_servos(scheduler);
    }
    return err;
}
