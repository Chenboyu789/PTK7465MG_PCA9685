#include "business_servo_cycle.h"
#include "trajectory.h"

static void make_pulses(const servo_cycle_config_t *config, double fraction,
                         uint32_t pulses[APP_SERVO_CHANNEL_COUNT])
{
    for (unsigned i = 0; i < APP_SERVO_CHANNEL_COUNT; ++i) {
        pulses[i] = (uint32_t)(config->start_us[i] +
            fraction * ((double)config->end_us[i] - config->start_us[i]) + 0.5);
    }
}

esp_err_t business_servo_cycle_start(business_servo_cycle_t *cycle, const app_servo_port_t *port,
                                     const servo_cycle_config_t *config)
{
    if (cycle == NULL || !app_servo_port_valid(port) || config == NULL ||
        config->motion_us <= 0 || config->hold_us < 0 || config->update_us <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cycle->state != APP_MOTION_STOPPED) { return ESP_ERR_INVALID_STATE; }
    uint32_t low[APP_SERVO_CHANNEL_COUNT], high[APP_SERVO_CHANNEL_COUNT];
    make_pulses(config, 0, low);
    make_pulses(config, 1, high);
    esp_err_t err = port->validate_pulses(port->context, low);
    if (err == ESP_OK) { err = port->validate_pulses(port->context, high); }
    if (err != ESP_OK) { return err; }
    cycle->port = *port;
    cycle->config = *config;
    err = port->write_pulses(port->context, low);
    if (err != ESP_OK) { return err; }
    cycle->state = APP_MOTION_HOLD_START;
    cycle->next_command_us = port->now_us(port->context) + config->hold_us;
    return ESP_OK;
}

esp_err_t business_servo_cycle_process(business_servo_cycle_t *cycle)
{
    if (cycle == NULL) { return ESP_ERR_INVALID_ARG; }
    if (cycle->state == APP_MOTION_STOPPED) { return ESP_ERR_INVALID_STATE; }
    int64_t now = cycle->port.now_us(cycle->port.context);
    if (now < cycle->next_command_us) { return ESP_OK; }
    if (cycle->state == APP_MOTION_HOLD_START || cycle->state == APP_MOTION_HOLD_END) {
        cycle->motion_start_us = now;
        cycle->state = cycle->state == APP_MOTION_HOLD_START
            ? APP_MOTION_TO_END : APP_MOTION_TO_START;
    }
    int64_t elapsed = now - cycle->motion_start_us;
    bool arrived = elapsed >= cycle->config.motion_us;
    double t = arrived ? 1.0 : (double)elapsed / cycle->config.motion_us;
    /* Quintic S curve: zero velocity and acceleration at both endpoints. */
    double fraction = trajectory_quintic(t);
    if (cycle->state == APP_MOTION_TO_START) { fraction = 1.0 - fraction; }
    uint32_t pulses[APP_SERVO_CHANNEL_COUNT];
    make_pulses(&cycle->config, fraction, pulses);
    esp_err_t err = cycle->port.write_pulses(cycle->port.context, pulses);
    if (err != ESP_OK) {
        cycle->state = APP_MOTION_STOPPED;
        return err;
    }
    int64_t completed = cycle->port.now_us(cycle->port.context);
    if (arrived) {
        cycle->state = cycle->state == APP_MOTION_TO_END
            ? APP_MOTION_HOLD_END : APP_MOTION_HOLD_START;
        cycle->next_command_us = completed + cycle->config.hold_us;
    } else {
        /* No replay of missed samples. Include bus time in the update budget. */
        cycle->next_command_us = now + cycle->config.update_us;
        if (cycle->next_command_us <= completed) {
            cycle->next_command_us = completed + cycle->config.update_us;
        }
        int64_t endpoint = cycle->motion_start_us + cycle->config.motion_us;
        if (completed < endpoint && cycle->next_command_us > endpoint) {
            cycle->next_command_us = endpoint;
        }
    }
    return ESP_OK;
}

void business_servo_cycle_stop(business_servo_cycle_t *cycle)
{
    if (cycle != NULL) { cycle->state = APP_MOTION_STOPPED; }
}
