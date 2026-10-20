#include "board.h"
#include "board_config.h"
#include "esp_log.h"
#include <stdbool.h>

static const char *TAG = "board";

esp_err_t board_init(board_hardware_t *board)
{
    if (board == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (board->i2c_bus != NULL || board->pwm != NULL) {
        ESP_LOGE(TAG, "Context already initialized: bus=%p pwm=%p",
                 (void *)board->i2c_bus, (void *)board->pwm);
        return ESP_ERR_INVALID_STATE;
    }
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_LOGI(TAG, "Creating I2C bus: port=%d SDA=%d SCL=%d",
             BOARD_I2C_PORT, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
    esp_err_t err = i2c_new_master_bus(&bus_config, &board->i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }
    const pca9685_config_t pwm_config = {
        .address = BOARD_PCA9685_ADDRESS,
        .i2c_speed_hz = BOARD_I2C_SPEED_HZ,
        .oscillator_hz = BOARD_PCA9685_OSCILLATOR_HZ,
        .frequency_hz = BOARD_PCA9685_FREQUENCY_HZ,
        .timeout_ms = BOARD_I2C_TIMEOUT_MS,
    };
    err = pca9685_create(board->i2c_bus, &pwm_config, &board->pwm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_create failed (address=0x%02X): %s",
                 BOARD_PCA9685_ADDRESS, esp_err_to_name(err));
        if (i2c_del_master_bus(board->i2c_bus) == ESP_OK) {
            board->i2c_bus = NULL;
        }
    }
    return err;
}

esp_err_t board_deinit(board_hardware_t *board)
{
    if (board == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (board->pwm != NULL) {
        esp_err_t err = pca9685_delete(board->pwm);
        if (err != ESP_OK) {
            return err;
        }
        board->pwm = NULL;
    }
    if (board->i2c_bus != NULL) {
        esp_err_t err = i2c_del_master_bus(board->i2c_bus);
        if (err != ESP_OK) {
            return err;
        }
        board->i2c_bus = NULL;
    }
    return ESP_OK;
}
