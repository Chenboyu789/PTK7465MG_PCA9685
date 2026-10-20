#pragma once

#include "pca9685.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    pca9685_handle_t pwm;
} board_hardware_t;

/** Pass a zero-initialized context. Owns the I2C bus and PCA9685 handle.
 * All channels remain OFF after initialization. No concurrent init/deinit.
 */
esp_err_t board_init(board_hardware_t *board);

/** Stop all clients first. On failure, context retains resources for retry. */
esp_err_t board_deinit(board_hardware_t *board);

#ifdef __cplusplus
}
#endif
