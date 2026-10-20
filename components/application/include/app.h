#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_BUSINESS_NONE = 0,
    APP_BUSINESS_SERVO_CYCLE,
} app_business_id_t;

typedef enum {
    APP_MOTION_STOPPED = 0,
    APP_MOTION_HOLD_START,
    APP_MOTION_TO_END,
    APP_MOTION_HOLD_END,
    APP_MOTION_TO_START,
} app_motion_state_t;

typedef enum {
    APP_LIFECYCLE_UNINITIALIZED = 0,
    APP_LIFECYCLE_INITIALIZING,
    APP_LIFECYCLE_READY,
    APP_LIFECYCLE_CLEANUP_REQUIRED,
} app_lifecycle_t;

typedef struct {
    app_lifecycle_t lifecycle;
    app_business_id_t active;
    app_motion_state_t motion;
    esp_err_t last_error;      /* Latest failure, cleared by successful explicit selection/start. */
    esp_err_t shutdown_error;  /* Latest scheduler all-OFF result. */
    esp_err_t cleanup_error;   /* Latest startup resource cleanup result. */
} app_status_t;

/** Task-context snapshot protected by a short critical section; no hardware I/O.
 * Reports last completed owner-task operation, not queued requests.
 */
esp_err_t app_get_status(app_status_t *status);

/** Retry failed startup cleanup only. Serialize with app_start(); not a running
 * application shutdown API. Retains resources again if cleanup fails.
 */
esp_err_t app_retry_cleanup(void);

/** Initialize hardware and create the sole business/servo-owner task.
 * Starts the servo-cycle business by default. Call from one startup task only;
 * success means the task was created, not that its first PWM write succeeded.
 * Repeated start while initialized returns INVALID_STATE.
 */
esp_err_t app_start(void);

/** Thread-safe asynchronous selection after app_start() succeeds.
 * NONE stops the current business and disables all servo outputs. The old
 * business is stopped and outputs must be disabled successfully before a new
 * business may start. Re-selecting the active business does not restart it.
 * ESP_OK means queued, not completed; execution failures are logged by app.c.
 * A full queue returns TIMEOUT without changing the queued requests.
 */
esp_err_t app_select_business(app_business_id_t business);

#ifdef __cplusplus
}
#endif
