idf_component_register(
    SRCS "app.c" "app_profile.c" "app_scheduler.c" "business_servo_cycle.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES esp_common
    PRIV_REQUIRES board servo freertos esp_timer log
)
