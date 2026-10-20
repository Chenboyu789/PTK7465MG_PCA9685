#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define taskENTER_CRITICAL(mux) ((void)(mux))
#define taskEXIT_CRITICAL(mux) ((void)(mux))
