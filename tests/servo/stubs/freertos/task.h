#pragma once
#include <stdint.h>
#define pdPASS 1
typedef void (*TaskFunction_t)(void *);
int xTaskCreate(TaskFunction_t task, const char *name, uint32_t stack, void *arg,
                unsigned priority, void **handle);
