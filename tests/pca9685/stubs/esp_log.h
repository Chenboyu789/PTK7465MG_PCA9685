#pragma once

/* Host protocol tests do not depend on ESP-IDF logging. */
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGE(tag, ...) ((void)(tag))
