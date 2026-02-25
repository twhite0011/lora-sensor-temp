#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t shtc3_init(void);
esp_err_t shtc3_read(float *temperature_c, float *humidity_rh);
void shtc3_deinit(void);
