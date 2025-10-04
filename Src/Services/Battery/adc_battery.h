#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "boards_custom.h"

// ---- Battery % thresholds (mV) ----
#ifndef VBAT_EMPTY_MV_DEFAULT
#define VBAT_EMPTY_MV_DEFAULT        2600
#endif
#ifndef VBAT_FULL_START_MV_DEFAULT
#define VBAT_FULL_START_MV_DEFAULT   3500
#endif
#ifndef VBAT_FULL_MAX_MV_DEFAULT
#define VBAT_FULL_MAX_MV_DEFAULT     4400
#endif

void     battery_gauge_set_thresholds(uint16_t empty_mv,
                                      uint16_t full_start_mv,
                                      uint16_t full_max_mv);
uint8_t  battery_percent_from_mv(uint16_t vbat_mv);

bool     battery_adc_init(void);
bool     battery_read_mv(uint16_t* vbat_mv);   // VBAT in millivolts
uint8_t  battery_percent_stable(uint16_t vbat_mv);
