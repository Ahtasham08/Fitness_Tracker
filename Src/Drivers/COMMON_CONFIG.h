#ifndef COMMON_MACRO_FITNESS_TRACKER_H
#define COMMON_MACRO_FITNESS_TRACKER_H

#include "nrf.h"
#include "nordic_common.h"
#include "boards_custom.h"

#include <math.h>

#include "nrf_drv_uart.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrfx_rtc.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "app_error.h"
#include "nrf_delay.h"

#include "boards.h"
#include <inttypes.h>


#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_drv_clock.h"
#include "nrf_fstorage_nvmc.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_wdt.h"
#include "nrf_log_default_backends.h"

#include "cJSON.h"


// Helper macros for stringification
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Create the firmware version string
#define FIRMWARE_VERSION TOSTRING(MAJOR_VERSION) "." TOSTRING(MINOR_VERSION) "." TOSTRING(BUILD_VERSION)

#define DEBUG_DUMMY_DATA 0
#define DEBUG_BLE_OFF 0
#define DEBUG_FLASH_LOGIC 0
#define DEBUG_CONFIGURE_DEVICE 1

#endif