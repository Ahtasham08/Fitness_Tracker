#ifndef COMMON_MACRO_FITNESS_TRACKER_H
#define COMMON_MACRO_FITNESS_TRACKER_H

#include "sdk_config.h"
#include "nrf.h"
#include "nrfx.h"          // must precede specific nrfx peripheral headers
#include "nrfx_twim.h"
#include "nrfx_rtc.h"
#include "nrf_gpio.h"   // defines NRF_GPIO_PIN_MAP(...)
#include "nrf.h"        // (usually pulled by nrf_gpio.h, but safe to include)


// ----- Choose ONE board header you actually use -----
#include "boards_custom.h" // or: #include "boards.h"

// C std / utils
#include <math.h>
#include <inttypes.h>

// App libs (nrfx-friendly ones)
#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"
#include "nrf_delay.h"

#ifdef SOFTDEVICE_PRESENT
  #include "nrf_sdh.h"
  #include "nrf_sdh_ble.h"
  #include "nrf_fstorage_sd.h"
  // Do NOT init LFCLK via nrfx_clock here; SoftDevice owns it.
#else
  #include "nrfx_clock.h"          // <-- replace nrf_drv_clock.h
  #include "nrf_fstorage_nvmc.h"
  // You must init LFCLK yourself with nrfx_clock when SD is absent.
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Replace app_uart with one of these (pick ONE):
// #include "nrfx_uarte.h"          // simple, low-level
// #include "nrf_libuarte_async.h"  // higher-level, nrfx-based helper

#include "cJSON.h"

// Pins / debug
#define LOG_SHIM_TX_PIN  NRF_GPIO_PIN_MAP(0,6)
#define LOG_SHIM_BAUD    9600

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FIRMWARE_VERSION TOSTRING(MAJOR_VERSION) "." TOSTRING(MINOR_VERSION) "." TOSTRING(BUILD_VERSION)

#define DEBUG_DUMMY_DATA        0
#define DEBUG_BLE_OFF           0
#define DEBUG_FLASH_LOGIC       0
#define DEBUG_CONFIGURE_DEVICE  1

#endif
