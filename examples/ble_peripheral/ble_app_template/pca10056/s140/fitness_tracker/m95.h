#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "boards_custom.h"     // pins + status_t live here
#include "nrf_uarte.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize GPIO + UARTE0 and start RX
void    m95_init(void);

// Power sequence (rail enable + PWRKEY pulse + settle)
bool    m95_power_on(void);
bool    m95_power_off(void);

// Send an AT command (CRLF is optional; we’ll append if missing)
bool    m95_send_cmd(const char *cmd);

// Non-blocking line pump: returns true when a complete modem line is ready
bool    m95_service(void);

// Copy last ready line into caller buffer. Returns true if a fresh line was copied.
bool    m95_get_line(char *out, size_t outlen);

// Optional control
void    m95_set_dtr(bool assert_level);

#ifdef __cplusplus
}
#endif
