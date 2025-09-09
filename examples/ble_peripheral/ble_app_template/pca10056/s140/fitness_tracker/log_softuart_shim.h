#pragma once
/**
 * nrf_log → SoftUART shim
 *
 * Include this header *after* any <nrf_log.h> includes to override NRF_LOG_* macros.
 * It maps INFO/WARNING/ERROR/DEBUG and HEXDUMP to the SoftUART TX-only prints.
 *
 * Requirements:
 *   - softuart.h / softuart.c added to project
 *   - Call debug_port_init() (provided below) early at boot
 *   - Docklight/terminal at 9600 8N1 (or your baud)
 *
 * Notes:
 *   - Adds "\r\n" automatically.
 *   - HEXDUMP prints 16 bytes per line.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "softuart.h"

#ifndef LOG_SHIM_BAUD
#define LOG_SHIM_BAUD   9600
#endif

#ifndef LOG_SHIM_TX_PIN
#warning "LOG_SHIM_TX_PIN not defined; defaulting to P0.29"
#define LOG_SHIM_TX_PIN NRF_GPIO_PIN_MAP(0,29)
#endif

static inline void debug_port_init(void) {
    softuart_cfg_t cfg = { .tx_pin = LOG_SHIM_TX_PIN, .baud = LOG_SHIM_BAUD };
    (void)softuart_init(&cfg);
    softuart_printf("DBG: SoftUART log online (%lu baud)\r\n", (unsigned long)LOG_SHIM_BAUD);
}

static inline void _softlog_puts(const char* lvl, const char* fmt, va_list ap) {
    char buf[256];
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) return;
    softuart_printf("%s %s\r\n", lvl, buf);
}

static inline void softlog_info(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); _softlog_puts("I:", fmt, ap); va_end(ap);
}
static inline void softlog_warn(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); _softlog_puts("W:", fmt, ap); va_end(ap);
}
static inline void softlog_error(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); _softlog_puts("E:", fmt, ap); va_end(ap);
}
static inline void softlog_debug(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); _softlog_puts("D:", fmt, ap); va_end(ap);
}

static inline void softlog_hexdump(const char* lvl, const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    char line[80];
    size_t i = 0;
    while (i < len) {
        size_t chunk = (len - i > 16) ? 16 : (len - i);
        int pos = 0;
        pos += snprintf(line+pos, sizeof(line)-pos, "%s ", lvl);
        for (size_t j=0;j<chunk;j++) {
            pos += snprintf(line+pos, sizeof(line)-pos, "%02X ", p[i+j]);
        }
        softuart_printf("%s\r\n", line);
        i += chunk;
    }
}

// Flush: wait for SoftUART to finish
static inline void softlog_flush(void) { softuart_flush(); }

// ---- Macro overrides ----
#ifdef NRF_LOG_INFO
#undef NRF_LOG_INFO
#endif
#ifdef NRF_LOG_WARNING
#undef NRF_LOG_WARNING
#endif
#ifdef NRF_LOG_ERROR
#undef NRF_LOG_ERROR
#endif
#ifdef NRF_LOG_DEBUG
#undef NRF_LOG_DEBUG
#endif
#ifdef NRF_LOG_HEXDUMP_INFO
#undef NRF_LOG_HEXDUMP_INFO
#endif
#ifdef NRF_LOG_HEXDUMP_WARNING
#undef NRF_LOG_HEXDUMP_WARNING
#endif
#ifdef NRF_LOG_HEXDUMP_ERROR
#undef NRF_LOG_HEXDUMP_ERROR
#endif
#ifdef NRF_LOG_FLUSH
#undef NRF_LOG_FLUSH
#endif

#define NRF_LOG_INFO(...)          softlog_info(__VA_ARGS__)
#define NRF_LOG_WARNING(...)       softlog_warn(__VA_ARGS__)
#define NRF_LOG_ERROR(...)         softlog_error(__VA_ARGS__)
#define NRF_LOG_DEBUG(...)         softlog_debug(__VA_ARGS__)

#define NRF_LOG_HEXDUMP_INFO(p,l)    softlog_hexdump("I:", (p), (l))
#define NRF_LOG_HEXDUMP_WARNING(p,l) softlog_hexdump("W:", (p), (l))
#define NRF_LOG_HEXDUMP_ERROR(p,l)   softlog_hexdump("E:", (p), (l))

#define NRF_LOG_FLUSH()            softlog_flush()
