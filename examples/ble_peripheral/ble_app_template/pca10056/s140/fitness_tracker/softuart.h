#pragma once
/**
 * SoftUART TX-only for nRF52 (SDK 17.x) — reliable at 9600..19200 baud.
 * - Uses TIMER2 interrupts to bit-bang one GPIO as 8N1.
 * - Idle level = HIGH, start bit = LOW.
 * - TX-only by design (for field debug via CH340/USB-UART).
 * - Very low footprint; avoids consuming UARTE0/1 (keep them for M95 & GPS).
 *
 * Limitations:
 * - ISR-driven; keep baud <= 19200 for safety.
 * - Not suitable for high-throughput logging. Prefer flash logging + drains.
 *
 * Wiring:
 *   MCU pin (SOFTUART_TX_PIN)  ->  CH340 RXD
 *   GND (common)
 *
 * Docklight / terminal settings: 9600 8N1, no flow control.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t tx_pin;        // GPIO number (use NRF_GPIO_PIN_MAP())
    uint32_t baud;          // e.g., 9600 or 19200
} softuart_cfg_t;

bool softuart_init(const softuart_cfg_t* cfg);
void softuart_putc(uint8_t c);
void softuart_write(const void* data, size_t len);
void softuart_printf(const char* fmt, ...);
void softuart_flush(void);           // block until idle

#ifdef __cplusplus
}
#endif
