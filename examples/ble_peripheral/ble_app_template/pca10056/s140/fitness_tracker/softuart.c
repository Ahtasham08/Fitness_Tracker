#include "softuart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "nrf_gpio.h"
#include "nrf_timer.h"
#include "app_util_platform.h"
#include "nrf.h"

// ---- Configuration ----
#define SOFTUART_TIMER              NRF_TIMER2
#define SOFTUART_TIMER_IRQn         TIMER2_IRQn
#define SOFTUART_IRQ_PRIORITY       6          // lower priority (higher number) to reduce jitter
#define SOFTUART_TX_BUF_SIZE        512        // bytes queued for transmit
#define SOFTUART_MAX_PRINTF         256

// Timer base: 1 MHz (1 tick = 1 us)
#define TIMER_BASE_PRESCALER        4          // 16MHz / 2^4 = 1MHz

// ---- State ----
static struct {
    uint32_t tx_pin;
    uint32_t baud;
    uint16_t ticks_per_bit;

    volatile uint8_t  sending;   // 0 idle, 1 active
    volatile uint8_t  bit_idx;   // 0=start, 1..8=data bits, 9=stop
    volatile uint8_t  cur_byte;

    // simple ring buffer
    volatile uint16_t widx;
    volatile uint16_t ridx;
    uint8_t  buf[SOFTUART_TX_BUF_SIZE];
} g_su = {0};

static inline bool _buf_empty(void) { return g_su.widx == g_su.ridx; }
static inline bool _buf_full(void)  { return (uint16_t)(g_su.widx + 1u) % SOFTUART_TX_BUF_SIZE == g_su.ridx; }
static inline void _buf_put(uint8_t c) { g_su.buf[g_su.widx] = c; g_su.widx = (uint16_t)(g_su.widx + 1u) % SOFTUART_TX_BUF_SIZE; }
static inline uint8_t _buf_get(void) { uint8_t c = g_su.buf[g_su.ridx]; g_su.ridx = (uint16_t)(g_su.ridx + 1u) % SOFTUART_TX_BUF_SIZE; return c; }

static void _timer_start_bitperiod(void) {
    // schedule next bit edge after ticks_per_bit
    uint32_t cc = SOFTUART_TIMER->CC[0] + g_su.ticks_per_bit;
    SOFTUART_TIMER->CC[0] = cc;
    SOFTUART_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
}

static void _start_next_byte(void) {
    if (_buf_empty()) { g_su.sending = 0; return; }
    g_su.cur_byte = _buf_get();
    g_su.bit_idx  = 0;
    g_su.sending  = 1;
    // Start bit (LOW)
    nrf_gpio_pin_clear(g_su.tx_pin);
    _timer_start_bitperiod();
}

bool softuart_init(const softuart_cfg_t* cfg) {
    if (!cfg || cfg->tx_pin == 0xFFFFFFFFu || cfg->baud == 0) return false;

    g_su.tx_pin = cfg->tx_pin;
    g_su.baud   = cfg->baud;

    // Compute ticks/bit at 1 MHz
    uint32_t tpb = (1000000u + (cfg->baud/2u)) / cfg->baud; // rounded
    if (tpb < 40) tpb = 40;   // guard (>=25 kbps not supported)
    g_su.ticks_per_bit = (uint16_t)tpb;

    // GPIO idle high
    nrf_gpio_cfg_output(g_su.tx_pin);
    nrf_gpio_pin_set(g_su.tx_pin);

    // Timer 2 at 1 MHz, free-running
    SOFTUART_TIMER->PRESCALER = TIMER_BASE_PRESCALER;
    SOFTUART_TIMER->MODE      = TIMER_MODE_MODE_Timer;
    SOFTUART_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    SOFTUART_TIMER->TASKS_CLEAR = 1;
    SOFTUART_TIMER->CC[0] = 0;
    SOFTUART_TIMER->SHORTS = 0;
    SOFTUART_TIMER->TASKS_START = 1;

    // NVIC
    NVIC_ClearPendingIRQ(SOFTUART_TIMER_IRQn);
    NVIC_SetPriority(SOFTUART_TIMER_IRQn, SOFTUART_IRQ_PRIORITY);
    NVIC_EnableIRQ(SOFTUART_TIMER_IRQn);

    g_su.widx = g_su.ridx = 0;
    g_su.sending = 0;

    return true;
}

void softuart_putc(uint8_t c) {
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (!_buf_full()) {
        _buf_put(c);
        if (!g_su.sending) {
            // Start transmission immediately
            SOFTUART_TIMER->TASKS_CAPTURE[1] = 1;            // snapshot timer into CC[1]
            SOFTUART_TIMER->CC[0] = SOFTUART_TIMER->CC[1];
            _start_next_byte();
        }
    }
    if (!key) __enable_irq();
}

void softuart_write(const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    for (size_t i=0; i<len; ++i) {
        softuart_putc(p[i]);
    }
}

void softuart_printf(const char* fmt, ...) {
    char tmp[SOFTUART_MAX_PRINTF];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    size_t out = (size_t)n;
    if (out > sizeof(tmp)) out = sizeof(tmp);
    softuart_write(tmp, out);
}

void softuart_flush(void) {
    while (g_su.sending || !_buf_empty()) { __WFE(); }
}

void TIMER2_IRQHandler(void) {
    if (SOFTUART_TIMER->EVENTS_COMPARE[0]) {
        SOFTUART_TIMER->EVENTS_COMPARE[0] = 0;

        // Advance one bit
        g_su.bit_idx++;
        if (g_su.bit_idx >= 1 && g_su.bit_idx <= 8) {
            // data bits LSB first
            uint8_t bit = (g_su.cur_byte >> (g_su.bit_idx - 1)) & 1u;
            if (bit) nrf_gpio_pin_set(g_su.tx_pin);
            else     nrf_gpio_pin_clear(g_su.tx_pin);
            _timer_start_bitperiod();
        } else if (g_su.bit_idx == 9) {
            // stop bit
            nrf_gpio_pin_set(g_su.tx_pin);
            _timer_start_bitperiod();
        } else {
            // end of frame
            if (_buf_empty()) {
                g_su.sending = 0;
                SOFTUART_TIMER->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
            } else {
                _start_next_byte();
            }
        }
    }
}
