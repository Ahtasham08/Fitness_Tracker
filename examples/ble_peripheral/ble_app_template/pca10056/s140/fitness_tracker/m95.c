#include <string.h>
#include "m95.h"

#include "nrfx_uarte.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "app_error.h"

#include "boards_custom.h"     // pins + status_t live here

                 
#define LOG_SHIM_TX_PIN  NRF_GPIO_PIN_MAP(0,6)   // CH340 RXD wire here
#define LOG_SHIM_BAUD    9600
#include "log_softuart_shim.h"            // redirects NRF_LOG_* → SoftUART
// +++ END ADD +++

// ---------- UARTE0 for M95 ----------
static const nrfx_uarte_t M95_UART = NRFX_UARTE_INSTANCE(0);

/*
// RX line framing
#define M95_RX_BUF_SZ   256
static uint8_t  m95_rx_buf[M95_RX_BUF_SZ];
static uint8_t  m95_line_buf[256];
static size_t   m95_line_idx = 0;
static volatile bool m95_line_ready = false;
static bool     m95_in_line = false;

*/

// --- top of file ---
#define M95_DMA_RX_SZ   1               // was 256; 1 ensures ISR per byte
static uint8_t  m95_rx_buf[M95_DMA_RX_SZ];

// keep your line buffer as-is
static uint8_t  m95_line_buf[256];
static size_t   m95_line_idx = 0;
static volatile bool m95_line_ready = false;
static bool     m95_in_line = false;


// TX completion flag
static volatile bool m95_tx_done = true;

// ------- tiny helpers -------
static inline void m95_gpio_defaults(void)
{
    // Power rail enable
    nrf_gpio_cfg_output(PIN_GSM_PWR_EN);
    nrf_gpio_pin_clear(PIN_GSM_PWR_EN);  // rail off

    // PWRKEY/EMERG/DTR (MCU HIGH asserts via transistor stages on your board)
    nrf_gpio_cfg_output(PIN_GSM_PWRKEY);
    nrf_gpio_cfg_output(PIN_GSM_EMERG);
    nrf_gpio_cfg_output(PIN_GSM_DTR);
    nrf_gpio_pin_clear(PIN_GSM_PWRKEY);  // deassert
    nrf_gpio_pin_clear(PIN_GSM_EMERG);   // deassert
    nrf_gpio_pin_clear(PIN_GSM_DTR);     // keep low (awake)

    // STATUS input (LOW at MCU = modem ON; inverted by your Q9 stage)
    nrf_gpio_cfg_input(PIN_GSM_STATUS, NRF_GPIO_PIN_NOPULL);
}

static void m95_uart_evt(nrfx_uarte_event_t const * e, void * ctx)
{
    (void)ctx;
    switch (e->type)
    {
    case NRFX_UARTE_EVT_RX_DONE: {
        // frame ASCII lines terminated by '\n'
        for (size_t i = 0; i < e->data.rxtx.bytes; i++) {
            uint8_t c = m95_rx_buf[i];

            if (c == '\r') continue;
            if (c == '\n') {
                // end of line
                if (m95_line_idx >= sizeof(m95_line_buf)) m95_line_idx = sizeof(m95_line_buf)-1;
                m95_line_buf[m95_line_idx] = 0;
                m95_line_ready = true;
                m95_in_line = false;
                m95_line_idx = 0;
                continue;
            }
            if (!m95_in_line) m95_in_line = true;
            if (m95_line_idx < sizeof(m95_line_buf)-1)
                m95_line_buf[m95_line_idx++] = c;
            else {
                // overflow → drop this line
                m95_in_line = false;
                m95_line_idx = 0;
            }
        }
        // continue RX
        (void)nrfx_uarte_rx(&M95_UART, m95_rx_buf, sizeof(m95_rx_buf));
    } break;

    case NRFX_UARTE_EVT_TX_DONE:
        m95_tx_done = true;
        break;

    case NRFX_UARTE_EVT_ERROR:
        // restart RX on error
        (void)nrfx_uarte_rx(&M95_UART, m95_rx_buf, sizeof(m95_rx_buf));
        break;

    default:
        break;
    }
}

void m95_init(void)
{
    m95_gpio_defaults(); // make sure this sets: PWR_EN=0, PWRKEY=0, EMERG=0, STATUS as input+PU

    // You don't need to set TX as GPIO output or RX as input here;
    // UARTE will take ownership. But it's harmless if you keep it.
    // nrf_gpio_cfg_output(M95_UART_TX);
    // nrf_gpio_cfg_input(M95_UART_RX, NRF_GPIO_PIN_NOPULL);

    nrfx_uarte_config_t cfg = NRFX_UARTE_DEFAULT_CONFIG;


    // --- NORMAL mapping ---
   // cfg.pseltxd = M95_UART_TX;  // MCU → M95
   // cfg.pselrxd = M95_UART_RX;  // M95 → MCU

    // --- SWAP TEST (TEMP): uncomment this block, rebuild, flash ---
     cfg.pseltxd = M95_UART_RX;  // SWAP
     cfg.pselrxd = M95_UART_TX;  // SWAP

    cfg.pselrts = M95_UART_RTS;             // DISCONNECTED is fine
    cfg.pselcts = M95_UART_CTS;             // DISCONNECTED is fine
    cfg.hwfc    = NRF_UARTE_HWFC_DISABLED;
    cfg.parity  = NRF_UARTE_PARITY_EXCLUDED;
    cfg.baudrate = NRF_UARTE_BAUDRATE_115200;  // start fast; fallback handled in bring-up
    cfg.interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY;

    APP_ERROR_CHECK(nrfx_uarte_init(&M95_UART, &cfg, m95_uart_evt));
    APP_ERROR_CHECK(nrfx_uarte_rx(&M95_UART, m95_rx_buf, sizeof(m95_rx_buf)));
}








// --- helpers for autobaud (no app_timer needed) ---
static void m95_uarte_reinit(nrf_uarte_baudrate_t baud)
{
    nrfx_uarte_uninit(&M95_UART);

    nrfx_uarte_config_t cfg = NRFX_UARTE_DEFAULT_CONFIG;

    // --- SWAP TEST (same as in m95_init) ---
    cfg.pseltxd = M95_UART_RX;   // SWAP
    cfg.pselrxd = M95_UART_TX;   // SWAP

    // If you go back to normal later, change both places together:
    // cfg.pseltxd = M95_UART_TX;
    // cfg.pselrxd = M95_UART_RX;

    cfg.pselrts = M95_UART_RTS;
    cfg.pselcts = M95_UART_CTS;
    cfg.hwfc    = NRF_UARTE_HWFC_DISABLED;
    cfg.parity  = NRF_UARTE_PARITY_EXCLUDED;
    cfg.baudrate = baud;
    cfg.interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY;

    APP_ERROR_CHECK(nrfx_uarte_init(&M95_UART, &cfg, m95_uart_evt));
    APP_ERROR_CHECK(nrfx_uarte_rx(&M95_UART, m95_rx_buf, sizeof m95_rx_buf));
}


// Poll for an exact "OK" for up to timeout_ms (uses nrf_delay_ms)
static bool m95_wait_for_ok(uint32_t timeout_ms)
{
    char ln[160];
    const uint32_t step_ms = 10;
    uint32_t waited = 0;

    while (waited < timeout_ms) {
        while (m95_service()) {
            if (m95_get_line(ln, sizeof ln)) {
                NRF_LOG_INFO("M95> %s", nrf_log_push(ln));
                if (strcmp(ln, "OK") == 0) return true;
            }
        }
        nrf_delay_ms(step_ms);
        waited += step_ms;
    }
    return false;
}

// Reinit UART at 'baud', fire "AT" attempts for autobaud, look for OK
static bool m95_try_probe(nrf_uarte_baudrate_t baud, uint8_t attempts, uint16_t gap_ms)
{
    m95_uarte_reinit(baud);
    for (uint8_t i = 0; i < attempts; i++) {
        (void)m95_send_cmd("AT");     // sends \r\n in your implementation
        if (m95_wait_for_ok(250)) return true;
        nrf_delay_ms(gap_ms);
    }
    return false;
}
















bool m95_power_on(void)
{
    // Idle defaults
    nrf_gpio_pin_clear(PIN_GSM_EMERG);
    nrf_gpio_pin_clear(PIN_GSM_DTR);       // awake (LOW on your board)

    // Rail on + settle
    nrf_gpio_pin_set(PIN_GSM_PWR_EN);
    nrf_delay_ms(2000);

    // Press PWRKEY (MCU HIGH => module LOW via transistor)
    NRF_LOG_INFO("M95: PWRKEY HIGH 2000 ms");
    nrf_gpio_pin_set(PIN_GSM_PWRKEY);
    nrf_delay_ms(2000);
    nrf_gpio_pin_clear(PIN_GSM_PWRKEY);
    NRF_LOG_INFO("M95: PWRKEY released");

    // Wait up to ~8 s for STATUS LOW (LOW@MCU means ON)
    bool on = false;
    for (int i = 0; i < 160; i++) {            // 160 * 50ms ≈ 8 s
        if (nrf_gpio_pin_read(PIN_GSM_STATUS) == 0) { on = true; break; }
        nrf_delay_ms(50);
    }
    NRF_LOG_INFO("M95: STATUS=%d (LOW@MCU means ON)", nrf_gpio_pin_read(PIN_GSM_STATUS));
    if (!on) {
        NRF_LOG_ERROR("M95: STATUS never asserted. Check PWRKEY hold/polarity & EMERG.");
        nrf_gpio_pin_clear(PIN_GSM_PWR_EN);
        return false;
    }

    // Autobaud probe: 115200 then 9600  (helpers shown below)
    if (!m95_try_probe(NRF_UARTE_BAUDRATE_115200, 12, 50)) {
        NRF_LOG_WARNING("M95: No OK at 115200; trying 9600");
        if (!m95_try_probe(NRF_UARTE_BAUDRATE_9600, 12, 50)) {
            NRF_LOG_ERROR("M95: No AT response at either baud.");
            return false;
        }
    }

    // Make future boots deterministic
    (void)m95_send_cmd("ATE0");        (void)m95_wait_for_ok(500);
    (void)m95_send_cmd("AT+IPR=9600"); (void)m95_wait_for_ok(500);
    (void)m95_send_cmd("AT&W");        (void)m95_wait_for_ok(500);

    (void)m95_send_cmd("ATI");         // identification; lines will arrive via ISR
    return true;
}







bool m95_power_off(void)
{
    // Emergency off (optional): EMERG HIGH briefly
    nrf_gpio_pin_set(PIN_GSM_EMERG);
    nrf_delay_ms(120);
    nrf_gpio_pin_clear(PIN_GSM_EMERG);

    // Rail off
    nrf_gpio_pin_clear(PIN_GSM_PWR_EN);
    nrf_delay_ms(50);
    return true;
}

bool m95_send_cmd(const char *cmd)
{
    if (!cmd) return false;

    // Compose "cmd\r\n" if not already CR/LF terminated
    char buf[192];
    size_t n = strlen(cmd);
    if (n >= sizeof(buf)-2) n = sizeof(buf)-3;
    memcpy(buf, cmd, n);
    if (n == 0 || buf[n-1] != '\n') {
        buf[n++] = '\r';
        buf[n++] = '\n';
    }
    buf[n] = 0;

    m95_tx_done = false;
    nrfx_err_t err = nrfx_uarte_tx(&M95_UART, (uint8_t*)buf, (uint16_t)n);
    if (err != NRFX_SUCCESS) return false;

    // simple wait (short; modem echoes fast)
    uint32_t guard = 0;
    while (!m95_tx_done && ++guard < 200000) { /* ~few ms spin */ }
    return m95_tx_done;
}

bool m95_service(void)
{
    // Just report whether a line is waiting; the UART ISR sets the flag
    return m95_line_ready;
}

bool m95_get_line(char *out, size_t outlen)
{
    if (!m95_line_ready || !out || outlen == 0) return false;
    // copy once
    size_t n = strnlen((const char*)m95_line_buf, sizeof(m95_line_buf));
    if (n >= outlen) n = outlen - 1;
    memcpy(out, m95_line_buf, n);
    out[n] = 0;
    m95_line_ready = false;
    return true;
}

void m95_set_dtr(bool assert_level)
{
    if (assert_level) nrf_gpio_pin_set(PIN_GSM_DTR);
    else              nrf_gpio_pin_clear(PIN_GSM_DTR);
}
