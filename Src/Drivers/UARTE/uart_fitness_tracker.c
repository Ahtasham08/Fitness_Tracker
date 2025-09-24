
#include "boards_custom.h"
#include "COMMON_CONFIG.h"
#include "uart_fitness_tracker.h"
#include "nrf_queue.h"
#include "nrfx_timer.h"
#include "nrf_libuarte_async.h"

NRF_LIBUARTE_ASYNC_DEFINE(gsm_uarte, 0, 2, 0, NRF_LIBUARTE_PERIPHERAL_NOT_USED, 255, 5);
nrfx_uarte_t gps_uarte = NRFX_UARTE_INSTANCE(1);

static uint8_t gsm_rx_buf[UART_RX_BUF_SIZE];
static uint8_t gps_rx_buf[UART_RX_BUF_SIZE];
static volatile bool gsm_tx_idle = true;
static uint8_t tx_buf_a[64], tx_buf_b[64];
static uint8_t *tx_cur = tx_buf_a, *tx_next = tx_buf_b;

// Safe TX helper (copies into a dedicated buffer and gates on TX_DONE)
bool gsm_tx_safe(const uint8_t *p, size_t n)
{
    if (!p || n == 0 || n > sizeof(tx_buf_a))
        return false;

    // wait until previous TX has completed
    while (!gsm_tx_idle)
    {
        __WFE();
    } // or small nrf_delay_ms(1)

    // swap buffers so the one DMA is using isn't modified
    uint8_t *send_buf = tx_cur;
    tx_cur = tx_next;
    tx_next = send_buf;

    memcpy(send_buf, p, n);

    gsm_tx_idle = false;
    ret_code_t rc = nrf_libuarte_async_tx(&gsm_uarte, send_buf, n);
    if (rc == NRF_ERROR_BUSY)
    { // shouldn't happen due to gate, but be safe
        gsm_tx_idle = true;
        return false;
    }
    APP_ERROR_CHECK(rc);
    return true; // completion signaled by TX_DONE
}


typedef struct
{
    uint8_t *p_data;
    uint32_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue, 10, NRF_QUEUE_MODE_NO_OVERFLOW);

void (*callback_gsm)(char *, uint8_t);
void (*callback_gps)(char *, uint8_t);

/* ===================================================
   Callback Handler
   =================================================== */
// GSM Event Handler
static void gsm_uart_event_handler(void *context, nrf_libuarte_async_evt_t *p_evt)
{
    nrf_libuarte_async_t *p_libuarte = (nrf_libuarte_async_t *)context;
    ret_code_t ret;

    switch (p_evt->type)
    {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
        break;
    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
        // Copy chunk for parsing; user callback (if any) still receives pointer view
        // length-aware logs
        NRF_LOG_INFO("GSM RX %dB", (int)p_evt->data.rxtx.length);
        memset(gsm_rx_buf, 0, sizeof(gsm_rx_buf));
        memcpy(gsm_rx_buf, (const char *)p_evt->data.rxtx.p_data, (int)p_evt->data.rxtx.length);

        NRF_LOG_INFO("GSM Received: %s", gsm_rx_buf);

        // hand off to higher layer (parser)
        if (callback_gsm)
        {
            callback_gsm(gsm_rx_buf, (uint8_t)p_evt->data.rxtx.length);
        }
        break;
    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        gsm_tx_idle = true;
        NRF_LOG_INFO("GSM TX Done");
        NRF_LOG_FLUSH();

        break;
    default:
        break;
    }
}

// GPS Event Handler
static void gps_uart_event_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
    switch (p_event->type)
    {
    case NRFX_UARTE_EVT_RX_DONE:
    {
        if (p_event->data.rxtx.bytes < UART_RX_BUF_SIZE)
            gps_rx_buf[p_event->data.rxtx.bytes] = '\0';
        NRF_LOG_INFO("%s", (uint32_t)gps_rx_buf);

        // Re-start reception for next data
        APP_ERROR_CHECK(nrfx_uarte_rx(&gps_uarte, gps_rx_buf, UART_RX_BUF_SIZE));
        break;
    }

    case NRFX_UARTE_EVT_TX_DONE:
        NRF_LOG_ERROR("GPS UART error: 0x%08x", p_event->data.error.error_mask);
        // Clear errors and restart reception
        APP_ERROR_CHECK(nrfx_uarte_rx(&gps_uarte, gps_rx_buf, UART_RX_BUF_SIZE));
   
        break;

    case NRFX_UARTE_EVT_ERROR:
        NRF_LOG_ERROR("GPS UART error: 0x%X", p_event->data.error.error_mask);
        nrfx_uarte_rx(&gps_uarte, gps_rx_buf, sizeof(gps_rx_buf));
        break;

    default:
        break;
    }
}

// Init GSM UARTE
void gsm_uart_init(void (*callback)(char *, uint8_t))
{
    callback_gsm = callback;
    // Global buffer (goes to RAM)
    nrf_libuarte_async_config_t gsm_libuarte_async_config = {
        .tx_pin = GSM_UART_TX,
        .rx_pin = GSM_UART_RX,
        .baudrate = NRF_UARTE_BAUDRATE_115200,
        .parity = NRF_UARTE_PARITY_EXCLUDED,
        .hwfc = NRF_UARTE_HWFC_DISABLED,
        .cts_pin = NRF_UARTE_PSEL_DISCONNECTED, // CRITICAL: Disable CTS
        .rts_pin = NRF_UARTE_PSEL_DISCONNECTED, // CRITICAL: Disable RTS
        .timeout_us = 2000,
        .int_prio = APP_IRQ_PRIORITY_MID};

    ret_code_t err_code = nrf_libuarte_async_init(&gsm_uarte, &gsm_libuarte_async_config, gsm_uart_event_handler, (void *)&gsm_uarte);

    APP_ERROR_CHECK(err_code);

    nrf_libuarte_async_enable(&gsm_uarte);

    NRF_LOG_INFO("gsm uarte initialized");
}

// Init GPS UARTE
void gps_uart_init(void (*callback)(char *, uint8_t))
{
    nrfx_uarte_config_t gps_config = {
        .pseltxd = GPS_UART_TX,
        .pselrxd = GPS_UART_RX,
        .pselcts = NRF_UARTE_PSEL_DISCONNECTED,
        .pselrts = NRF_UARTE_PSEL_DISCONNECTED,
        .p_context = NULL,
        .hwfc = NRF_UARTE_HWFC_DISABLED,
        .parity = NRF_UARTE_PARITY_EXCLUDED,
        .baudrate = NRF_UARTE_BAUDRATE_9600,
        .interrupt_priority = NRFX_UARTE_DEFAULT_CONFIG_IRQ_PRIORITY};

    ret_code_t err_code = nrfx_uarte_init(&gps_uarte, &gps_config, gps_uart_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_uarte_rx(&gps_uarte, gps_rx_buf, 2);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("GPS UART initialized");
}

/**
 * @brief Initialize the UARTE driver with the specified configuration.
 *
 * @param nothing
 * @return true if initialization is successful, false otherwise.
 */
bool uarte_init(uint8_t uart_type, void(*callback)(char*,uint8_t))
{
    ret_code_t err_code = NRFX_SUCCESS;
    switch (uart_type)
    {
    case GSM_UART:
        gsm_uart_init(callback); // callback set later
        break;
    case GPS_UART:
        gps_uart_init(callback); // callback set later
        break;
    default:
        break;
    }

    return true;
}

ret_code_t uarte_abort(uint8_t uart_type)
{
    ret_code_t err_code = NRFX_SUCCESS;

    switch (uart_type)
    {
    case GSM_UART:
        // err_code = nrf_libuarte_async_tx_abort(&gsm_uarte);
        // APP_ERROR_CHECK(err_code);
        break;
    case GPS_UART:
        nrfx_uarte_tx_abort(&gps_uarte);
        break;
    default:
        break;
    }

    return err_code;
}
/**
 * @brief Transmit data over UART.
 *
 * @param data Pointer to the data buffer to be transmitted.
 * @param length Length of the data buffer.
 * @return true if transmission is successful, false otherwise.
 */
ret_code_t uarte_transmit(uint8_t uart_type, const uint8_t *p_data, size_t length)
{
    ret_code_t err_code;

    switch (uart_type)
    {
    case GSM_UART:
        gsm_tx_safe(p_data, length);
        break;
    case GPS_UART:
        err_code = nrfx_uarte_tx(&gps_uarte, p_data, length);
        APP_ERROR_CHECK(err_code);
        break;
    default:
        break;
    }

    return err_code;
}

/**
 * @brief Receive data over UART.
 *
 * @param buffer Pointer to the buffer to store received data.
 * @param length Length of the buffer.
 * @return true if reception is successful, false otherwise.
 */
bool uarte_receive(uint8_t *buffer, size_t length);



/**
 * @brief Deinitialize the UARTE driver.
 */
void uarte_deinit(uint8_t uart_type)
{
    switch (uart_type)
    {
    case GSM_UART:
        nrf_libuarte_async_uninit(&gsm_uarte);
        break;
    case GPS_UART:
        nrfx_uarte_uninit(&gps_uarte);
        break;
    default:
        break;
    }
}

void lfclk_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    // Wait for LFCLK to start
    while (!nrf_drv_clock_lfclk_is_running())
    {
        // Wait
    }
    NRF_LOG_INFO("LFCLK started");
}