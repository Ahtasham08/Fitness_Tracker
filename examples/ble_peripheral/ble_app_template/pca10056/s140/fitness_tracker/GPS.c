
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "GPS.h"
#include "boards_custom.h"
#include "app_error.h"
#include "nrfx_uarte.h"






#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#include "nrf_gpio.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_delay.h"    // optional if you use delay



#include "nrf_gpio.h"
#include "nrf_delay.h"


#include "GPS.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <time.h>


#define LOG_SHIM_TX_PIN  NRF_GPIO_PIN_MAP(0,6)   // CH340 RXD wire here
#define LOG_SHIM_BAUD    9600
#include "log_softuart_shim.h"            // redirects NRF_LOG_* → SoftUART
// +++ END ADD +++

// ---------- helpers ----------


// Debug: print the last raw NMEA sentence
void gps_debug_raw(void) {
    if (!line_ready) return;   // nothing new
    line_ready = false;        // consume

    char raw[256];
    strncpy(raw, (char*)line_buf, sizeof(raw)-1);
    raw[sizeof(raw)-1] = 0;

    NRF_LOG_INFO("RAW GPS: %s", nrf_log_push(raw));
}


static int split_csv(char *s, char *tok[], int max_tok) {
    int n = 0;
    for (char *p = strtok(s, ","); p && n < max_tok; p = strtok(NULL, ",")) tok[n++] = p;
    return n;
}

static double dm_to_deg(const char *dm, char hemi) {
    if (!dm || !*dm) return 0.0;
    double v = atof(dm);
    int deg = (int)(v / 100.0);
    double minutes = v - deg * 100.0;
    double out = deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 's' || hemi == 'W' || hemi == 'w') out = -out;
    return out;
}

static bool is_type(const char *s, const char *type3) {
    // Accept any talker prefix (GP/GN/GA/GB/...); check chars 3..5
    return (s && type3 && strncmp(s + 3, type3, 3) == 0);
}





// --- DMS helpers (no libm needed) ---
static inline long double _absld(long double x){ return (x < 0) ? -x : x; }

uint8_t gps_get_dms_degrees(long double dd) {
    long double v = _absld(dd);
    return (uint8_t)v; // trunc toward zero is fine since v >= 0
}

uint8_t gps_get_dms_minutes(long double dd) {
    long double v = _absld(dd);
    uint8_t deg = (uint8_t)v;
    long double frac_deg = v - (long double)deg;
    return (uint8_t)(frac_deg * 60.0L); // trunc is floor for positive
}

uint8_t gps_get_dms_seconds(long double dd) {
    long double v = _absld(dd);
    uint8_t deg = (uint8_t)v;
    long double frac_deg = v - (long double)deg;
    long double total_min = frac_deg * 60.0L;
    uint8_t min = (uint8_t)total_min;
    long double frac_min = total_min - (long double)min;
    uint8_t sec = (uint8_t)(frac_min * 60.0L + 0.5L); // round to nearest
    if (sec == 60) sec = 59; // clamp
    return sec;
}

// Convenience: compute D/M/S + hemisphere in one go
void gps_decimal_to_dms(long double dd,
                        uint8_t *D, uint8_t *M, uint8_t *S,
                        char *hem, char posHem, char negHem) {
    char h = (dd >= 0) ? posHem : negHem;
    long double v = (dd >= 0) ? dd : -dd;

    uint8_t d = (uint8_t)v;
    long double rem = (v - (long double)d) * 60.0L;
    uint8_t m = (uint8_t)rem;
    uint8_t s = (uint8_t)((rem - (long double)m) * 60.0L + 0.5L);

    if (s == 60) { s = 0; m++; }
    if (m == 60) { m = 0; d++; }

    if (D) *D = d; if (M) *M = m; if (S) *S = s; if (hem) *hem = h;
}



// ---------- working state we’ll merge into my_gps_data ----------
typedef struct {
    bool   rmc_valid;
    double lat_deg;
    double lon_deg;
    char   lat_hem;
    char   lon_hem;
    float  speed_kn;
    float  cog_deg;
    int    fix_quality;
    int    sats;
    float  hdop;
    float  alt_m;
    uint16_t hh, mm, ss;      // add time
    uint32_t t_rmc_ms;
    uint32_t t_gga_ms;
} gps_work_t;


static gps_work_t gw = {0};

// app timer ticks → ms (adjust if you prefer another timebase)
static uint32_t now_ms(void) {
    // Convert RTC ticks to ms assuming 32.768 kHz; replace with your utility if you have one
    uint32_t ticks = app_timer_cnt_get();
    // Avoid float here; simple scale for RTC at 32768 Hz:
    return (uint32_t)((ticks * 1000ULL) / 32768ULL);
}

// parse $--RMC (time/valid/position/speed/cog/date)
static bool parse_rmc_line(char *line) {
    char buf[256]; strncpy(buf, line, sizeof(buf)-1); buf[255] = 0;
    char *t[16] = {0}; int n = split_csv(buf, t, 16);
    if (n < 10) return false;

    // time hhmmss
    if (t[1] && strlen(t[1]) >= 6) {
        gw.hh = (t[1][0]-'0')*10 + (t[1][1]-'0');
        gw.mm = (t[1][2]-'0')*10 + (t[1][3]-'0');
        gw.ss = (t[1][4]-'0')*10 + (t[1][5]-'0');
    }

    gw.rmc_valid = (t[2] && t[2][0] == 'A');
    if (gw.rmc_valid) {
        gw.lat_deg = dm_to_deg(t[3], t[4] ? t[4][0] : 'N');
        gw.lon_deg = dm_to_deg(t[5], t[6] ? t[6][0] : 'E');
        gw.lat_hem = t[4] ? t[4][0] : 'N';
        gw.lon_hem = t[6] ? t[6][0] : 'E';
        gw.speed_kn = t[7] ? (float)atof(t[7]) : 0.f;
        gw.cog_deg  = t[8] ? (float)atof(t[8]) : 0.f;
    }
    gw.t_rmc_ms = now_ms();
    return true;
}

// parse $--GGA (fix quality/sats/hdop/alt)
static bool parse_gga_line(char *line) {
    char buf[256]; strncpy(buf, line, sizeof(buf)-1); buf[255] = 0;
    char *t[16] = {0}; int n = split_csv(buf, t, 16);
    // $--GGA,time,lat,N,lon,E,6fix,7sats,8hdop,9alt,M,geoid,...
    if (n < 10) return false;

    gw.fix_quality = (t[6] && *t[6]) ? atoi(t[6]) : 0;
    gw.sats        = (t[7] && *t[7]) ? atoi(t[7]) : 0;
    gw.hdop        = (t[8] && *t[8]) ? (float)atof(t[8]) : 99.f;
    gw.alt_m       = (t[9] && *t[9]) ? (float)atof(t[9]) : 0.f;

    gw.t_gga_ms = now_ms();
    return true;
}



// decide when we have a coherent fix (fresh RMC + GGA, good quality)
static bool fix_complete(void) {
    uint32_t t = now_ms();
    bool fresh_rmc = (t - gw.t_rmc_ms) < 1500;  // ~1.5 s window at 1 Hz
    bool fresh_gga = (t - gw.t_gga_ms) < 1500;
    return gw.rmc_valid && (gw.fix_quality > 0) && fresh_rmc && fresh_gga;
}














/* GPS UART instance */
static const nrfx_uarte_t GPS_UART = NRFX_UARTE_INSTANCE(1);

gps_packet_t my_gps_data = {0};   // <-- provide the definitio

#define GPS_RX_BUF_SIZE 256
// existing globals you already have:
static uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];   // 256
uint8_t  line_buf[256];
volatile bool line_ready = false;
static size_t line_idx = 0;
static bool in_sentence = false;

static void gps_uarte_event_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
    case NRFX_UARTE_EVT_RX_DONE:
        // iterate the bytes that just arrived in gps_rx_buf
        for (size_t i = 0; i < p_event->data.rxtx.bytes; i++) {
            uint8_t c = gps_rx_buf[i];

            if (c == '$') {                   // start of a sentence
                in_sentence = true;
                line_idx = 0;
                line_buf[line_idx++] = c;
                continue;
            }
            if (!in_sentence) continue;       // ignore noise (e.g., "<info> ...") until '$'

            if (c == '\r') continue;          // skip CR
            if (c == '\n') {                  // end of sentence
                line_buf[(line_idx < sizeof(line_buf)) ? line_idx : sizeof(line_buf)-1] = 0;
                line_ready = true;            // hand to driver-level parser
                in_sentence = false;
                continue;
            }
            if (line_idx < sizeof(line_buf)-1) line_buf[line_idx++] = c;
            else { in_sentence = false; line_idx = 0; } // overflow, resync
        }

        // re-start reception for next chunk (same buffer & size as your code)
        APP_ERROR_CHECK(nrfx_uarte_rx(&GPS_UART, gps_rx_buf, sizeof(gps_rx_buf)-1));
        break;

    case NRFX_UARTE_EVT_ERROR:
        // Clear errors and restart reception
        APP_ERROR_CHECK(nrfx_uarte_rx(&GPS_UART, gps_rx_buf, sizeof(gps_rx_buf)-1));
        break;

    default:
        break;
    }
}





static bool fix_ready = false;

// --- put near the top of GPS.c ---
static uint8_t hex_nibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    c = (char)(c & ~0x20);                 // to upper-case
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0xFF;                           // invalid
}

static bool nmea_checksum_ok(const char *s) {
    if (!s || s[0] != '$') return false;
    const char *ast = strrchr(s, '*');
    if (!ast || !ast[1] || !ast[2]) return false;

    uint8_t cs = 0;
    for (const char *p = s + 1; p < ast; ++p) cs ^= (uint8_t)*p;

    uint8_t hi = hex_nibble(ast[1]);
    uint8_t lo = hex_nibble(ast[2]);
    if (hi == 0xFF || lo == 0xFF) return false;
    return cs == (uint8_t)((hi << 4) | lo);
}



bool gps_service(void)
{
    if (!line_ready) return false;

    char s[256];
    strncpy(s, (char*)line_buf, sizeof(s)-1);
    s[sizeof(s)-1] = 0;
    line_ready = false;  // clear after copying to avoid races

    if (!nmea_checksum_ok(s)) {
        return false;
    }

    NRF_LOG_DEBUG("%s", nrf_log_push(s));


    if (is_type(s, "RMC")) {
        parse_rmc_line(s);
    } else if (is_type(s, "GGA")) {
        parse_gga_line(s);
    } else {
        return false; // ignore others
    }

    if (fix_complete()) {
        my_gps_data.isValid       = true;
        my_gps_data.lat           = (long double)gw.lat_deg;
        my_gps_data.lon           = (long double)gw.lon_deg;
        my_gps_data.lathem        = (gw.lat_deg >= 0) ? 'N' : 'S';
        my_gps_data.lonhem        = (gw.lon_deg >= 0) ? 'E' : 'W';
        my_gps_data.altitude      = gw.alt_m;
        my_gps_data.numSatellites = (uint8_t)gw.sats;
        my_gps_data.fixQuality    = (uint8_t)gw.fix_quality;
        my_gps_data.valid_data    = (uint8_t)(gw.fix_quality > 0);
        my_gps_data.timeHours     = gw.hh;
        my_gps_data.timeMinutes   = gw.mm;
        my_gps_data.timeSeconds   = gw.ss;
        my_gps_data.horizontalDilution = gw.hdop;
        // my_gps_data.heightOfGeoid = gw.geoid_m; // if you added it

        // DMS + hemisphere fields
        uint8_t d,m,ssec; char hem;
        gps_decimal_to_dms(my_gps_data.lat, &d,&m,&ssec, &hem, 'N','S');
        my_gps_data.lat_degrees = d;
        my_gps_data.lat_minutes = m;
        my_gps_data.lat_seconds = ssec;
        my_gps_data.lathem      = (uint8_t)hem;

        gps_decimal_to_dms(my_gps_data.lon, &d,&m,&ssec, &hem, 'E','W');
        my_gps_data.long_degrees = d;
        my_gps_data.long_minutes = m;
        my_gps_data.long_seconds = ssec;
        my_gps_data.lonhem       = (uint8_t)hem;

        // If you extended gps_packet_t
        // my_gps_data.speed_knots = gw.speed_kn;
        // my_gps_data.course_deg  = gw.cog_deg;

        fix_ready = true;
        return true;
    }

    return false;
}





// Optional helper: copy out last good fix
bool gps_get_fix(gps_packet_t *out)
{
    if (!fix_ready || !out) return false;
    *out = my_gps_data;
    fix_ready = false; // consume
    return true;
}










// ---------- Helper to print a fix with only integer formatting ----------
void log_fix(const gps_packet_t *fix)
{
    int32_t lat_u = (int32_t)(fix->lat * 1000000.0L);
    int32_t lon_u = (int32_t)(fix->lon * 1000000.0L);
    int32_t alt_cm = (int32_t)(fix->altitude * 100.0f);
    uint32_t hdop_centi = (uint32_t)(fix->horizontalDilution * 100.0f);

    char line[160];

    // Line 1
    snprintf(line, sizeof(line),
             "GPS FIX %02u:%02u:%02uZ lat=%ld.%06lu%c lon=%ld.%06lu%c alt=%ld.%02lu m",
             (unsigned)fix->timeHours, (unsigned)fix->timeMinutes, (unsigned)fix->timeSeconds,
             (long)(lat_u/1000000), (unsigned long)((lat_u<0)? -((long)lat_u%1000000) : (lat_u%1000000)), (char)fix->lathem,
             (long)(lon_u/1000000), (unsigned long)((lon_u<0)? -((long)lon_u%1000000) : (lon_u%1000000)), (char)fix->lonhem,
             (long)(alt_cm/100),    (unsigned long)((alt_cm<0)? -((long)alt_cm%100)    : (alt_cm%100)));
    NRF_LOG_INFO("%s", nrf_log_push(line));   // (three times, one per line)

    // Line 2
    snprintf(line, sizeof(line),
             "sats=%u q=%u hdop=%lu.%02lu",
             (unsigned)fix->numSatellites,
             (unsigned)fix->fixQuality,
             (unsigned long)(hdop_centi/100), (unsigned long)(hdop_centi%100));
   NRF_LOG_INFO("%s", nrf_log_push(line));   // (three times, one per line)

    // Line 3 (DMS)
      // Line 3 (DMS, ASCII only — "deg" instead of '°')
      snprintf(line, sizeof(line),
               "DMS lat=%u deg %u' %u\"%c  lon=%u deg %u' %u\"%c",
               (unsigned)fix->lat_degrees,  (unsigned)fix->lat_minutes,  (unsigned)fix->lat_seconds,  (char)fix->lathem,
               (unsigned)fix->long_degrees, (unsigned)fix->long_minutes, (unsigned)fix->long_seconds, (char)fix->lonhem);
      NRF_LOG_INFO("%s", nrf_log_push(line));

}


/* GPS GPIO init */
static void gps_gpio_init(void)
{
    nrf_gpio_cfg_output(PIN_GPS_PWR);
    nrf_gpio_cfg_output(PIN_GPS_RST);
    nrf_gpio_cfg_output(PIN_GPS_VBKUP);

    nrf_gpio_pin_set(PIN_GPS_PWR);
    nrf_gpio_pin_set(PIN_GPS_VBKUP);
    nrf_gpio_pin_clear(PIN_GPS_RST); // deassert reset

    
}

/* GPS UARTE init */
static void gps_uarte_init(void)
{
    // Configure pins manually
    nrf_gpio_cfg_output(GPS_UART_TX);
    nrf_gpio_cfg_input(GPS_UART_RX, NRF_GPIO_PIN_NOPULL);

    nrfx_uarte_config_t cfg = NRFX_UARTE_DEFAULT_CONFIG;
    cfg.pseltxd = GPS_UART_TX;
    cfg.pselrxd = GPS_UART_RX;
    cfg.hwfc   = NRF_UARTE_HWFC_DISABLED;
    cfg.parity = NRF_UARTE_PARITY_EXCLUDED;
    cfg.baudrate = NRF_UARTE_BAUDRATE_115200;
    cfg.interrupt_priority = 6;

    ret_code_t err = nrfx_uarte_init(&GPS_UART, &cfg, gps_uarte_event_handler);
    APP_ERROR_CHECK(err);

    err = nrfx_uarte_rx(&GPS_UART, gps_rx_buf, sizeof(gps_rx_buf)-1);
    APP_ERROR_CHECK(err);
}

/* Main GPS init */
void gps_init(void)
{
    gps_gpio_init();
    gps_uarte_init();
   // NRF_LOG_INFO("GPS Initialized");
}












///////////////////////////////////////////////////////////////////////////////

/*
 * GPS.h
 *
 *  Created on: Aug 17, 2025
 *      Author: 
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

//#include "configs.h"
//#include "ap_data.h"
//#include "api_ab_uart.h"
//#include "firmware_defaults.h"
#include "stdarg.h"
//#include "dr_gps_lc76f.h"
//#include "api_ab_rtc.h"
//#include "log.h"
//#include "api_ab_soft_timer.h"
//#include "ap_data.h"
//#include "unit_testing.h"
//#include "STTLE103.h"
//#include "bb_assert.h"

#define GPS_READ_TIME_OUT   5000U
#define UART_GPS_RCV_SIZE   2048U
#define UART_GPS_TX_SIZE    2048U
#define GPS_READ_SIZE       2048U
#define FIXID_DATA_SIZE     82
#define GPS_TRANSMIT_SIZE   64U

#define GPS_LPMC            "$PGKC051,0*37\r\n"





typedef enum
{
    gps_start=0,
    gps_read_data,
    gps_motion_distance_cal,
    gps_geo_fence_distance_cal,
    gps_low_pwr_mode,
    gps_stop
} gps_stages_uart;

typedef enum
{
    gps_lpmc_send=0,
    gps_lpmc_send_wait_responce,

}gps_lpmc_t;

typedef enum
{
    gps_write_tx=0,
    gps_write_tx_complete,

}gps_write_t;

typedef enum
{
    gps_read_rx=0,
    gps_read_rx_complete

}gps_read_t;

/** @brief This struct (gps_generic_info) is a generic internal structure used by
 * API to store relevant information about GPS driver.
 */
typedef struct
{
    bool gps_init;
    bool gps_reset_status;

  /*  
    io_pin_number force_on;
    io_pin_number fix_indecator;

    io_pin_number gps_power;
    io_pin_number gps_backup_power;
    io_pin_number gps_reset;

  */

    uint8_t fix_state;
    uint8_t error_count;
    uint16_t old_rx_size;
    uint8_t gps_uart_buffer[UART_GPS_RCV_SIZE];
    uint8_t gps_uart_txbuffer[UART_GPS_TX_SIZE];
    gps_stages_uart gps_stage;
    gps_write_t gps_write;
    gps_lpmc_t   gps_lpmc;
    gps_read_t   gps_lc76f_read;

} gps_generic_info;

/*
static gps_generic_info gps_info =  {
                                        .gps_init = false,
                                        .gps_reset_status = false,
                                       // .force_on = GPS_WAKEUP_PIN,
                                       // .fix_indecator = GPS_IPPS_PIN,
                                       // .gps_power = GPS_PWR_PIN,
                                        .gps_lpmc = gps_lpmc_send,
                                        .gps_write = gps_write_tx,
                                        .gps_lc76f_read = gps_read_rx,
                                        .old_rx_size=0U,
                                        .gps_backup_power = GPS_BCKP_PIN,
                                       // .gps_reset = GPS_RST_PIN,
                                        .fix_state = 0U,
                                        .gps_stage = gps_start,
                                        .error_count = 0U,
                                    };


uart_t gps_uart =
{
    .rx_pin_no     = GPS_RX_PIN,
    .tx_pin_no     = GPS_TX_PIN,
    .rts_pin_no    = NULL,
    .cts_pin_no    = NULL,
    .flow_control  = false,
    .baud_rate     = GPS_BAUD_RATE,
    .parity        = u_parity_odd,
    .stopbits      = u_stopbits_one,
    .databits      = u_databits_eight,
    .hw_uart       = GPS_UART_HANDLE
};




status_t gps_initialize()
{
    status_t ustatus = fail;
    status_t status_stimer = fail;
    status_t return_status = fail;

    io_configuration enable_pin_configuration;

    ustatus = uart_initialize( &gps_uart );

    if(( ustatus == success ) && (gps_info.gps_reset_status == false))
    {
        status_stimer = timer_initiliaze( &gps_timer , NULL);
        if(status_stimer == success)
        {
            return_status = success;
        }
        if(return_status == success)
        {
            io_get_default_Config( &enable_pin_configuration );

            enable_pin_configuration.pin_direction      = io_pin_output;

            return_status = io_initialize( &gps_info.gps_reset , &enable_pin_configuration , NULL);

            if( return_status == success )
            {
#if(STTNE201 == _ENABLE)
                return_status = io_set_pin_high( &gps_info.gps_reset );
#else
                return_status = io_set_pin_low( &gps_info.gps_reset );
#endif
                if(return_status == success)
                {
                    gps_info.gps_reset_status = true;
                }
            }
        }
    }
    if( ustatus == success )
    {
        gps_info.gps_init = true;
        return_status = success;
    }

    return return_status;
}

*/
static uint8_t gps_get_packet_size(uint8_t* buffer)
{
    uint8_t i;
    for(i = 0 ;  i < 255U; i++ )
    {
        if(*(buffer+i) == '\n' )
        {
            break;
        }
    }

    return (i+1);
}




typedef enum {
    GPS_TX_PENDING,
    GPS_TX_SUCCESS,
    GPS_TX_FAIL
} gps_tx_status_t;

/* Timeout timer for GPS TX */
APP_TIMER_DEF(gps_tx_timer);
static bool gps_tx_done = true;  // track TX completion

/* Callback for GPS TX timer */
static void gps_tx_timeout_handler(void * p_context)
{
    gps_tx_done = true; // force completion on timeout
}

/* GPS UART write function (blocking style with timeout via app_timer) */
gps_tx_status_t gps_uart_write(uint8_t* data, size_t length, uint32_t timeout_ms)
{
    ret_code_t err;

    if (!data || length == 0)
        return GPS_TX_FAIL;

    if (!gps_tx_done) // previous TX still in progress
        return GPS_TX_PENDING;

    gps_tx_done = false;

    // Start timeout timer
    err = app_timer_start(gps_tx_timer, APP_TIMER_TICKS(timeout_ms), NULL);
    APP_ERROR_CHECK(err);

    // Blocking transmit
    err = nrfx_uarte_tx(&GPS_UART, data, length);
    if (err != NRFX_SUCCESS)
    {
        gps_tx_done = true;
        app_timer_stop(gps_tx_timer);
        return GPS_TX_FAIL;
    }

    gps_tx_done = true;
    app_timer_stop(gps_tx_timer);

    return GPS_TX_SUCCESS;
}




///////////////














/* Global GPS info */
static gps_generic_info gps_info = {0};



/* Low-power mode function */
status_t gps_low_power_mode(void)
{
    status_t return_status = pending;

    switch (gps_info.gps_lpmc)
    {
        case gps_lpmc_send:
        {
            // Send LPMC command
            return_status = gps_uart_write(GPS_LPMC, sizeof(GPS_LPMC) - 1, 100); // 100 ms timeout

            if (return_status == success)
            {
                return_status = pending;
                gps_info.gps_lpmc = gps_lpmc_send_wait_responce;
            }
        } break;

        case gps_lpmc_send_wait_responce:
        {
            // Read response (stub for now)
            return_status = gps_uart_read(gps_info.gps_uart_buffer,
                                          sizeof(gps_info.gps_uart_buffer),
                                          100);
        } break;
    }

    // Reset state after success or fail
    if ((return_status == fail) || (return_status == success))
    {
        gps_info.gps_lpmc = gps_lpmc_send;
    }

    return return_status;
}

/////////////////////




////3
/* Deinitialize GPS */
status_t gps_deinitialize(void)
{
    status_t return_status = fail;
    ret_code_t err;

    if (!gps_info.gps_init)
        return fail;

    // Stop any ongoing reception
    nrfx_uarte_rx_abort(&GPS_UART);
    nrfx_uarte_tx_abort(&GPS_UART);

    // Deinitialize UARTE
    nrfx_uarte_uninit(&GPS_UART);

    // Disable GPS power
    nrf_gpio_pin_clear(PIN_GPS_PWR);
    nrf_gpio_pin_clear(PIN_GPS_VBKUP);

    // Clear GPS info
    gps_info.gps_init = false;
    gps_info.gps_reset_status = false;
    gps_info.gps_lpmc = gps_lpmc_send;
    gps_info.gps_write = gps_write_tx;
    gps_info.gps_stage = gps_start;
    gps_info.fix_state = 0;
    gps_info.error_count = 0;
    memset(gps_info.gps_uart_buffer, 0, sizeof(gps_info.gps_uart_buffer));
    memset(gps_info.gps_uart_txbuffer, 0, sizeof(gps_info.gps_uart_txbuffer));

    return_status = success;

    return return_status;
}


////3

////4

status_t gps_uart_read(uint8_t* data, uint16_t timeout_ms, uint16_t* size)
{
    if (!data || !size)
        return fail;

    status_t return_status = pending;
    ret_code_t err;
    uint16_t rx_len = 0;

    switch (gps_info.gps_lc76f_read)
    {
        case gps_read_rx:
        {
            // Start reception
            memset(gps_info.gps_uart_buffer, 0, sizeof(gps_info.gps_uart_buffer));
            rx_len = sizeof(gps_info.gps_uart_buffer) - 1;

            err = nrfx_uarte_rx(&GPS_UART, gps_info.gps_uart_buffer, rx_len);
            APP_ERROR_CHECK(err);

            // Set stage to wait for RX complete
            gps_info.gps_lc76f_read = gps_read_rx_complete;
        }
        break;

        case gps_read_rx_complete:
        {
            // Check if any data has arrived
            // Note: In nrfx_uarte, we get data in the event handler, so here we just copy latest buffer
            *size = strlen((char*)gps_info.gps_uart_buffer);
            if (*size > 0)
            {
                memcpy(data, gps_info.gps_uart_buffer, *size);
                // Clear internal buffer for next read
                memset(gps_info.gps_uart_buffer, 0, sizeof(gps_info.gps_uart_buffer));
                return_status = success;
            }
            else
            {
                return_status = fail;
            }

            gps_info.gps_lc76f_read = gps_read_rx;
        }
        break;
    }

    return return_status;
}

////4


///5

void gps_clear(void)
{
    // Clear internal UART buffer
    memset(gps_info.gps_uart_buffer, 0, sizeof(gps_info.gps_uart_buffer));
}

////5



////6
status_t gps_extract_data_string(uint8_t* gps_data, uint8_t* gngga)
{
    if (!gps_data || !gngga)
        return fail;

    status_t status = fail;

    // Search for GNGGA sentence
    uint8_t* gngga_start = (uint8_t*)strstr((char*)gps_data, "$GNGGA");
    if (gngga_start != NULL)
    {
        uint8_t* gngga_end = (uint8_t*)strstr((char*)gngga_start, "\r\n");
        if (gngga_end != NULL)
        {
            uint16_t gngga_length = gngga_end - gngga_start;
            if (gngga_length < GNGGA_LENGTH) // Ensure buffer is sufficient
            {
                strncpy((char*)gngga, (char*)gngga_start, gngga_length);
                gngga[gngga_length] = '\0';
                // Optional: set GPS fix flag if you have a global struct
                // ap_data.rtv_gps_fixed_flag = true;
                status = success;
            }
        }
    }

    return status;
}

////5


////6


ap_data_t ap_data = {0};
status_t gps_data_read(gps_packet_t *gps_ptr)
{
    if (!gps_ptr) return fail;

    uint16_t len = 0U;
    uint8_t gps_buf[GNGGA_LENGTH];  // buffer for extracted GNGGA string
    status_t status = pending;

    switch (gps_info.gps_stage)
    {
        case gps_start:
        {
            if (ap_data.rtv_gps_power_status == false)
            {
                status = gps_config_power_vcc_enable();
            }
            else
            {
                status = success;
            }

            if (status == success)
            {
                status = gps_initialize();
                if (status == success)
                {
                    status = pending;
                    gps_info.gps_stage = gps_read_data;
                    memset(gps_info.gps_uart_buffer, 0, UART_GPS_RCV_SIZE);
                    gps_info.fix_state = 0U;
                    gps_clear();
                }
            }
            else
            {
                status = fail;
                gps_info.gps_stage = gps_start;
            }
        } break;

        case gps_read_data:
        {
            status = gps_uart_read(gps_info.gps_uart_buffer, GPS_READ_TIME_OUT, &len);
            if (status == success)
            {
                status = gps_extract_data_string(gps_info.gps_uart_buffer, gps_buf);
                if (status == success)
                {
                    gps_info.error_count = 0;
                    status = gps_parse(gps_buf, gps_ptr);  // parse into gps_packet_t
                }

                len = 0U;
                memset(gps_info.gps_uart_buffer, 0, UART_GPS_RCV_SIZE);
                gps_clear();
            }

            if (status == fail)
            {
                gps_info.error_count++;
                memset(gps_info.gps_uart_buffer, 0, UART_GPS_RCV_SIZE);
                if (ap_data.rtv_gps_fixed_flag == false)
                {
                    ap_data.rtv_gps_fixed_flag = true;
                    status = success;
                }
                else
                {
                    ap_data.rtv_gps_fixed_flag = false;
                    ap_data.rtv_gps_fixed_activity_flag = fail;
                }
            }

            if (status == success)
            {
                status = pending;
                //if (ap_data.rtv_type_bin_pickup == MOTION_ENABLE_FEATURE) //commented now
                   // gps_info.gps_stage = gps_motion_distance_cal;
                //else if (ap_data.rtv_fence_enable == GEOFENCE_ENABLE_FEATURE)
                    gps_info.gps_stage = gps_geo_fence_distance_cal;
                //else
                    gps_info.gps_stage = gps_low_pwr_mode;
            }
        } break;

        case gps_motion_distance_cal:
        {
           // status = calculate_motion_distance(); //commented now
           // if (status == fail) BB_ASSERT(false);  //commented now
            status = pending;
            gps_info.gps_stage = gps_geo_fence_distance_cal;
        } break;

        case gps_geo_fence_distance_cal:
        {
           // status = calculate_distance_geo_fence_center();//commented now
          //  if (status == fail) BB_ASSERT(false);
            status = pending;
            gps_info.gps_stage = gps_low_pwr_mode;
        } break;

        case gps_low_pwr_mode:
        {
            status = gps_low_power_mode();
            if (status == success) gps_clear();
        } break;

        default:
        {
            status = fail;
        } break;
    }

    if ((status == success) || (status == fail))
    {
        status = gps_deinitialize();

        if (ap_data.rtv_gps_fixed_flag == true && ap_data.rtv_gps_power_status == true)
        {
            status = gps_config_power_vcc_disable();
            ap_data.rtv_gps_fixed_activity_flag = success;
        }

        gps_info.gps_stage = gps_start;
    }

    return status;
}


////6

status_t gps_read()
{
    status_t status = pending;

    gps_packet_t data_pack;

    status = gps_data_read(&data_pack);

    return status;
}


////7

status_t gps_config_power_v_bckp_enable(void)
{
    status_t status = success;

    // Configure backup power pin as output
    nrf_gpio_cfg_output(PIN_GPS_VBKUP);

    // Enable GPS backup power (set high)
    nrf_gpio_pin_set(PIN_GPS_VBKUP);

    return status;
}

status_t gps_config_power_v_bckp_disable(void)
{
    status_t status = success;

    // Disable GPS backup power (set low)
    nrf_gpio_pin_clear(PIN_GPS_VBKUP);

    return status;
}


////7




status_t gps_config_power_vcc_enable(void)
{
    status_t status = success;

    nrf_gpio_cfg_output(PIN_GPS_PWR);
    nrf_gpio_pin_set(PIN_GPS_PWR);

    return status;
}

status_t gps_config_power_vcc_disable(void)
{
    status_t status = success;

    nrf_gpio_pin_clear(PIN_GPS_PWR);

    return status;
}






status_t gps_config_device()
{
    uint8_t gps_buf[GPS_READ_SIZE];
    uint16_t len = 0U;
    status_t status_s = pending;
    status_t status = pending;

    switch (gps_info.gps_stage )
    {
        case gps_start:
        {
            status_s = gps_initialize();
            if( status_s == success )
            {
                status_s = pending;

                gps_info.gps_stage = gps_read_data;

                memset( gps_info.gps_uart_buffer , 0 , UART_GPS_RCV_SIZE );

                gps_clear();
                nrf_delay_ms(1000);
            }

        }break;

        case gps_read_data:
        {
            status_s = gps_uart_read( gps_info.gps_uart_buffer ,GPS_READ_TIME_OUT , &len );
            if( status_s == success )
            {
                gps_clear();
            }

        }break;
    }

    if( status_s == fail || status_s==success)
    {
        if(status_s == success)
        {
            gps_info.gps_reset_status = false;
            ap_data.rtv_gps_fixed_flag = true;
            ap_data.rtv_gps_fixed_activity_flag  = fail;
           
        }

        status = gps_deinitialize();
        if( status == success )
        {
          gps_info.gps_stage = gps_start;
        }
    }

    return status_s;
}

#define UTC_TIME_OFFSET    5          /*  PAKISTAN is GMT+5 , this offset is Time zone dependent*/

void gps_extract_time(uint8_t* time_token, uint16_t* hours, uint16_t* minutes, uint16_t* seconds)
{
    if (time_token != NULL)
    {
        sscanf(time_token, "%2d%2d%2d", hours, minutes, seconds);
    }
}






status_t gps_parse(uint8_t * input, gps_packet_t * packet)
{
    status_t status = success;
    char *p = strchr((char*)input, ',');

    if (!p || !packet)
        return fail;

    // Extract time
    gps_extract_time((uint8_t*)(p+1), &packet->timeHours, &packet->timeMinutes, &packet->timeSeconds);

    // Latitude
    p = strchr(p+1, ',');
    packet->lat = atof(p+1);

    p = strchr(p+1, ',');
    packet->lathem = (p[1] != ',') ? p[1] : '?';

    // Longitude
    p = strchr(p+1, ',');
    packet->lon = atof(p+1);

    p = strchr(p+1, ',');
    packet->lonhem = (p[1] != ',') ? p[1] : '?';

    // Valid data / fix quality
    p = strchr(p+1, ',');
    packet->valid_data = atoi(p+1);
    packet->isValid = (packet->valid_data != 0);
    packet->fixQuality = packet->valid_data;

    if (packet->isValid)
    {
        // Satellites
        p = strchr(p+1, ',');
        packet->numSatellites = atoi(p+1);

        // Horizontal dilution
        p = strchr(p+1, ',');
        packet->horizontalDilution = atof(p+1);

        // Altitude
        p = strchr(p+1, ',');
        packet->altitude = atof(p+1);

        // Altitude unit (optional)
        p = strchr(p+1, ',');
        packet->unit = (uint8_t)p[1]; 

        // Height of Geoid
        p = strchr(p+1, ',');
        packet->heightOfGeoid = atof(p+1);

        // Checksum
        p = strstr((char*)input, "*");
        packet->checksum = (uint8_t*)p;

        // Convert lat/lon to degrees, minutes, seconds
        uint8_t deg, min, sec;
        gps_convert_decimal_deg_dms(&deg, &min, &sec, packet->lat);
        packet->lat_degrees = deg;
        packet->lat_minutes = min;
        packet->lat_seconds = sec;

        gps_convert_decimal_deg_dms(&deg, &min, &sec, packet->lon);
        packet->long_degrees = deg;
        packet->long_minutes = min;
        packet->long_seconds = sec;

     /*   NRF_LOG_INFO("GPS Valid: Lat %d°%d'%d'' %c, Lon %d°%d'%d'' %c, Satellites: %d",
                     packet->lat_degrees, packet->lat_minutes, packet->lat_seconds, packet->lathem,
                     packet->long_degrees, packet->long_minutes, packet->long_seconds, packet->lonhem,
                     packet->numSatellites);
                     */
    }
    else
    {
        NRF_LOG_ERROR("GPS Data invalid");
    }

    return status;
}


















void gps_clear_gps_packet(gps_packet_t * loc)
{
    loc->isValid           =0;
    loc->lathem            ='\0';
    loc->lat               =0;
    loc->lonhem            ='\0';
    loc->lon               =0;
    loc->fixQuality     ='\0';
    loc->numSatellites  ='\0';
    loc->checksum       ='\0';
    loc->time              =0;
    loc->timeHours         =0;
    loc->timeMinutes       =0;
    loc->timeSeconds       =0;
    loc->heightOfGeoid     =0;
    loc->altitude          =0;
}

long double gps_convert_to_decimal_degrees(uint16_t degree, float minSec)
{
    long double result = 0.00000;
    long double temp = 0.00000;
    temp = minSec/60;
    result = degree + temp;
    return result;
}

float gps_knots_to_mph(float knots)
{
    return knots * 1.15077945;
}




uint32_t gps_unix_timestamp(uint16_t year,uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, uint16_t second)
{
    struct tm t;
    time_t t_of_day;

    t.tm_year = (2000+year)-1900;           /* Year - 1900 */
    t.tm_mon = month-1;                     /* Month, where 0 = jan */
    t.tm_mday = day;                        /* Day of the month */
    hour = hour + UTC_TIME_OFFSET;
    if(hour > 23)
    {
        hour -= 24;
    }
    t.tm_hour= hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = -1;                        /* Is DST on? 1 = yes, 0 = no, -1 = unknown */
    t_of_day = mktime(&t);

    return t_of_day;
}


long double gps_convert_degminsec_decimal_deg(uint8_t degrees, uint8_t minutes, uint8_t seconds)
{
    long double result;

    long double tmp = ((long double) (minutes/60.0))+((float)(seconds/3600.0));

    result = ((long double)degrees) + tmp;

    return result;
}

void gps_convert_decimal_deg_dms(uint8_t* degrees, uint8_t* minutes, uint8_t* seconds, float dd)
{
    long double val = 0;

    *degrees = (uint8_t)(dd);

    val = dd - (long double) *degrees;

    *minutes = (uint8_t) (val * 100);

    val = (val*100) - *minutes;

    *seconds = val * 60;
}






status_t gps_check_communication()
{
    uint8_t gps_buf[GPS_READ_SIZE];
    uint16_t len = 0U;
    status_t status = pending;
    status_t status_s = pending;

    switch (gps_info.gps_stage )
    {
        case gps_start:
        {
            status_s = gps_initialize();
            if( status_s == success )
            {
                status_s = pending;

                gps_info.gps_stage = gps_read_data;

                memset( gps_info.gps_uart_buffer , 0 , UART_GPS_RCV_SIZE );

                gps_clear();
            }

        }break;

        case gps_read_data:
        {
            status_s = gps_uart_read( gps_info.gps_uart_buffer ,GPS_READ_TIME_OUT , &len );
            if( status_s == success )
            {
                gps_clear();
            }

        }break;
    }

    if( status_s == fail || status_s==success)
    {
        if(status_s == success)
        {
            gps_info.gps_reset_status = false;
        }

        status = gps_deinitialize();
        if( status == success )
        {
          status = gps_config_power_vcc_disable();
          if( status == success )
          {
              gps_info.gps_stage = gps_start;
          }
          else if( status == fail )
          {
              status_s = fail;
          }
        }
    }

    return status_s;
}















status_t gps_initialize(void)
{
    nrfx_err_t err_code;

    // Configure GPS reset pin
    nrf_gpio_cfg_output(PIN_GPS_RST);
    nrf_gpio_pin_set(PIN_GPS_RST);    // Hold low to reset
    nrf_delay_ms(100);
    nrf_gpio_pin_clear(PIN_GPS_RST);      // Release reset

    // Configure UARTE
    nrfx_uarte_config_t uarte_cfg = NRFX_UARTE_DEFAULT_CONFIG;
    uarte_cfg.pseltxd = GPS_UART_TX;
    uarte_cfg.pselrxd = GPS_UART_RX;
    uarte_cfg.baudrate = NRF_UARTE_BAUDRATE_9600;
    uarte_cfg.parity = NRF_UARTE_PARITY_EXCLUDED;
    uarte_cfg.hwfc = NRF_UARTE_HWFC_DISABLED;

    err_code = nrfx_uarte_init(&GPS_UART, &uarte_cfg, NULL);
    if (err_code != NRFX_SUCCESS)
        return fail;

    // Power and backup pins
    nrf_gpio_cfg_output(PIN_GPS_PWR);
    nrf_gpio_pin_set(PIN_GPS_PWR);      // Turn on GPS

    nrf_gpio_cfg_output(PIN_GPS_VBKUP);
    nrf_gpio_pin_set(PIN_GPS_VBKUP);    // Enable backup power

    gps_info.gps_init = true;
    return success;
}






///////////////////////////////////////////////////////////////////////////////

























