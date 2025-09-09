#include "adc_battery.h"
#include "nrfx_saadc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"



#ifndef BAT_CAL
#define BAT_CAL 1.26f   // tune with a DMM: BAT_CAL = Vbat_true / Vbat_reported
#endif

// ---- tunables (easy to tweak) ----
#define BAT_SETTLE_MS         30    // time after enabling BAT_EN
#define BAT_WARMUP_SAMPLES     8
#define BAT_AVG_SAMPLES        16
#define BAT_HYSTERESIS_PCT     2     // % change required before updating

static uint16_t s_last_pct = 255;    // 255 = invalid -> always update first time






static bool sample_raw(int16_t* out);
static uint16_t read_once_mv(void);



static bool s_saadc_inited = false;

// add near top
static volatile bool s_cal_done = false;

static void saadc_evt_handler(nrfx_saadc_evt_t const * p_event) {
    if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
        s_cal_done = true;
    }
}

bool battery_adc_init(void)
{
    if (!s_saadc_inited) {
        nrfx_saadc_config_t cfg = NRFX_SAADC_DEFAULT_CONFIG;
        cfg.resolution = NRF_SAADC_RESOLUTION_12BIT;          // make it explicit
        cfg.oversample = NRF_SAADC_OVERSAMPLE_8X;    // does 8 sub-samples per trigger//NRF_SAADC_OVERSAMPLE_DISABLED;       // we’ll average in SW
        APP_ERROR_CHECK(nrfx_saadc_init(&cfg, saadc_evt_handler));

        nrf_saadc_channel_config_t ch =
            NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BATTERY_ADC_INPUT);
        ch.gain      = NRF_SAADC_GAIN1_6;                     // FS at pin = 3.6 V
        ch.reference = NRF_SAADC_REFERENCE_INTERNAL;          // 0.6 V
        ch.acq_time  = NRF_SAADC_ACQTIME_40US;                // <- was 10us
        ch.burst     = NRF_SAADC_BURST_ENABLED;  //NRF_SAADC_BURST_DISABLED;
        APP_ERROR_CHECK(nrfx_saadc_channel_init(0, &ch));

        // One-time offset calibration
        s_cal_done = false;
        APP_ERROR_CHECK(nrfx_saadc_calibrate_offset());
        while (!s_cal_done) { /* spin */ }

        // BAT-EN control
        nrf_gpio_cfg_output(PIN_BAT_EN);
        nrf_gpio_pin_clear(PIN_BAT_EN);
        s_saadc_inited = true;
    }
    return true;
}

bool battery_read_mv(uint16_t* vbat_mv)
{
    if (!s_saadc_inited) battery_adc_init();

    // Turn divider on and let the node settle
    nrf_gpio_pin_set(PIN_BAT_EN);
    nrf_delay_ms(BAT_SETTLE_MS);

    // Throw away a few to charge the sampling cap
    for (int i = 0; i < BAT_WARMUP_SAMPLES; i++) { int16_t d; sample_raw(&d); }

// --- spread-and-median parameters ---
#define VBAT_GROUPS       7      // number of groups
#define VBAT_GROUP_SIZE   8      // samples per group
#define VBAT_SPAN_MS      60     // cover activity over ~60 ms

float group_means[VBAT_GROUPS];
const uint32_t gap_ms = (VBAT_GROUPS > 1) ? (VBAT_SPAN_MS / (VBAT_GROUPS - 1)) : 0;

for (int g = 0; g < VBAT_GROUPS; g++) {
    int32_t acc = 0;
    for (int i = 0; i < VBAT_GROUP_SIZE; i++) {
        int16_t s; sample_raw(&s);
        acc += (int32_t)s;
        nrf_delay_us(200);
    }
    group_means[g] = acc / (float)VBAT_GROUP_SIZE;
    if (g + 1 < VBAT_GROUPS) nrf_delay_ms(gap_ms);
}

// median-of-groups (tiny insertion sort)
for (int i = 1; i < VBAT_GROUPS; i++) {
    float key = group_means[i];
    int j = i - 1;
    while (j >= 0 && group_means[j] > key) { group_means[j+1] = group_means[j]; j--; }
    group_means[j+1] = key;
}
const float avg_counts = group_means[VBAT_GROUPS/2];

    // Convert counts -> Vpin -> VBAT
    const float vref = 0.6f;
    const float gain = (1.0f/6.0f);
    const float adc_fs_volts  = vref / gain;     // 3.6 V
    const float adc_max_count = 4095.0f;         // 12-bit
    const float v_pin = (avg_counts / adc_max_count) * adc_fs_volts;

    const float divider = (BAT_R_TOP_OHMS + BAT_R_BOTTOM_OHMS) / (float)BAT_R_BOTTOM_OHMS;

    #ifndef BAT_CAL
    #define BAT_CAL 1.00f    // leave =1.00 if your resistor values are correct BAT_CAL = Vbat_true / Vbat_reported
    #endif

    float vbat = v_pin * divider * BAT_CAL;

    nrf_gpio_pin_clear(PIN_BAT_EN); // divider off (save leakage)

    uint32_t mv = (uint32_t)(vbat * 1000.0f + 0.5f);
    if (vbat_mv) *vbat_mv = (mv > 65535u) ? 65535u : (uint16_t)mv;
    return true;
}






uint8_t battery_percent_stable(uint16_t mv)
{
    uint8_t pct = battery_percent_from_mv(mv);
    if (s_last_pct == 255) { s_last_pct = pct; return pct; }

    // apply hysteresis
    uint8_t lo = (s_last_pct > BAT_HYSTERESIS_PCT) ? (s_last_pct - BAT_HYSTERESIS_PCT) : 0;
    uint8_t hi = (s_last_pct + BAT_HYSTERESIS_PCT > 100) ? 100 : (s_last_pct + BAT_HYSTERESIS_PCT);

    if (pct < lo || pct > hi) s_last_pct = pct;
    return s_last_pct;
}





static uint16_t read_once_mv(void) {
    int16_t s;
    APP_ERROR_CHECK(nrfx_saadc_sample_convert(0, &s));
    if (s < 0) s = 0;

    const float vref = 0.6f, gain = (1.0f/6.0f);
    const float fs   = vref / gain;                 // 3.6 V at pin
    const float vpin = (s / 4095.0f) * fs;

    //const float scale = (BAT_R_TOP_OHMS + BAT_R_BOTTOM_OHMS) / (float)BAT_R_BOTTOM_OHMS;
    const float scale = ((BAT_R_TOP_OHMS + BAT_R_BOTTOM_OHMS) / (float)BAT_R_BOTTOM_OHMS) * BAT_CAL;

    uint32_t mv = (uint32_t)((vpin * scale) * 1000.0f + 0.5f);
    return (mv > 65535u) ? 65535u : (uint16_t)mv;
}


static bool sample_raw(int16_t* out) {
    nrf_saadc_value_t val;
    APP_ERROR_CHECK(nrfx_saadc_sample_convert(0, &val));
    if (val < 0) val = 0;
    *out = (int16_t)val;
    return true;
}


static void battery_debug_dump(void) {
    uint16_t mv;
    if (battery_read_mv(&mv)) {
        // print or log 'mv' with your logger
        // e.g., NRF_LOG_INFO("VBAT = %u mV", mv);
    }
}

// ---- Simple percentage mapper ----
static uint16_t s_empty_mv      = VBAT_EMPTY_MV_DEFAULT;
static uint16_t s_full_start_mv = VBAT_FULL_START_MV_DEFAULT;
static uint16_t s_full_max_mv   = VBAT_FULL_MAX_MV_DEFAULT;

void battery_gauge_set_thresholds(uint16_t empty_mv,
                                  uint16_t full_start_mv,
                                  uint16_t full_max_mv)
{
    s_empty_mv      = empty_mv;
    s_full_start_mv = full_start_mv;
    s_full_max_mv   = full_max_mv;
}

uint8_t battery_percent_from_mv(uint16_t mv)
{
    // clamp to configured max
    if (mv >= s_full_start_mv) return 100;
    if (mv <= s_empty_mv)      return 0;

    uint32_t span = (uint32_t)s_full_start_mv - s_empty_mv;   // e.g. 700 mV
    uint32_t num  = (uint32_t)mv - s_empty_mv;

    // linear map with rounding
    uint32_t pct = (num * 100u + span/2u) / span;
    if (pct > 100u) pct = 100u;
    return (uint8_t)pct;
}
