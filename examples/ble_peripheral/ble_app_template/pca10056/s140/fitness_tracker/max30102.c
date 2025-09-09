#include "max30102.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_delay.h"

#ifndef NRF_SUCCESS
#define NRF_SUCCESS 0
#endif

#define MASK18(x) ((x) & 0x3FFFFu)

static uint32_t i2c_write(const nrfx_twim_t *i2c, uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buf[1+32];
    if (len > 32) return 1;
    buf[0] = reg;
    if (len) memcpy(&buf[1], data, len);
    return nrfx_twim_tx(i2c, MAX30102_I2C_ADDR, buf, (uint8_t)(1+len), false);
}
static uint32_t i2c_write_u8(const nrfx_twim_t *i2c, uint8_t reg, uint8_t val) {
    return i2c_write(i2c, reg, &val, 1);
}
static uint32_t i2c_read(const nrfx_twim_t *i2c, uint8_t reg, uint8_t *data, size_t len) {
    uint32_t err = nrfx_twim_tx(i2c, MAX30102_I2C_ADDR, &reg, 1, true);
    if (err != NRF_SUCCESS) return err;
    return nrfx_twim_rx(i2c, MAX30102_I2C_ADDR, data, (uint8_t)len);
}

uint32_t max30102_init(max30102_t *dev, const nrfx_twim_t *i2c) {
    if (!dev || !i2c) return 1;
    dev->i2c = i2c;

    // Probe PART ID
    uint8_t id = 0;
    uint32_t err = i2c_read(dev->i2c, MAX30102_REG_PART_ID, &id, 1);
    if (err != NRF_SUCCESS) return err;
    if (id != MAX30102_EXPECTED_PART_ID) return 2;
    return NRF_SUCCESS;
}

uint32_t max30102_read_part_id(max30102_t *dev, uint8_t *out) {
    if (!dev || !out) return 1;
    return i2c_read(dev->i2c, MAX30102_REG_PART_ID, out, 1);
}

uint32_t max30102_soft_reset(max30102_t *dev) {
    if (!dev) return 1;
    uint32_t err = i2c_write_u8(dev->i2c, MAX30102_REG_MODE_CONFIG, 0x40);
    nrf_delay_ms(10);
    return err;
}

uint32_t max30102_config_spo2(max30102_t *dev, uint16_t sr_hz, uint8_t pw_code, uint8_t adc_range_code)
{
    if (!dev) return 1;

    uint8_t sr_bits;
    switch (sr_hz) {
        case 50:  sr_bits = 0; break;
        case 100: sr_bits = 1; break;
        case 200: sr_bits = 2; break;
        case 400: sr_bits = 3; break;
        case 800: sr_bits = 4; break;
        case 1000:sr_bits = 5; break;
        default:  sr_bits = 1; break; // 100 Hz
    }

    // Correct field placement:
    // [6:5] = adc_range_code (0..3), [4:2] = sr_bits (0..5), [1:0] = pw_code (0..3)
    uint8_t spo2 = ((adc_range_code & 0x3) << 5) | ((sr_bits & 0x7) << 2) | (pw_code & 0x3);
    return i2c_write_u8(dev->i2c, MAX30102_REG_SPO2_CONFIG, spo2);
}


uint32_t max30102_set_mode_spo2(max30102_t *dev) {
    if (!dev) return 1;
    uint8_t v=0; i2c_read(dev->i2c, MAX30102_REG_MODE_CONFIG, &v, 1);
    v = (v & ~0x07) | 0x03;
    return i2c_write_u8(dev->i2c, MAX30102_REG_MODE_CONFIG, v);
}

// LED1 = RED, LED2 = IR
uint32_t max30102_set_led_currents(max30102_t *dev, uint8_t ir, uint8_t red) {
    uint32_t err = i2c_write_u8(dev->i2c, MAX30102_REG_LED1_PA, red);
    if (err != NRF_SUCCESS) return err;
    return i2c_write_u8(dev->i2c, MAX30102_REG_LED2_PA, ir);
}


uint32_t max30102_fifo_setup(max30102_t *dev, uint8_t sample_avg, bool roll_over, uint8_t fifo_afull) {
    if (!dev) return 1;
    uint8_t ave_bits=0;
    switch (sample_avg) { case 1: ave_bits=0; break; case 2: ave_bits=1; break; case 4: ave_bits=2; break;
        case 8: ave_bits=3; break; case 16: ave_bits=4; break; case 32: ave_bits=5; break; default: ave_bits=0; break; }
    uint8_t v = (ave_bits<<5) | ((roll_over?1:0)<<4) | (fifo_afull & 0x0F);
    return i2c_write_u8(dev->i2c, MAX30102_REG_FIFO_CONFIG, v);
}

uint32_t max30102_enable_interrupts(max30102_t *dev,
                                    bool data_ready, bool alc_ovf,
                                    bool prox, bool die_temp)
{
    if (!dev) return 1;

    // MAX30102 INT_ENABLE1:
    // Bit7 A_FULL_EN, Bit6 PPG_RDY_EN, Bit5 ALC_OVF_EN, Bit4 PROX_INT_EN
    uint8_t en1 = 0;
    if (data_ready) en1 |= (1u << 6);  // PPG_RDY_EN (was 1<<5 before — wrong)
    if (alc_ovf)   en1 |= (1u << 5);   // ALC_OVF_EN
    if (prox)      en1 |= (1u << 4);   // PROX_INT_EN
    uint32_t err = i2c_write_u8(dev->i2c, MAX30102_REG_INT_ENABLE1, en1);
    if (err != NRF_SUCCESS) return err;

    // MAX30102 INT_ENABLE2:
    // Bit1 DIE_TEMP_RDY_EN
    uint8_t en2 = 0;
    if (die_temp) en2 |= (1u << 1);    // DIE_TEMP_RDY_EN
    return i2c_write_u8(dev->i2c, MAX30102_REG_INT_ENABLE2, en2);
}


uint32_t max30102_clear_interrupts(max30102_t *dev) {
    uint8_t d;
    i2c_read(dev->i2c, MAX30102_REG_INT_STATUS1, &d, 1);
    i2c_read(dev->i2c, MAX30102_REG_INT_STATUS2, &d, 1);
    return NRF_SUCCESS;
}











///////////////////




// === Add these to hr.c ===

// Read one register (public)
uint32_t max30102_read_reg(max30102_t *dev, uint8_t reg, uint8_t *val) {
    return i2c_read(dev->i2c, reg, val, 1);
}

// Read both FIFO pointers (public)
uint32_t max30102_get_fifo_pointers(max30102_t *dev, uint8_t *wr, uint8_t *rd) {
    if (!dev || !wr || !rd) return 1;
    uint32_t err = i2c_read(dev->i2c, MAX30102_REG_FIFO_WR_PTR, wr, 1);
    if (err != NRF_SUCCESS) return err;
    return i2c_read(dev->i2c, MAX30102_REG_FIFO_RD_PTR, rd, 1);
}

// Return number of complete (RED+IR) samples waiting (public)
uint8_t max30102_fifo_sample_count(max30102_t *dev) {
    uint8_t wr = 0, rd = 0;
    (void)max30102_get_fifo_pointers(dev, &wr, &rd);
    return (uint8_t)((wr - rd) & 0x1F);
}



//////////////////////////









uint32_t max30102_read_fifo_sample(max30102_t *dev, max30102_sample_t *out) {
    if (!dev || !out) return 1;
    uint8_t raw[6];
    uint32_t err = i2c_read(dev->i2c, MAX30102_REG_FIFO_DATA, raw, sizeof raw);
    if (err != NRF_SUCCESS) return err;

    // SpO₂ mode: RED first, then IR
    uint32_t red = ((uint32_t)raw[0]<<16)|((uint32_t)raw[1]<<8)|raw[2];
    uint32_t ir  = ((uint32_t)raw[3]<<16)|((uint32_t)raw[4]<<8)|raw[5];
    out->red = red & 0x3FFFF;
    out->ir  = ir  & 0x3FFFF;
    return NRF_SUCCESS;
}


uint32_t max30102_fifo_reset(max30102_t *dev)
{
    if (!dev) return 1;
    uint32_t err;
    uint8_t z = 0;

    err = i2c_write_u8(dev->i2c, MAX30102_REG_FIFO_WR_PTR, z);
    if (err != NRF_SUCCESS) return err;
    err = i2c_write_u8(dev->i2c, MAX30102_REG_OVF_COUNTER, z);
    if (err != NRF_SUCCESS) return err;
    err = i2c_write_u8(dev->i2c, MAX30102_REG_FIFO_RD_PTR, z);
    return err;
}





//

// ------------------------
// BPM helper implementation
// ------------------------

void max30102_bpm_init(max30102_t *dev)
{
    if (!dev) return;
    memset(&dev->bpm, 0, sizeof(dev->bpm));
    dev->bpm.armed = 1;
    dev->bpm.prev_x = 0;
    dev->bpm.prev2_x = 0;
    dev->bpm.sample_ctr = 0;
    dev->bpm.last_peak_sample = 0;
}


bool max30102_bpm_update(max30102_t *dev, uint32_t ir, uint32_t now_ms, uint16_t *bpm_out)
{
    (void)now_ms; // sample-based timing
    if (!dev || !bpm_out) return false;
    max30102_bpm_state_t *s = &dev->bpm;

    // ---- 1) DC removal (EMA, alpha ≈ 1/32; Q8) ----
    if (s->avg_q8 == 0) s->avg_q8 = (ir << 8);
    s->avg_q8 += ( ((int32_t)(ir << 8) - (int32_t)s->avg_q8) >> 5 );

    int32_t x  = (int32_t)ir - (int32_t)(s->avg_q8 >> 8);  // AC component
    int32_t ax = (x >= 0) ? x : -x;

    // ---- 2) EMAs of magnitude and slope (alpha ≈ 1/16) ----
    int32_t dx  = x - s->prev_x;
    int32_t adx = (dx >= 0) ? dx : -dx;

    s->ema_ax = s->ema_ax - (s->ema_ax >> 4) + (uint32_t)(ax  >> 4);
    s->ema_dx = s->ema_dx - (s->ema_dx >> 4) + (uint32_t)(adx >> 4);

    // ---- 3) Dynamic thresholds (more permissive) ----
    // small floors keep us sensitive; also require a minimal signal level
    const uint32_t MAG_FLOOR        = 4;
    const uint32_t SLOPE_FLOOR      = 2;
    const uint32_t SIGNAL_MIN_AX    = 25;   // don't try to detect when |x| is tiny

    // slightly softer than 1.0*EMA to catch weaker pulses
    uint32_t thr_mag   = (s->ema_ax * 3u) / 4u + 8;   // ≈0.75*ema_ax + 8
    uint32_t thr_slope = (s->ema_dx >> 1) + 4;        // ≈0.5*ema_dx + 4

    if (thr_mag   < MAG_FLOOR)   thr_mag   = MAG_FLOOR;
    if (thr_slope < SLOPE_FLOOR) thr_slope = SLOPE_FLOOR;

    // ---- 4) Sample counter (assumes 100 Hz) ----
    s->sample_ctr++;

    if ((s->sample_ctr & 0xFF) == 0) {
        NRF_LOG_INFO("dbg: |x|=%ld thr_mag=%lu thr_slope=%lu",
                     (long)((x>=0)?x:-x),
                     (unsigned long)thr_mag,
                     (unsigned long)thr_slope);
    }

    // ---- 5) Beat detection: accept local maxima OR minima with slope gates ----
    const uint32_t MIN_SAMP = 35;   // ~350 ms -> 171 BPM max
    const uint32_t MAX_SAMP = 200;  // ~2000 ms -> 30 BPM min

    bool detected = false;

    if (s->armed && s->ema_ax > SIGNAL_MIN_AX) {
        int32_t p  = s->prev_x;
        int32_t pp = s->prev2_x;
        int32_t ap = (p >= 0) ? p : -p;

        int32_t rise  = p - pp;
        int32_t fall  = p - x;
        int32_t arise = (rise >= 0) ? rise : -rise;
        int32_t afall = (fall >= 0) ? fall : -fall;

        bool is_local_max = (p > pp) && (p > x);
        bool is_local_min = (p < pp) && (p < x);

        if ((is_local_max || is_local_min) &&
            (ap    > (int32_t)thr_mag)   &&
            (arise > (int32_t)thr_slope) &&
            (afall > (int32_t)thr_slope))
        {
            if (s->last_peak_sample != 0) {
                uint32_t ds = s->sample_ctr - s->last_peak_sample; // samples between beats
                if (ds >= MIN_SAMP && ds <= MAX_SAMP) {
                    // BPM = 6000 / ds  (6000 = 60 * 100 Hz)
                    uint32_t bpm = 6000u / ds;

                    // Smooth BPM with EMA (alpha = 1/4)
                    if (!s->have_ema) {
                        s->bpm_ema_q8 = (uint16_t)(bpm << 8);
                        s->have_ema   = 1;
                    } else {
                        s->bpm_ema_q8 = s->bpm_ema_q8 - (s->bpm_ema_q8 >> 2) + (uint16_t)(bpm << 6);
                    }
                    *bpm_out = (uint16_t)(s->bpm_ema_q8 >> 8);
                    detected = true;
                }
            }
            s->last_peak_sample = s->sample_ctr;
            s->armed = 0; // disarm until hysteresis re-arms
        }
    }

    // ---- 6) Hysteresis re-arm (quicker) ----
    // re-arm once the magnitude drops to ~25% of dynamic threshold
    if (!s->armed) {
        int32_t aprev = (s->prev_x >= 0) ? s->prev_x : -s->prev_x;
        if ((uint32_t)aprev < (thr_mag >> 2)) {  // was >>1
            s->armed = 1;
        }
    }

    // ---- 7) Update history ----
    s->prev2_x = s->prev_x;
    s->prev_x  = x;

    return detected;
}

