#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "nrfx_twim.h"

// 7-bit I2C address
#define MAX30102_I2C_ADDR         0x57

// Registers (subset)
#define MAX30102_REG_INT_STATUS1  0x00
#define MAX30102_REG_INT_STATUS2  0x01
#define MAX30102_REG_INT_ENABLE1  0x02
#define MAX30102_REG_INT_ENABLE2  0x03
#define MAX30102_REG_FIFO_WR_PTR  0x04
#define MAX30102_REG_OVF_COUNTER  0x05
#define MAX30102_REG_FIFO_RD_PTR  0x06
#define MAX30102_REG_FIFO_DATA    0x07
#define MAX30102_REG_FIFO_CONFIG  0x08
#define MAX30102_REG_MODE_CONFIG  0x09
#define MAX30102_REG_SPO2_CONFIG  0x0A
#define MAX30102_REG_LED1_PA      0x0C
#define MAX30102_REG_LED2_PA      0x0D
#define MAX30102_REG_PART_ID      0xFF

#define MAX30102_EXPECTED_PART_ID 0x15







typedef struct {
    uint32_t ir;   // 18-bit valid
    uint32_t red;  // 18-bit valid
} max30102_sample_t;

// ---- BPM helper state (lightweight) ----


typedef struct {
    uint32_t avg_q8;
    uint32_t thr;
    uint32_t last_peak_ms;
    uint8_t  armed;
    uint16_t bpm_ema_q8;
    uint8_t  have_ema;

    int32_t  prev_x;         // NEW: for local maxima
    int32_t  prev2_x;        // NEW: for local maxima

    // --- NEW: sample-based timing (robust even if ms tick jitters) ---
    uint32_t sample_ctr;         // increments every sample fed to bpm_update()
    uint32_t last_peak_sample;   // sample count at last detected peak

    uint32_t ema_ax;      // EMA of |x| in Q0 (no shift)
    uint32_t ema_dx;      // EMA of |dx| (slope magnitude) in Q0
} max30102_bpm_state_t;




typedef struct {
    const nrfx_twim_t *i2c;
    max30102_bpm_state_t bpm;
} max30102_t;

uint32_t max30102_init(max30102_t *dev, const nrfx_twim_t *i2c);
uint32_t max30102_read_part_id(max30102_t *dev, uint8_t *out);
uint32_t max30102_soft_reset(max30102_t *dev);
uint32_t max30102_config_spo2(max30102_t *dev, uint16_t sr_hz, uint8_t pw_code, uint8_t adc_range_code);
uint32_t max30102_set_mode_spo2(max30102_t *dev);
uint32_t max30102_set_led_currents(max30102_t *dev, uint8_t ir, uint8_t red);
uint32_t max30102_fifo_setup(max30102_t *dev, uint8_t sample_avg, bool roll_over, uint8_t fifo_afull);
uint32_t max30102_enable_interrupts(max30102_t *dev, bool data_ready, bool alc_ovf, bool prox, bool die_temp);
uint32_t max30102_clear_interrupts(max30102_t *dev);

// Read one IR/RED pair from FIFO (blocking)
uint32_t max30102_read_fifo_sample(max30102_t *dev, max30102_sample_t *out);


uint32_t max30102_fifo_reset(max30102_t *dev);



uint32_t max30102_read_reg(max30102_t *dev, uint8_t reg, uint8_t *val);
uint32_t max30102_get_fifo_pointers(max30102_t *dev, uint8_t *wr, uint8_t *rd);
uint8_t  max30102_fifo_sample_count(max30102_t *dev);




// Initialize/reset the BPM estimator state (call once after HR config)
void max30102_bpm_init(max30102_t *dev);

// Feed one IR sample with current time in ms; returns true if a new BPM is ready
bool max30102_bpm_update(max30102_t *dev, uint32_t ir, uint32_t now_ms, uint16_t *bpm_out);
