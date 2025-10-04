/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "COMMON_CONFIG.h"

#include "communication.h"
#include "GPS.h"
#include "i2c.h"
#include "lis3dh.h"
#include "adc_battery.h"
#include "max30102.h"

typedef struct
{
    gps_packet_t gps_data;
    uint16_t heart_rate;
    uint8_t spo2_level;
    uint8_t pct;
    uint16_t battery_voltage_mv;
    int16_t acc_x_mg;
    int16_t acc_y_mg;
    int16_t acc_z_mg;
} fitness_data_t;

fitness_data_t fitness_data;

static void print_reset_reason(void)
{
    uint32_t r = NRF_POWER->RESETREAS;
    NRF_LOG_INFO("RESETREAS=0x%08lx", (unsigned long)r);
    NRF_POWER->RESETREAS = r; // clear for next time
}

/**@brief Application main function.
 */

static inline uint32_t ms_now(void)
{
    uint32_t ticks = app_timer_cnt_get();
    return (uint32_t)((uint64_t)ticks * 1000ull / APP_TIMER_CLOCK_FREQ);
}

int main(void)
{
    log_init();
    print_reset_reason();
    // lfclk_init();
    NRF_LOG_INFO("Fitness Tracker started.");
    NRF_LOG_INFO("Firmware Version: %s", (uint32_t)FIRMWARE_VERSION);
    NRF_LOG_FLUSH();
     GPIO_INIT();
    gps_init();
    //gps_initialize();                 // Initialize GPS GPIOs + UARTE
    //gps_check_communication();
    //gps_config_device();
    NRF_LOG_INFO("GPS Initialized in main.");
    status_t gps_status;

    // gps_status = gps_data_read(&my_gps_data);

   
    Communication_Init();
// Enter main loop.

    i2c0_init();
    APP_ERROR_CHECK_BOOL(battery_adc_init());

    uint8_t who = 0;
    if (lis3dh_whoami(&who))
    {
        NRF_LOG_INFO("LIS3DHTR WHO_AM_I = 0x%02X", who); // expect 0x33
    }
    else
    {
        NRF_LOG_ERROR("WHO_AM_I read failed");
    }

    if (!lis3dh_init(LIS3DH_ODR_100HZ, LIS3DH_FS_2G))
    {
        NRF_LOG_ERROR("LIS3DHTR init failed");
    }
#if 0
    // HR Sensor START
    //  3) Configure sensor
    hr_bus_init();

     --- MAX30102 init (no GPIOTE) ---
     max30102_t hr;
     APP_ERROR_CHECK(max30102_init(&hr, &HR_TWIM));

    // Optional: confirm ID
     uint8_t id = 0;
     if (max30102_read_part_id(&hr, &id) == NRF_SUCCESS) {
     NRF_LOG_INFO("MAX30102 PART ID: 0x%02X (expect 0x15)", id);
     }

     APP_ERROR_CHECK(max30102_soft_reset(&hr));
     nrf_delay_ms(5);

    // Keep SR=100 Hz, PW=411us (code 3), but increase ADC range to max (code 3)
     APP_ERROR_CHECK(max30102_config_spo2(&hr, /*sr_hz=*/100, /*pw_code=*/3, /*adc_range_code=*/3));

    // Lower LED currents a lot to avoid clipping (0x10 ≈ 3.2 mA)
    // Try ~8 mA first
     APP_ERROR_CHECK(max30102_set_led_currents(&hr, 0x30, 0x30));  // (IR, RED)

    // If IR/RED still < 30k → try 0x40
    // If IR/RED start to approach 0x3FFFF → back down (or keep range=3 and lower current)
    // If IR/RED still < 30k → try 0x40
    // If IR/RED start to approach 0x3FFFF → back down (or keep range=3 and lower current)

    // Enter SPO2 mode now so samples are produced
     APP_ERROR_CHECK(max30102_set_mode_spo2(&hr));

    // FIFO: avg=4, rollover OFF, A_FULL≈16 samples
     APP_ERROR_CHECK(max30102_fifo_setup(&hr, /*sample_avg=*/4, /*roll_over=*/false, /*fifo_afull=*/0x0F));
     APP_ERROR_CHECK(max30102_fifo_reset(&hr));

    // Enable PPG_RDY only for now (simpler)
     APP_ERROR_CHECK(max30102_enable_interrupts(&hr, /*data_ready=*/true, /*alc_ovf=*/false, /*prox=*/false, /*die_temp=*/false));

    // Clear any latched IRQs so INT starts clean
    (void)max30102_clear_interrupts(&hr);

    // Optional: toss first few samples (settle)
     for (int i = 0; i < 8; i++) {
     max30102_sample_t dummy;
    (void)max30102_read_fifo_sample(&hr, &dummy);
     }

    // BPM estimator state
     max30102_bpm_init(&hr);

    // (optional) quick readback of key regs
     uint8_t r=0;
     max30102_read_reg(&hr, MAX30102_REG_MODE_CONFIG, &r);  NRF_LOG_INFO("MODE=0x%02X", r);
     max30102_read_reg(&hr, MAX30102_REG_SPO2_CONFIG, &r);  NRF_LOG_INFO("SPO2_CFG=0x%02X", r);
     max30102_read_reg(&hr, MAX30102_REG_FIFO_CONFIG, &r);  NRF_LOG_INFO("FIFO_CFG=0x%02X", r);
     max30102_read_reg(&hr, MAX30102_REG_INT_ENABLE1, &r);  NRF_LOG_INFO("INT_EN1=0x%02X", r);
#endif

    static char buffer[100];
    gps_packet_t fix;
    for (;;)
    {
        idle_state_handle();

        if (gps_service())
        {

            if (gps_get_fix(&fix))
            {
                if (fix.isValid)
                {
                    log_fix(&fix);
                }
                else
                {
                    NRF_LOG_WARNING("GPS fix invalid");
                }
            }
            fitness_data.gps_data = fix;
        }
        else
        {
            NRF_LOG_WARNING("GPS fix invalid");
            gps_debug_raw();
            fitness_data.gps_data.isValid = false;
        }

        if (lis3dh_read_xyz_mg(&fitness_data.acc_x_mg, &fitness_data.acc_y_mg, &fitness_data.acc_z_mg))
        {
            NRF_LOG_INFO("ACC X=%d mg Y=%d mg Z=%d mg",
                         fitness_data.acc_x_mg, fitness_data.acc_y_mg, fitness_data.acc_z_mg);
        }
        else
        {
            NRF_LOG_ERROR("ACC read failed");
        }

        // after battery_adc_init(), anywhere you print VBAT:

        if (battery_read_mv(&fitness_data.battery_voltage_mv))
        {
            fitness_data.pct = battery_percent_from_mv(fitness_data.battery_voltage_mv);
            NRF_LOG_INFO("VBAT = %u mV (%u%%)", fitness_data.battery_voltage_mv, fitness_data.pct);
        }

        // HR Sensor
#if 0
         if (hr_int_is_asserted()) {
             (void)max30102_clear_interrupts(&hr);            // releases INT latch
             uint8_t n = max30102_fifo_sample_count(&hr);     // how many complete samples?
             for (uint8_t i = 0; i < n; i++) {
                 max30102_sample_t s;
                 if (max30102_read_fifo_sample(&hr, &s) != NRF_SUCCESS) break;

                uint16_t bpm;
                if (max30102_bpm_update(&hr, s.ir, ms_now(), &bpm)) {
                    NRF_LOG_INFO("BPM ~ %u", bpm);
                }
                if ((i & 0x03) == 0) {
                    NRF_LOG_INFO("IR=%lu RED=%lu", (unsigned long)s.ir, (unsigned long)s.red);
                }
            }
        }
#endif
        snprintf(buffer, sizeof(buffer), "HR: %u, SpO2: %u%%, Battery: %umV, ACC: X=%d Y=%d Z=%d, GPS fix: %d GPS_lon: %.6f GPS_lat: %.6f Alt: %.2f",
                 fitness_data.heart_rate,
                 fitness_data.spo2_level,
                 fitness_data.battery_voltage_mv,
                 fitness_data.acc_x_mg, fitness_data.acc_y_mg, fitness_data.acc_z_mg,
                 fitness_data.gps_data.isValid,
                 (double)fitness_data.gps_data.lon,
                 (double)fitness_data.gps_data.lat,
                 (double)fitness_data.gps_data.altitude);
        NRF_LOG_INFO("%s", buffer);
        Communication_SendData((uint8_t *)buffer, strlen(buffer));
        // ~50 Hz polling is fine for 100 Hz sensor
        nrf_delay_ms(20);

        NRF_LOG_FLUSH();
    }
}

/**
 * @}
 */
