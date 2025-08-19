
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



/* GPS UART instance */
static const nrfx_uarte_t GPS_UART = NRFX_UARTE_INSTANCE(1);

#define GPS_RX_BUF_SIZE 256
static uint8_t gps_rx_buf[GPS_RX_BUF_SIZE];
gps_packet_t my_gps_data;

 uint8_t line_buf[256];
static size_t line_idx = 0;
volatile bool line_ready = false;

static void gps_uarte_event_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_RX_DONE:
          // Null-terminate and log


            if (p_event->data.rxtx.bytes < GPS_RX_BUF_SIZE)
                gps_rx_buf[p_event->data.rxtx.bytes] = '\0';
            NRF_LOG_INFO("%s", (uint32_t)gps_rx_buf);

            // Re-start reception for next data
            APP_ERROR_CHECK(nrfx_uarte_rx(&GPS_UART, gps_rx_buf, GPS_RX_BUF_SIZE));
            break;

 /*
  
        for (size_t i = 0; i < p_event->data.rxtx.bytes; i++)
        {
            char c = line_buf[i];

            if (c == '\n')   // end of sentence
            {
                line_buf[line_idx] = '\0';   // null-terminate
                line_idx = 0;
                line_ready = true;
            }
            else
            {
                if (line_idx < GPS_RX_BUF_SIZE - 1)
                {
                    line_buf[line_idx++] = c;
                }
                else
                {
                    // overflow → reset
                    line_idx = 0;
                }
            }
        }

        // restart reception
        APP_ERROR_CHECK(nrfx_uarte_rx(&GPS_UART, line_buf, sizeof(line_buf)));
    break;

    */

        case NRFX_UARTE_EVT_ERROR:
            NRF_LOG_ERROR("GPS UART error: 0x%08x", p_event->data.error.error_mask);
            // Clear errors and restart reception
            APP_ERROR_CHECK(nrfx_uarte_rx(&GPS_UART, gps_rx_buf, GPS_RX_BUF_SIZE));
            break;

        default:
            break;
    }
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

uint8_t gps_get_dms_degrees(long double decimal_degrees)
{
    return (uint8_t) (decimal_degrees);
}

uint8_t gps_get_dms_minutes(long double decimal_degrees)
{
    return (uint8_t)((decimal_degrees - get_dms_degrees(decimal_degrees))*60);
}

uint8_t gps_get_dms_seconds(long double decimal_degrees)
{
    return (uint8_t)((((decimal_degrees - get_dms_degrees(decimal_degrees))*60) - get_dms_minutes(decimal_degrees))*60);
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

























