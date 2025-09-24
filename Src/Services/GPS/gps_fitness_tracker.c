
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "gps_fitness_tracker.h"
#include "boards_custom.h"
#include "COMMON_CONFIG.h"
#include "uart_fitness_tracker.h"

#include <time.h>

/* Timeout timer for GPS TX */
APP_TIMER_DEF(gps_tx_timer);

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

void Process_Gpsdata(char* data, uint8_t len)
{

}

/* Main GPS init */
void gps_init(void)
{
    gps_gpio_init();
    gps_uart_init(Process_Gpsdata);
}


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
    err = uarte_transmit(GPS_UART, data, length);
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
    uarte_abort(GPS_UART);
    uarte_abort(GPS_UART);

    // Deinitialize UARTE
    uarte_deinit(GPS_UART);
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

            err = nrfx_uarte_rx(&gps_uarte, gps_info.gps_uart_buffer, rx_len);
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
    
    uarte_init(GPS_UART,Process_Gpsdata);

    // Power and backup pins
    nrf_gpio_cfg_output(PIN_GPS_PWR);
    nrf_gpio_pin_set(PIN_GPS_PWR);      // Turn on GPS

    nrf_gpio_cfg_output(PIN_GPS_VBKUP);
    nrf_gpio_pin_set(PIN_GPS_VBKUP);    // Enable backup power

    gps_info.gps_init = true;
    return success;
}






///////////////////////////////////////////////////////////////////////////////

























