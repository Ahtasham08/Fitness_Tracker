/*
 * GPS.h
 *
 *  Created on: Aug 17, 2025
 *      Author: 
 */


#pragma once


#include "nrf_gpio.h"
#include "nrf_log.h"





#ifndef DR_GPS_LC76F_H_
#define DR_GPS_LC76F_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

//#include "api_ab_uart.h"
//#include "firmware_defaults.h"
//#include "unit_testing.h"
//#include "common_status.h"

typedef struct
{
    bool isValid;
    uint8_t valid_data;
    uint8_t fixQuality;
    uint8_t numSatellites;
    uint8_t *checksum;
    uint8_t unit;


    float time;                               /*time Seconds*/
    uint16_t timeHours;                       /*time Hours*/
    uint16_t timeMinutes;                     /*time Minutes*/
    uint16_t timeSeconds;                     /*time Seconds*/


    float heightOfGeoid;
    float altitude;
    float horizontalDilution;
    long double lat;                          /*LAT decimal degree format */
    long double lon;                          /*LON decimal degree format */

    uint8_t     lathem;                     /*hemisphere N(North) or S(South) */
    uint8_t     lat_degrees;
    uint8_t     lat_minutes;
    uint8_t     lat_seconds;

    uint8_t     lonhem;                      /*hemisphere E(East) or W(West) */
    uint8_t     long_degrees;
    uint8_t     long_minutes;
    uint8_t     long_seconds;

}gps_packet_t;

typedef enum {
    fail = 0,
    success = 1,
    pending = 2
} status_t;



typedef struct {
    bool rtv_gps_power_status;
    bool rtv_gps_fixed_flag;
    bool rtv_type_bin_pickup;
    bool rtv_fence_enable;
    status_t rtv_gps_fixed_activity_flag;
} ap_data_t;

extern ap_data_t ap_data;   // declare in header




/* Macros */
#define GPS_BAUD_RATE    u_buad_9600
#define GNGGA_LENGTH 128
// Make GPS data global so it persists
// Declaration only
extern gps_packet_t my_gps_data;
extern  volatile bool line_ready;

extern uint8_t line_buf[256];
/* Functions */

void gps_process_data(uint8_t *data, size_t len);


status_t gps_initialize();
status_t gps_low_power_mode(void);
status_t gps_deinitialize(void);
status_t gps_uart_read(uint8_t* data, uint16_t timeout_ms, uint16_t* size);
status_t gps_extract_data_string(uint8_t* gps_data, uint8_t* gngga);
status_t gps_data_read(gps_packet_t *gps_ptr);
status_t gps_read();
status_t gps_write(uint8_t* data , uint32_t timeout );
status_t gps_config_power_v_bckp_enable();
status_t gps_config_power_v_bckp_disable();
status_t gps_config_power_vcc_enable();
status_t gps_config_power_vcc_disable();



/* Functions  of related to parseling*/
void gps_clear();
void gps_clear_packet(gps_packet_t * loc);
void gps_convert_decimal_deg_dms(uint8_t* degrees, uint8_t* minutes, uint8_t* seconds, float dd);

float gps_knots_to_mph(float knots);

long double gps_convert_to_decimal_degrees(uint16_t degree, float minSec);
long double gps_convert_degminsec_decimal_deg(uint8_t degrees, uint8_t minutes, uint8_t seconds);

uint32_t gps_unix_timestamp(uint16_t year,uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, uint16_t second);

uint8_t  gps_get_dms_degrees(long double decimal_degrees);
uint8_t  gps_get_dms_minutes(long double decimal_degrees);
uint8_t  gps_get_dms_seconds(long double decimal_degrees);

status_t gps_parse(uint8_t * input, gps_packet_t * packet);

status_t gps_check_communication();


void gps_init(void);


#endif



