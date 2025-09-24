/**
 * @file uart_fitness_tracker.h
 * @brief Nordic UARTE driver header file for Fitness Tracker project.
 * 
 * This file contains the declarations and configurations for the Nordic UARTE driver.
 * 
 * @author Ahtasham Baig
 * @date 2023
 */

#ifndef UART_FITNESS_TRACKER_H
#define UART_FITNESS_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stddef.h"
#include "string.h"

#include "nrf52840.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif

#include "nrfx_uarte.h"

extern nrfx_uarte_t gps_uarte;
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

enum {
    GSM_UART,   // GSM UARTE instance
    GPS_UART    // GPS UARTE instance
};

/**
 * @brief Initialize the UARTE driver with the specified configuration.
 * 
 * @param config Pointer to the UART configuration structure.
 * @return true if initialization is successful, false otherwise.
 */
bool uarte_init(uint8_t uart_type, void(*callback)(char*,uint8_t));

/**
 * @brief Transmit data over UART.
 *
 * @param data Pointer to the data buffer to be transmitted.
 * @param length Length of the data buffer.
 * @return true if transmission is successful, false otherwise.
 */
ret_code_t uarte_transmit(uint8_t uart_type, const uint8_t *p_data, size_t length);

ret_code_t uarte_abort(uint8_t uart_type);


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
void uarte_deinit(uint8_t uart_type);

/**
 * @brief Initialize UART specifically for GSM module communication.
 */
void gsm_uart_init(void(*callback)(char*,uint8_t));

/**
 *  @brief Initialize UART specifically for GPS module communication.
 */
void gps_uart_init(void(*callback)(char*,uint8_t));
/**
 * @brief Initialize the low-frequency clock required for UART operation.
 */
void lfclk_init(void);

void testUart();

#endif // uart_fitness_tracker_H