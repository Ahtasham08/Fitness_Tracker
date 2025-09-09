#pragma once
#include "nrf_gpio.h"
#include "nrf_uarte.h"   // add this near the top



// If your project already defines status_t (success/fail/pending), remove this block.

#pragma once
#ifndef APP_STATUS_T_DEFINED
#define APP_STATUS_T_DEFINED
typedef enum { fail = 0, success = 1, pending = 2 } status_t;
#endif



/* GPS UART pins */
#define GPS_UART_TX 30
#define GPS_UART_RX 31

/* GPS control pins */
#define PIN_GPS_PWR   15
#define PIN_GPS_RST   23
#define PIN_GPS_VBKUP 16


// I2C / LIS3DHTR pins
#define BOARD_I2C0_SCL_PIN    26   // P0.26 on your schematic
#define BOARD_I2C0_SDA_PIN    27   // P0.27 on your schematic
#define LIS3DH_CS_PIN     -1   // -1 means chip select is tied high (I2C mode)


// Battery measurement
#define PIN_BAT_EN           12            // P0.12 → BAT-EN
// SAADC input for BAT-ADC (P0.02 / AIN0)
#define BATTERY_ADC_INPUT    NRF_SAADC_INPUT_AIN0

// Divider values (ohms)
#define BAT_R_TOP_OHMS       3000.0f       // R14
#define BAT_R_BOTTOM_OHMS    8870.0f       // R16


//HR pins
// ---------- Heart-rate sensor (MAX30102) ----------
#define HR_I2C_SCL_PIN   NRF_GPIO_PIN_MAP(1,10)   // I2C3_SCL
#define HR_I2C_SDA_PIN   NRF_GPIO_PIN_MAP(0,3)    // I2C3_SDA
#define HR_INT_PIN       NRF_GPIO_PIN_MAP(1,11)   // HR-INT (active low)



//M95 Pins

// ---------- M95 (GSM) UART on UARTE0 ----------
#define M95_UART_TX   NRF_GPIO_PIN_MAP(1,14)  // MCU-TX  -> M95-RX  (Z3 → P1.14)
#define M95_UART_RX   NRF_GPIO_PIN_MAP(1,15)  // MCU-RX  <- M95-TX  (Z4 → P1.15)

#define M95_UART_RTS  NRF_UARTE_PSEL_DISCONNECTED
#define M95_UART_CTS  NRF_UARTE_PSEL_DISCONNECTED

// ---------- M95 control pins ----------
#define PIN_GSM_PWR_EN  NRF_GPIO_PIN_MAP(1,5)   // POGSM0PEN enable (D6 → P1.05)
#define PIN_GSM_PWRKEY  NRF_GPIO_PIN_MAP(0,21)  // MCU-PWRKEY (E2 → P0.21)
#define PIN_GSM_EMERG   NRF_GPIO_PIN_MAP(1,3)   // MCU-EMERG  (B6 → P1.03)
#define PIN_GSM_STATUS  NRF_GPIO_PIN_MAP(1,12)  // MCU-STATUS (Z1 → P1.12)
#define PIN_GSM_DTR     NRF_GPIO_PIN_MAP(1,4)   // MCU-DTR    (C6 → P1.04)  // optional; set false if not used
