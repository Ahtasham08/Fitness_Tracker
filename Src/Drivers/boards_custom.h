#ifndef BOARDS_CUSTOM_H
#define BOARDS_CUSTOM_H

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

#define DEVICE_NAME_BLE                    "TestPCB_"                               /**< Name of device. Will be included in the advertising data. */

#define MAJOR_VERSION 1
#define MINOR_VERSION 0
#define BUILD_VERSION 0

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
/* GSM UART pins */
#define GSM_UART_TX NRF_GPIO_PIN_MAP(1,14)
#define GSM_UART_RX NRF_GPIO_PIN_MAP(1,15)
#define GSM_MCU_STATUS NRF_GPIO_PIN_MAP(1,12) // Input from GSM to MCU
#define GSM_MCU_RST   NRF_GPIO_PIN_MAP(0,18) // Input from GSM to MCU
#define GSM_MCU_DTR   NRF_GPIO_PIN_MAP(1,4) // Output from MCU to GSM

/* Board power pin*/
#define MCU_EN3V8 NRF_GPIO_PIN_MAP(0,11) // Output from MCU to enable 3.8V power supply
/* GSM Control Pins */
#define PIN_GSM_PWR NRF_GPIO_PIN_MAP(1,5)  // Power control pin for GSM module
#define PIN_GSM_PWRKEY NRF_GPIO_PIN_MAP(0,21)
#define PIN_GSM_EMERG_OFF NRF_GPIO_PIN_MAP(1,3)

#define TIME_SENDING_PACKET  1 // 900 for 15 minutes

typedef enum COMMUNICATION_PRIORITY{
    GSM,
    BLE
};

typedef struct gsm_state_s {
    bool initialized;
    bool network_registered;
    bool gprs_attached;
    bool sim_ready;
    bool mqtt_connected;
    bool is_powered_on;
    uint8_t powerMode;
    char ip_address[16]; // Assuming IPv4
    char model[32];     // Model name
    char Revision[16]; // Revision string
    long long imei;      // IMEI is 15 digits + null terminator
} gsm_state_t;


typedef struct device_info_s {
    char device_name[32];
    uint8_t major;
    uint8_t minor;
    uint8_t build;
    bool deviceConnected;
  
    bool TimeElapsed;
    bool isDeviceStatusChanged;
    bool isNotification;
    bool isTimeSyncRequired;
    uint32_t TimeSynctime;
    uint8_t ble_retries;
    gsm_state_t gsm;
    uint8_t com_priority;
} device_info_t;

extern device_info_t device_info;


#endif // BOARDS_CUSTOM_H