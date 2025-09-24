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

/* GPS UART pins */
#define GPS_UART_TX 30
#define GPS_UART_RX 31

/* I2C pins */
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

/* GPS control pins */
#define PIN_GPS_PWR   15
#define PIN_GPS_RST   23
#define PIN_GPS_VBKUP 16

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