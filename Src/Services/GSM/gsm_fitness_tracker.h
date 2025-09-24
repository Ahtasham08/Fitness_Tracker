#ifndef GSM_FITNESS_TRACKER_H
#define GSM_FITNESS_TRACKER_H

#include <stdint.h>
#include <stdbool.h>

// UART configuration
#define GSM_UART_BAUDRATE 9600
#define GSM_UART_TIMEOUT 1000 // in milliseconds

// GSM module response buffer size
#define GSM_RESPONSE_BUFFER_SIZE 256

enum OPERATOR_MNC
{
    OPERATOR_JAZZ = 41001,    // Jazz (Mobilink) - Pakistan
    OPERATOR_TELENOR = 41006, // Telenor - Pakistan
    OPERATOR_ZONG = 41004,    // Zong - Pakistan
    OPERATOR_UFONE = 41003,   // Ufone - Pakistan
};

#define APN "ufone.internet"
#define USR ""
#define PWD ""

#define APN_STR "\"" APN "\",\""USR"\",\""PWD"\""

typedef enum {
    CMD_AT,
    CMD_ATE0,
    CMD_ATIFC,
    CMD_AT_IPR,
    CMD_AT_W,
    CMD_ATI,
    CMD_AT_GSN,
    CMD_AT_CPIN,
    CMD_AT_CMEE,
    CMD_AT_CREG_Q,
    CMD_AT_CREG_NETWORK_REGISTER,
    CMD_AT_CFUN_REBOOT,
    CMD_AT_COPS_SCAN,
    CMD_AT_COPS_PLMN,
    CMD_AT_COPS_Q,
    CMD_AT_COPS_MANUAL,
    CMD_AT_CSQ,
    CMD_AT_CGATT,
    CMD_AT_CGATT_Q,
    CMD_AT_QBAND_VALUE,
    CMD_AT_QIFGCNT,
    CMD_AT_QICSGP,
    CMD_AT_QICSGP_Q,
    CMD_AT_QIREGAPP_Q,
    CMD_AT_CGREG_Q,
    CMD_AT_QIACT_Q,
    CMD_AT_CGPADDR,
    CMD_AT_QPING,
    CMD_AT_QIMUX,
    CMD_AT_QIMODE,
    CMD_AT_QINDI,
    CMD_AT_QIDNSIP,
    CMD_AT_QIREGAPP,
    CMD_AT_QIACT,
    CMD_AT_QILOCIP,
    CMD_AT_QIDNSGIP,
    CMD_AT_QMTOPEN,
    CMD_AT_QMTOPEN_Q,
    CMD_AT_QIPROMPT,
    CMD_AT_QISDE,
    CMD_AT_QISEND_MQTT_CONNECT,
    CMD_AT_QMTCONN,
    CMD_AT_QMTCONN_Q,
    CMD_MQTT_CONNECT_PAYLOAD,
    CMD_AT_QMTSUB,
    CMD_AT_QMTPUB,
    CMD_AT_QISEND_KEEPALIVE,
    CMD_MQTT_KEEPALIVE_PAYLOAD,
    CMD_AT_QMTDISC,
    CMD_AT_QICLOSE,
    CMD_AT_QIDEACT,
    CMD_AT_QPOWD,
    CMD_POWER_ON,
    CMD_COUNT,
} GSM_Command;

enum GSM_Request
{
    GSM_REQ_NONE = CMD_COUNT+1,
    GSM_WAIT_CREG,
    GSM_WAIT_CGATT,
    GSM_WAIT_CSQ,
    GSM_WAIT_QIACT,
};

typedef enum GSM_ERROR_CODE{
    GSM_ERROR_SIM_NOT_READY = -1,
    GSM_ERROR_NO_NETWORK = -2,
    GSM_ERROR_GPRS_NOT_ATTACHED = -3,
    GSM_ERROR_PDP_CONTEXT_ACTIVATION_FAILED = -4,
    GSM_ERROR_MQTT_CONNECTION_FAILED = -5,
    GSM_ERROR_MQTT_SUBSCRIPTION_FAILED = -6,
    GSM_ERROR_MQTT_PUBLISH_FAILED = -7,
    GSM_ERR_PWR_OFF_FAILED = -8,
    GSM_ERROR_PWR_ON_FAILED = -9,
    GSM_ERROR_POOR_SIGNAL = -10,
    GSM_ERROR_NO_IMEI = -11,
    GSM_ERROR_GPRS_ATTACH_FAILED = -11,
    GSM_ERROR_MQTT_OPEN_FAILED = -12,
    GSM_ERROR_UNKNOWN = -99,
    
    GSM_SUCCESS = 0
}GSM_ErrorCode;

/* Command table with expected replies + timeouts */
typedef struct gsm_cmd {
    char const *cmd;          // command string to send (may be NULL for just waiting)
    char const *expect;       // substring or token we expect
    uint32_t    timeout_ms;   // max wait time for expect
} gsm_cmd_t;






// Function prototypes
/**
 * @brief Send an AT command to the GSM module and wait for the expected response.
 * 
 * @param command The AT command to send.
 * @param expectedResponse The expected response from the GSM module.
 * @param timeout Timeout in milliseconds to wait for the response.
 * @return true if the expected response is received, false otherwise.
 */

 
bool GSM_SendCommand(uint16_t cmd);
GSM_ErrorCode GSM_Init(void); // Initialize UART for GSM module communication
bool GSM_ReadResponse(char *responseBuffer, uint32_t bufferSize, uint32_t timeout);
bool GSM_CheckModule(void); // Check if the GSM module is responding
bool GSM_Setup(void); // Configure GSM module for operation
bool GSM_SendSMS(const char *phoneNumber, const char *message); // Send SMS
bool CheckNetworkStatus(void); // Check network registration status
void GSM_PowerOn(void); // Power on the GSM module
void GSM_PowerOff(void); // Power off the GSM module
void GSM_Reset(void); // Reset the GSM module
void GSM_EmergencyOff(void); // Emergency power off for GSM module
bool GSM_SendData(const uint8_t *data, size_t length);// Send data over GSM (e.g., via MQTT)

#endif // GSM_FITNESS_TRACKER_H