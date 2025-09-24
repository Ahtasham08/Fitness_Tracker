#include "boards_custom.h"
#include "COMMON_CONFIG.h"
#include "Communication.h"

#include "ble_fitness_tracker.h"
#include "gsm_fitness_tracker.h"

device_info_t device_info = {
    .device_name = DEVICE_NAME_BLE,
    .major = MAJOR_VERSION,
    .minor = MINOR_VERSION,
    .build = BUILD_VERSION,
    .gsm = {
        .initialized = false,
        .network_registered = false,
        .gprs_attached = false,
        .sim_ready = false,
        .mqtt_connected = false,
        .is_powered_on = false,
        .powerMode = 0,
        .ip_address = "",
        .model = "",
        .Revision = "",
        .imei = 0
    },
    .com_priority = GSM,
};

void Communication_Init(void)
{
    // Initialize communication services (e.g., UART, BLE, etc.)
    BLE_Init();
    GSM_ErrorCode error_code = GSM_Init();
    if(error_code != GSM_SUCCESS)
    {
       NRF_LOG_ERROR("GSM Init failed with code: %d", error_code);
       NRF_LOG_INFO("Falling back to BLE communication.");
       NRF_LOG_FLUSH();
       device_info.com_priority = BLE; // Fallback to BLE if GSM init fails
    }
}

void Communication_SendData(const uint8_t *data, uint16_t length)
{
    if (device_info.com_priority == GSM && device_info.gsm.initialized && device_info.gsm.network_registered)
    {
        // Send data via GSM
        if(GSM_SendData(data, length))
        {
            NRF_LOG_INFO("Data sent via GSM");
        }
        else
        {
            NRF_LOG_ERROR("Failed to send data via GSM, falling back to BLE");
            device_info.com_priority = BLE; // Fallback to BLE on failure
            BLE_SendData(data, length);
        }
    }
    else
    {
        // Send data via BLE
        BLE_SendData(data, length);
    }
}