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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include "sdk_config.h"
#include "nrfx.h"
#include "nrfx_twim.h"
#include "nrfx_uarte.h"

// ---- C stdlib
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ---- nRF / SoftDevice / BLE framework
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"

// ---- Logging (enable in sdk_config if you use it)
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


// +++ ADD THESE 3 LINES +++
#include "nrf_gpio.h"                     // for NRF_GPIO_PIN_MAP()
#define LOG_SHIM_TX_PIN  NRF_GPIO_PIN_MAP(0,6)   // CH340 RXD wire here
#define LOG_SHIM_BAUD    9600
#include "log_softuart_shim.h"            // redirects NRF_LOG_* → SoftUART
// +++ END ADD +++



// ---- GPIO / delays
#include "nrf_gpio.h"
#include "nrf_delay.h"

// ---- App headers
#include "GPS.h"
#include "i2c.h"
#include "lis3dh.h"
#include "adc_battery.h"
#include "max30102.h"
#include "m95.h"



#define LED_PIN 13  // Or any pin you want



void gpio_init(void) {
   // nrf_gpio_cfg_output(LED_PIN);
}


APP_TIMER_DEF(m_blinky_timer_id);

static void blinky_timeout_handler(void * p_context) {
    nrf_gpio_pin_toggle(LED_PIN);
}


static const nrfx_twim_t m_twim0 = NRFX_TWIM_INSTANCE(0);

#define I2C_SCL_PIN 26   // your SCL
#define I2C_SDA_PIN 27   // your SDA

static void twim_init(void)
{
    nrfx_twim_config_t config = {
        .scl                = I2C_SCL_PIN,
        .sda                = I2C_SDA_PIN,
        .frequency          = NRF_TWIM_FREQ_100K,
        .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .hold_bus_uninit    = false
    };
    APP_ERROR_CHECK(nrfx_twim_init(&m_twim0, &config, NULL, NULL));
    nrfx_twim_enable(&m_twim0);
}


static void i2c_scan(void)
{
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        // Zero-length write = probe
        nrfx_err_t err = nrfx_twim_tx(&m_twim0, addr, NULL, 0, false);
        if (err == NRFX_SUCCESS)
        {
            NRF_LOG_INFO("Found device at 0x%02X", addr);
            NRF_LOG_FLUSH();   // force print immediately
        }
        nrf_delay_ms(5);
    }
}


static bool i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *val)
{
    nrfx_err_t err;
    // Write register address, then repeated-start read 1 byte
    err = nrfx_twim_tx(&m_twim0, addr7, &reg, 1, true);  // no STOP
    if (err != NRFX_SUCCESS) return false;
    err = nrfx_twim_rx(&m_twim0, addr7, val, 1);
    return (err == NRFX_SUCCESS);
}

static void lis3dh_whoami_probe(void)
{
    uint8_t who = 0;
    const uint8_t addrs[] = { 0x18, 0x19 }; // SA0 low / high

    for (size_t i = 0; i < sizeof(addrs); i++) {
        bool ok = i2c_read_reg(addrs[i], 0x0F, &who); // WHO_AM_I register
        if (ok) {
            NRF_LOG_INFO("LIS3DHTR at 0x%02X WHO_AM_I=0x%02X", addrs[i], who);
            NRF_LOG_FLUSH();
        } else {
            NRF_LOG_INFO("No ACK from 0x%02X (WHO_AM_I)", addrs[i]);
            NRF_LOG_FLUSH();
        }
        nrf_delay_ms(5);
    }
}


////////////////////////////////////////////////////HR



static void hr_service(max30102_t* dev)
{
    (void)max30102_clear_interrupts(dev);

    uint8_t n = max30102_fifo_sample_count(dev);
    if (!n) return;

    for (uint8_t i = 0; i < n; i++) {
        max30102_sample_t s;
        if (max30102_read_fifo_sample(dev, &s) != NRF_SUCCESS) break;

        uint16_t bpm;
        if (max30102_bpm_update(dev, s.ir, 0, &bpm)) {
            NRF_LOG_INFO("BPM ≈ %u", bpm);
        }
        if ((i & 0x03) == 0) {
            NRF_LOG_INFO("IR=%lu RED=%lu", (unsigned long)s.ir, (unsigned long)s.red);
        }
    }
}




static inline uint32_t ms_now(void) {
    uint32_t ticks = app_timer_cnt_get();
    return (uint32_t)((uint64_t)ticks * 1000ull / APP_TIMER_CLOCK_FREQ);
}

////////////////////////////////////////////////////GPS

#define DEVICE_NAME                     "Nordic_Template"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */

    err_code = app_timer_create(&m_blinky_timer_id, APP_TIMER_MODE_REPEATED, blinky_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
  /*  if (NRF_LOG_PROCESS() == false) //previous
    {
        nrf_pwr_mgmt_run(); 
    }
    */

    //chaanged here

     if (!NRF_LOG_PROCESS()) {
        __WFE(); __SEV(); __WFE();   // light sleep
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}





// put near the top of main.c
static void m95_poll_lines(void)
{
    char ln[160];
    for (;;) {
        if (!m95_service()) break;
        if (m95_get_line(ln, sizeof ln)) {
            NRF_LOG_INFO("M95> %s", nrf_log_push(ln));
        }
    }
}




static void m95_gpio_bringup_once(void)
{
    // 0) defaults
    nrf_gpio_cfg_output(PIN_GSM_PWR_EN);
    nrf_gpio_cfg_output(PIN_GSM_PWRKEY);
    nrf_gpio_cfg_output(PIN_GSM_EMERG);
    nrf_gpio_cfg_output(PIN_GSM_DTR);
   // before enabling the rail
nrf_gpio_cfg_input(PIN_GSM_STATUS, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_pin_clear(PIN_GSM_EMERG); // do NOT emergency-off
    nrf_gpio_pin_clear(PIN_GSM_DTR);   // keep UART awake (DTR low)

    // 1) enable rail & settle
    nrf_gpio_pin_set(PIN_GSM_PWR_EN);
    nrf_delay_ms(1500);
    NRF_LOG_INFO("M95: STATUS after rail = %d (LOW@MCU means ON)", nrf_gpio_pin_read(PIN_GSM_STATUS));

    // 2) assert PWRKEY (MCU HIGH) for 2.0 s
    NRF_LOG_INFO("M95: PWRKEY HIGH 2000 ms");
    nrf_gpio_pin_set(PIN_GSM_PWRKEY);
    nrf_delay_ms(2000);
    nrf_gpio_pin_clear(PIN_GSM_PWRKEY);
    NRF_LOG_INFO("M95: PWRKEY released");

    // 3) wait for boot and watch STATUS flip
    for (int i = 0; i < 60; i++) { // ~6 s total
        int st = nrf_gpio_pin_read(PIN_GSM_STATUS);
        NRF_LOG_INFO("M95: STATUS=%d", st);
        nrf_delay_ms(100);
    }

    // 4) ping at 9600
    (void)m95_send_cmd("AT");
    // drain ~3 s to catch OK
    uint32_t start = app_timer_cnt_get();
    char ln[128];
    while (((app_timer_cnt_get() - start) * 1000ULL / 32768ULL) < 3000) {
        while (m95_service()) {
            if (m95_get_line(ln, sizeof ln)) NRF_LOG_INFO("M95> %s", nrf_log_push(ln));
        }
        nrf_delay_ms(5);
    }
}


static void print_reset_reason(void){
    uint32_t r = NRF_POWER->RESETREAS;
    NRF_LOG_INFO("RESETREAS=0x%08lx", (unsigned long)r);
    NRF_POWER->RESETREAS = r; // clear for next time
}








static bool cmd_expect(const char *cmd, const char *must_see, uint32_t timeout_ms);
static void drain_logs(uint32_t ms);

void app_start(void)
{
    m95_init();

    if (!m95_power_on()) {
        NRF_LOG_ERROR("M95: power-on failed");
        return;
    }

    // Catch any boot spew
    drain_logs(400);

    // Basic sanity + make responses predictable
    (void)cmd_expect("AT",     NULL,   800);
    (void)cmd_expect("ATE0",   NULL,   800);   // echo off
    (void)cmd_expect("ATI",    NULL,  1500);   // module ID + OK
    (void)cmd_expect("AT+IPR?",NULL,  1000);   // confirm baud

    // SIM / RF / Registration
    (void)cmd_expect("AT+CMEE=2", NULL, 800);  // verbose errors
    (void)cmd_expect("AT+CPIN?",  "CPIN:", 1500); // expect "CPIN: READY" then OK
    (void)cmd_expect("AT+CSQ",    "CSQ:", 1500);  // signal (0..31 or 99)
    (void)cmd_expect("AT+CREG=2", NULL,  800);    // enable URC +CREG: <stat>
    (void)cmd_expect("AT+CREG?",  "CREG:", 2000); // expect +CREG: 0,1 or 0,5 then OK
    (void)cmd_expect("AT+COPS?",  "+COPS:", 2000);// serving operator

    // (Optional) SMS ping to prove TX/RX fully:
    // (void)cmd_expect("AT+CMGF=1", NULL,  800); // text mode
    // then handle CMGS flow later
}



static bool cmd_expect(const char *cmd, const char *must_see, uint32_t timeout_ms);
static void drain_logs(uint32_t ms);

void app_m95_start(void)
{
    m95_init();

    if (!m95_power_on()) {
        NRF_LOG_ERROR("M95: power-on failed");
        return;
    }

    // Catch any boot spew
    drain_logs(400);

    // Basic sanity + make responses predictable
    (void)cmd_expect("AT",     NULL,   800);
    (void)cmd_expect("ATE0",   NULL,   800);   // echo off
    (void)cmd_expect("ATI",    NULL,  1500);   // module ID + OK
    (void)cmd_expect("AT+IPR?",NULL,  1000);   // confirm baud

    // SIM / RF / Registration
    (void)cmd_expect("AT+CMEE=2", NULL, 800);  // verbose errors
    (void)cmd_expect("AT+CPIN?",  "CPIN:", 1500); // expect "CPIN: READY" then OK
    (void)cmd_expect("AT+CSQ",    "CSQ:", 1500);  // signal (0..31 or 99)
    (void)cmd_expect("AT+CREG=2", NULL,  800);    // enable URC +CREG: <stat>
    (void)cmd_expect("AT+CREG?",  "CREG:", 2000); // expect +CREG: 0,1 or 0,5 then OK
    (void)cmd_expect("AT+COPS?",  "+COPS:", 2000);// serving operator

    // (Optional) SMS ping to prove TX/RX fully:
    // (void)cmd_expect("AT+CMGF=1", NULL,  800); // text mode
    // then handle CMGS flow later
}



/**@brief Function for application main entry.
 */
int main(void)
{

 print_reset_reason();
    bool erase_bonds;
    ret_code_t err_code;   // Declare the variable here

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    //power_management_init(); //commenetd out for now changed here 

   // ble_stack_init();
   // gap_params_init();
   // gatt_init();
   // advertising_init();
   // services_init();
   // conn_params_init();
    //peer_manager_init();


    debug_port_init();   // SoftUART up (9600 8N1). You’ll see a banner in Docklight.


    // Start execution.
    NRF_LOG_INFO("Fitness Tracker Application started.");
    application_timers_start();

//    advertising_start(erase_bonds);


    gpio_init();                  // Your general GPIOs
    gps_init();                   // Initialize GPS GPIOs + UARTE
    NRF_LOG_INFO("GPS Initialized in main.");

    // Start timer (period 500 ms)
    err_code = app_timer_start(m_blinky_timer_id, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);

    status_t gps_status;

   // gps_status = gps_data_read(&my_gps_data);

    //twim_init();

    //NRF_LOG_INFO("I2C scan start");
   // i2c_scan();

      // NRF_LOG_INFO("Probing LIS3DHTR WHO_AM_I...");
      // NRF_LOG_FLUSH();

    //lis3dh_whoami_probe(); // <-- run this

       i2c0_init();
       APP_ERROR_CHECK_BOOL(battery_adc_init());



    uint8_t who = 0;
    if (lis3dh_whoami(&who)) {
        NRF_LOG_INFO("LIS3DHTR WHO_AM_I = 0x%02X", who); // expect 0x33
    } else {
        NRF_LOG_ERROR("WHO_AM_I read failed");
    }

    if (!lis3dh_init(LIS3DH_ODR_100HZ, LIS3DH_FS_2G)) {
        NRF_LOG_ERROR("LIS3DHTR init failed");
    }

    //HR Sensor START
    // 3) Configure sensor
    hr_bus_init();





    // 2) Init the sensor (no GPIOTE)
// --- MAX30102 init (no GPIOTE) ---
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


    //Hr Sensor Config END

      m95_init();

      if (!m95_power_on()) {
          NRF_LOG_ERROR("M95: power-on failed");
          // If you're inside int main(void):
          // return 0;          // or: for(;;) { nrf_delay_ms(1000); }
          // If you're inside a void init function:
          // return;
      }

      // Simple AT smoke test
      m95_send_cmd("AT");
      m95_send_cmd("ATE0");        // echo off
      m95_send_cmd("ATI");         // identify
      m95_send_cmd("AT+CPIN?");    // SIM state
      m95_send_cmd("AT+CSQ");      // signal
      m95_send_cmd("AT+CREG=2");   // URCs on
      m95_send_cmd("AT+CREG?");    // registration

      // Drain replies for ~5s (500 * 10ms)
      char ln[192];
      for (int i = 0; i < 500; ++i) {
          while (m95_service()) {
              if (m95_get_line(ln, sizeof ln)) {
                  NRF_LOG_INFO("M95> %s", nrf_log_push(ln));
              }
          }
          nrf_delay_ms(10);
      }


 /*   
      m95_init();
      if (!m95_power_on()) {
          NRF_LOG_ERROR("M95 power-on failed");
      } else {
        (void)m95_send_cmd("AT");   // first ping at 9600
  nrf_delay_ms(50);
          // basic probe (non-blocking)
          (void)m95_send_cmd("ATE0");     // echo off
          (void)m95_send_cmd("ATI");      // ID
          (void)m95_send_cmd("AT+CPIN?"); // SIM?
          (void)m95_send_cmd("AT+CSQ");   // RSSI
          (void)m95_send_cmd("AT+CREG?"); // network reg
            }
  
        // after the block where you send ATE0/ATI/CPIN/CSQ/CREG
        uint32_t until_ms = 1500;   // drain for ~1.5 s
        uint32_t start = app_timer_cnt_get();
        while ( ( (app_timer_cnt_get() - start) * 1000ULL / 32768ULL ) < until_ms ) {
            m95_poll_lines();
            nrf_delay_ms(5);
        }
*/

    // Enter main loop.
    for (;;)
    {
        // Service GPS driver: parses internally, only returns true on a NEW valid fix
        // Service GPS: parse any complete NMEA line framed by the ISR.
        if (gps_service())
        {
            gps_packet_t fix;
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
        }
        else{
       // NRF_LOG_WARNING("GPS fix invalid");
       // gps_debug_raw();
        }


        int16_t x,y,z;
        if (lis3dh_read_xyz_mg(&x,&y,&z)) {
            NRF_LOG_INFO("ACC mg: X=%d Y=%d Z=%d", x,y,z);
        } else {
            NRF_LOG_ERROR("ACC read failed");
        }


        // after battery_adc_init(), anywhere you print VBAT:
        uint16_t vbat_mv;
        if (battery_read_mv(&vbat_mv)) {
            uint8_t pct = battery_percent_from_mv(vbat_mv);
            NRF_LOG_INFO("VBAT = %u mV (%u%%)", vbat_mv, pct);
        }


        // HR Sensor

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


      // ~50 Hz polling is fine for 100 Hz sensor
        nrf_delay_ms(20);


        //HR Sensor

  //      m95_app_service();  // drains URCs / advances state

  //    if (!ran_smoke && m95_app_ready()) {
   //       ran_smoke = true;
   //       m95_quick_selftest();
   //   }

  // m95_poll_lines();   // keep printing modem URCs and responses
             nrf_delay_ms(1000);

        NRF_LOG_FLUSH();

        idle_state_handle();
    }
}


/**
 * @}
 */
