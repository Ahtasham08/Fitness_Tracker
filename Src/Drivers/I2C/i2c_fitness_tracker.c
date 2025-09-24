#include "COMMON_CONFIG.h"
#include "i2c_fitness_tracker.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
#define TWI_ADDRESSES      127

#define TMP112_INIT() twi_init()

/* TWI instance. */
const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/* Indicates if operation on TWI has ended. */
volatile bool m_xfer_done = false;

/* TWI instance. */
//const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = I2C_SCL_PIN,
       .sda                = I2C_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}



void i2c_scanner()
{ 
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

  for (address = 0x08; address <= 0x77; address++) {
        // Try writing a dummy byte (use NRF_DRV_TWI_FLAG_TX_NO_STOP)
        err_code = nrf_drv_twi_tx(&m_twi, address, &sample_data, 1, false);

        if (err_code == NRF_SUCCESS) {
            detected_device = true;
            #if DEBUG_LOG_VERBOSITY == LOG_VERBOSE
            NRF_LOG_INFO("TWI device detected at address 0x%x.", address);
            #endif
         }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
    #if DEBUG_LOG_VERBOSITY == LOG_VERBOSE
        NRF_LOG_INFO("No device was found.");
        #endif
        NRF_LOG_FLUSH();
    }
}
void configure_twi_pins_for_low_power(void)
{
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 27), NRF_GPIO_PIN_NOPULL); // SCL
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0, 26), NRF_GPIO_PIN_NOPULL); // SDA
}

void disable_twi(void)
{
    nrf_drv_twi_disable(&m_twi);
}

void enable_twi(void)
{
    nrf_drv_twi_enable(&m_twi);
}
