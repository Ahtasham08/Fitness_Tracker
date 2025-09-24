#include "COMMON_CONFIG.h"
#include "gpio_fitness_tracker.h"


void button_handler(void);

void GPIO_INIT(void)
{
    ret_code_t err_code=0;
    nrf_gpio_cfg_output(PIN_GPS_PWR); // Pin number p0.22 for led
    nrf_gpio_cfg_output(GSM_MCU_RST); // Pin number p0.23 for led
    nrf_gpio_cfg_output(PIN_GSM_EMERG_OFF); // Pin number p1.03 for led
    nrf_gpio_cfg_output(PIN_GSM_PWRKEY); // Pin number p0.21 for led
    nrf_gpio_cfg_output(PIN_GSM_PWR); // Pin number p1.04 for led
    nrf_gpio_cfg_output(MCU_EN3V8); // Pin number p0.11 for led
    nrf_gpio_cfg_output(GSM_MCU_DTR);

    nrf_gpio_pin_set(GSM_MCU_DTR);

    nrf_gpio_pin_set(MCU_EN3V8);
    nrf_gpio_pin_clear(PIN_GPS_PWR);
    nrf_gpio_pin_clear(PIN_GSM_PWRKEY);
    nrf_gpio_pin_clear(PIN_GSM_EMERG_OFF);
    nrf_gpio_pin_set(PIN_GSM_PWR);

    
    nrf_gpio_cfg_input(GSM_MCU_STATUS,                      
            NRF_GPIO_PIN_NOPULL      // Pull-up resistor
            );

}

// Button event handler
void button_event_handler(uint8_t pin_no, uint8_t action)
{

}

// Long press timer handler
void long_press_timeout_handler(void * p_context)
{
    
}

// Multi press timer handler
void multi_press_timeout_handler(void * p_context)
{
 
}

// Debounce timer handler
void debounce_timeout_handler(void * p_context)
{
    //app_timer_start(m_long_press_timer, APP_TIMER_TICKS(LONG_PRESS_THRESHOLD_MS), NULL);  // Start long press timer
}


void check_button_on_startup(void) {
   
}

