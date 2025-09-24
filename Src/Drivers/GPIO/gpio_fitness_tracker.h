#ifndef FITNESS_TRACKER_GPIO_H
#define FITNESS_TRACKER_GPIO_H


#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "bsp.h"


void GPIO_INIT(void);

void PowerON(uint8_t GPIO);
void PowerOFF(uint8_t GPIO);
void SetLEDStatus(bool status);
bool LEDStatus();

void long_press_timeout_handler(void * p_context);
void multi_press_timeout_handler(void * p_context);
void debounce_timeout_handler(void * p_context);
void button_event_handler(uint8_t pin_no, uint8_t action);
void check_button_on_startup(void);

#endif