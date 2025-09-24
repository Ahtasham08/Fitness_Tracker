
#ifndef I2C_FITNESS_TRACKER_H
#define	I2C_FITNESS_TRACKER_H

#include "nrf_drv_twi.h"

extern const nrf_drv_twi_t m_twi;
extern volatile bool m_xfer_done;

void twi_init ();

void i2c_scanner();

void disable_twi(void);

void enable_twi(void);

#endif	/* I2C_FITNESS_TRACKER_H */

