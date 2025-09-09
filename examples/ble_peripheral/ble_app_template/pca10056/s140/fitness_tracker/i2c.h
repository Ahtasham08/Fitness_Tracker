#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "boards_custom.h"  // must define BOARD_I2C0_SCL_PIN / BOARD_I2C0_SDA_PIN
#include "nrfx_twim.h"

// Bus frequency lives here (as you wanted)
#ifndef BOARD_I2C0_FREQ
#define BOARD_I2C0_FREQ  NRF_TWIM_FREQ_100K
#endif

// Get shared TWIM0 handle (for advanced cases)
const nrfx_twim_t* i2c0_get(void);

// One-time init of the shared I2C0 (TWIM0). Safe to call multiple times.
bool i2c0_init(void);
bool i2c0_is_inited(void);

// Simple blocking helpers used by drivers
bool i2c0_tx(uint8_t addr7, const uint8_t* data, size_t len, bool no_stop);
bool i2c0_rx(uint8_t addr7, uint8_t* data, size_t len);
bool i2c0_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
bool i2c0_read_regs(uint8_t addr7, uint8_t reg, uint8_t* dst, size_t len);


//for 2nd BUS



// Expose the TWIM instance so other files (driver) can use it
extern const nrfx_twim_t HR_TWIM;

void hr_bus_init(void);                 // init TWIM1 + INT pin
bool hr_int_is_asserted(void);          // true if HR_INT (P1.11) is low