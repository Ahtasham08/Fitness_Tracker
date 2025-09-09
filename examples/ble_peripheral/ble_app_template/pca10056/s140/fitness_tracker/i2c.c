#include "sdk_config.h"   // must be first so NRFX_TWIM* macros are visible
#include "i2c.h"
#include "nrf_gpio.h"     // <-- needed for pin read
#include "app_error.h"    // <-- for APP_ERROR_CHECK

static const nrfx_twim_t m_twim0 = NRFX_TWIM_INSTANCE(0);
static bool s_inited = false;

const nrfx_twim_t* i2c0_get(void) { return &m_twim0; }
bool i2c0_is_inited(void) { return s_inited; }

bool i2c0_init(void)
{
    if (s_inited) return true;

    nrfx_twim_config_t cfg = {
        .scl                = BOARD_I2C0_SCL_PIN,
        .sda                = BOARD_I2C0_SDA_PIN,
        .frequency          = BOARD_I2C0_FREQ,
        .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .hold_bus_uninit    = false
    };

    if (nrfx_twim_init(&m_twim0, &cfg, NULL, NULL) != NRFX_SUCCESS) {
        return false;
    }
    nrfx_twim_enable(&m_twim0);
    s_inited = true;
    return true;
}

bool i2c0_tx(uint8_t addr7, const uint8_t* data, size_t len, bool no_stop)
{
    return (nrfx_twim_tx(&m_twim0, addr7, data, len, no_stop) == NRFX_SUCCESS);
}

bool i2c0_rx(uint8_t addr7, uint8_t* data, size_t len)
{
    return (nrfx_twim_rx(&m_twim0, addr7, data, len) == NRFX_SUCCESS);
}

bool i2c0_write_reg(uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c0_tx(addr7, buf, 2, false);
}

bool i2c0_read_regs(uint8_t addr7, uint8_t reg, uint8_t* dst, size_t len)
{
    if (!i2c0_tx(addr7, &reg, 1, true)) return false; // repeated start, no STOP
    return i2c0_rx(addr7, dst, len);
}



// for 2nd BUS

const nrfx_twim_t HR_TWIM = NRFX_TWIM_INSTANCE(1);

void hr_bus_init(void)
{
    // Configure INT pin as input with pull-up (active-low from MAX30102)
    nrf_gpio_cfg_input(HR_INT_PIN, NRF_GPIO_PIN_PULLUP);

    // Init TWIM1 with our HR pins
    nrfx_twim_config_t cfg = {
        .scl                = HR_I2C_SCL_PIN,
        .sda                = HR_I2C_SDA_PIN,
        .frequency          = NRF_TWIM_FREQ_400K,
        .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .hold_bus_uninit    = 0
    };
    ret_code_t err = nrfx_twim_init(&HR_TWIM, &cfg, NULL, NULL);
    APP_ERROR_CHECK(err);
    nrfx_twim_enable(&HR_TWIM);
}


// Active-low INT from MAX30102
bool hr_int_is_asserted(void)
{
    return nrf_gpio_pin_read(HR_INT_PIN) == 0;
}