#include "lis3dh.h"
#include "i2c.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

// Registers (subset)
#define REG_WHOAMI   0x0F
#define REG_CTRL1    0x20
#define REG_CTRL4    0x23
#define REG_OUT_X_L  0x28

static inline void cs_high_if_needed(void)
{
    if (LIS3DH_CS_PIN >= 0) {
        nrf_gpio_cfg_output((uint32_t)LIS3DH_CS_PIN);
        nrf_gpio_pin_set((uint32_t)LIS3DH_CS_PIN);
    }
}

bool lis3dh_whoami(uint8_t* who)
{
    cs_high_if_needed();
    return i2c0_read_regs(LIS3DH_I2C_ADDR, REG_WHOAMI, who, 1);
}

bool lis3dh_init(lis3dh_odr_t odr, lis3dh_fs_t fs)
{
    if (!i2c0_is_inited() && !i2c0_init()) return false;
    cs_high_if_needed();

    // CTRL1: ODR + enable X/Y/Z (LPen=0)
    uint8_t ctrl1 = ((uint8_t)odr & 0xF0) | 0x07;
    // CTRL4: BDU=1 (bit7), HR=1 (bit3), FS per arg
    uint8_t ctrl4 = 0x80 /*BDU*/ | 0x08 /*HR*/ | (fs & 0x30);

    if (!i2c0_write_reg(LIS3DH_I2C_ADDR, REG_CTRL1, ctrl1)) return false;
    if (!i2c0_write_reg(LIS3DH_I2C_ADDR, REG_CTRL4, ctrl4)) return false;

    nrf_delay_ms(2);
    return true;
}

bool lis3dh_read_xyz_mg(int16_t* x_mg, int16_t* y_mg, int16_t* z_mg)
{
    cs_high_if_needed();

    uint8_t start = REG_OUT_X_L | 0x80; // auto-increment
    uint8_t raw[6];
    if (!i2c0_read_regs(LIS3DH_I2C_ADDR, start, raw, sizeof raw)) return false;

    // 16-bit little-endian words, HR (12-bit) left-justified → shift right 4
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]) >> 4;
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]) >> 4;
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]) >> 4;

    if (x_mg) *x_mg = x;
    if (y_mg) *y_mg = y;
    if (z_mg) *z_mg = z;
    return true;
}
