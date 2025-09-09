#pragma once
#include <stdint.h>
#include <stdbool.h>

// Device defaults (override via compiler flags if needed)
#ifndef LIS3DH_I2C_ADDR
#define LIS3DH_I2C_ADDR  0x19   // You probed 0x19 (SA0=1)
#endif

#ifndef LIS3DH_CS_PIN
#define LIS3DH_CS_PIN    -1     // -1 if CS is tied high on PCB
#endif

typedef enum {
    LIS3DH_ODR_POWERDOWN = 0,
    LIS3DH_ODR_1HZ   = 0x10,
    LIS3DH_ODR_10HZ  = 0x20,
    LIS3DH_ODR_25HZ  = 0x30,
    LIS3DH_ODR_50HZ  = 0x40,
    LIS3DH_ODR_100HZ = 0x50,
    LIS3DH_ODR_200HZ = 0x60,
    LIS3DH_ODR_400HZ = 0x70
} lis3dh_odr_t;

typedef enum {
    LIS3DH_FS_2G  = 0x00,
    LIS3DH_FS_4G  = 0x10,
    LIS3DH_FS_8G  = 0x20,
    LIS3DH_FS_16G = 0x30
} lis3dh_fs_t;

bool lis3dh_whoami(uint8_t* who);                                   // expect 0x33
bool lis3dh_init(lis3dh_odr_t odr, lis3dh_fs_t fs);                  // sets ODR, HR, BDU, FS
bool lis3dh_read_xyz_mg(int16_t* x_mg, int16_t* y_mg, int16_t* z_mg);// 12-bit HR → ~1 mg/LSB @ ±2g
